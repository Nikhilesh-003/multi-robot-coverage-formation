"""Pure-python coverage planning (no ROS): cells -> zones -> boustrophedon lanes.

Coordinates: the occupancy grid uses ROS conventions (row 0 = lowest y, values
-1 unknown / 0..100 occupancy). A *coverage cell* is a square of `cell_size`
metres; it is coverable when its centre has at least `clearance` metres to the
nearest non-free map cell, i.e. the robot can drive through its centre.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class CellGrid:
    coverable: np.ndarray          # bool [rows, cols]
    cell_size: float
    origin_x: float                # world x of the grid's lower-left corner
    origin_y: float

    @property
    def shape(self):
        return self.coverable.shape

    def center(self, r: int, c: int) -> Tuple[float, float]:
        return (self.origin_x + (c + 0.5) * self.cell_size,
                self.origin_y + (r + 0.5) * self.cell_size)

    def cell_of(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        c = int(math.floor((x - self.origin_x) / self.cell_size))
        r = int(math.floor((y - self.origin_y) / self.cell_size))
        if 0 <= r < self.shape[0] and 0 <= c < self.shape[1]:
            return r, c
        return None


@dataclass
class Goal:
    """One navigation goal. `cells` are the coverage cells of the lane this goal belongs
    to; if all of them are already covered when the goal comes up, it is skipped."""
    x: float
    y: float
    yaw: float = 0.0
    cells: List[Tuple[int, int]] = field(default_factory=list)
    attempts: int = 0


@dataclass
class Zone:
    zid: int
    cells: List[Tuple[int, int]]
    bounds: Tuple[float, float, float, float]   # x_min, y_min, x_max, y_max (world)
    goals: List[Goal] = field(default_factory=list)
    owner: Optional[str] = None
    done: bool = False


def build_cell_grid(occ: np.ndarray, resolution: float, origin: Tuple[float, float],
                    cell_size: float, clearance: float, free_thresh: int = 25) -> CellGrid:
    """occ: int array [h, w] from nav_msgs/OccupancyGrid (row 0 = lowest y)."""
    free = (occ >= 0) & (occ <= free_thresh)
    # pad with a non-free border so the edge of the map counts as an obstacle
    dist = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * resolution
    h, w = occ.shape
    rows = int(math.floor(h * resolution / cell_size))
    cols = int(math.floor(w * resolution / cell_size))
    cov = np.zeros((rows, cols), bool)
    for r in range(rows):
        py = int(((r + 0.5) * cell_size) / resolution)
        for c in range(cols):
            px = int(((c + 0.5) * cell_size) / resolution)
            if py < h and px < w:
                cov[r, c] = dist[py, px] >= clearance
    return CellGrid(cov, cell_size, origin[0], origin[1])


def _split_equal(counts: np.ndarray, parts: int) -> List[int]:
    """Indices that split a 1-D histogram into `parts` runs of ~equal mass.
    Returns parts+1 boundaries (start inclusive, end exclusive)."""
    total = counts.sum()
    cum = np.cumsum(counts)
    bounds = [0]
    for k in range(1, parts):
        idx = int(np.searchsorted(cum, total * k / parts)) + 1
        bounds.append(max(bounds[-1] + 1, min(idx, len(counts) - (parts - k))))
    bounds.append(len(counts))
    return bounds


def partition_zones(grid: CellGrid, n_cols: int, n_rows: int) -> List[Zone]:
    """Split the coverable cells into n_cols x n_rows zones of ~equal area:
    first vertical strips of equal coverable area, then each strip into rows."""
    cov = grid.coverable
    zones: List[Zone] = []
    if not cov.any():
        return zones
    col_counts = cov.sum(axis=0)
    nz_cols = np.nonzero(col_counts)[0]
    c0, c1 = nz_cols[0], nz_cols[-1] + 1
    cb = [c0 + b for b in _split_equal(col_counts[c0:c1], n_cols)]
    for i in range(n_cols):
        strip = cov[:, cb[i]:cb[i + 1]]
        row_counts = strip.sum(axis=1)
        nz_rows = np.nonzero(row_counts)[0]
        if len(nz_rows) == 0:
            continue
        r0, r1 = nz_rows[0], nz_rows[-1] + 1
        rb = [r0 + b for b in _split_equal(row_counts[r0:r1], n_rows)]
        for j in range(n_rows):
            cells = [(r, c) for r in range(rb[j], rb[j + 1])
                     for c in range(cb[i], cb[i + 1]) if cov[r, c]]
            if not cells:
                continue
            x0, y0 = grid.center(rb[j], cb[i])
            x1, y1 = grid.center(rb[j + 1] - 1, cb[i + 1] - 1)
            h = grid.cell_size / 2
            zones.append(Zone(len(zones), cells, (x0 - h, y0 - h, x1 + h, y1 + h)))
    return zones


def boustrophedon(grid: CellGrid, cells: List[Tuple[int, int]],
                  direction: str = 'auto') -> List[Goal]:
    """Lawn-mower goals over `cells`: lanes along x ('x') or y ('y'); 'auto' runs the
    lanes along the longer side of the zone to minimise turns. Each contiguous run
    of cells in a lane yields a start goal and an end goal."""
    if not cells:
        return []
    rs = [r for r, _ in cells]
    cs = [c for _, c in cells]
    if direction == 'auto':
        direction = 'x' if (max(cs) - min(cs)) >= (max(rs) - min(rs)) else 'y'
    cellset = set(cells)
    lanes = sorted(set(rs if direction == 'x' else cs))
    goals: List[Goal] = []
    flip = False
    for lane in lanes:
        along = sorted(c for r, c in cells if r == lane) if direction == 'x' \
            else sorted(r for r, c in cells if c == lane)
        # split into contiguous runs
        runs, run = [], [along[0]]
        for a in along[1:]:
            if a == run[-1] + 1:
                run.append(a)
            else:
                runs.append(run)
                run = [a]
        runs.append(run)
        if flip:
            runs = [list(reversed(rn)) for rn in reversed(runs)]
        for rn in runs:
            seg = [(lane, a) if direction == 'x' else (a, lane) for a in rn]
            assert all(s in cellset for s in seg)
            sx, sy = grid.center(*seg[0])
            ex, ey = grid.center(*seg[-1])
            # BOTH ends carry the whole lane: the start goal must not be skipped just
            # because a transiting robot happened to cover the first cell, otherwise the
            # robot would drive to the lane end by any path and leave the lane unswept.
            goals.append(Goal(sx, sy, cells=list(seg)))
            if len(seg) > 1:
                goals.append(Goal(ex, ey, cells=seg))
        flip = not flip
    orient_goals(goals)
    return goals


def orient_goals(goals: List[Goal]) -> None:
    """Point each goal toward the next one (last goal keeps the previous heading)."""
    for k, g in enumerate(goals):
        if k + 1 < len(goals):
            nx, ny = goals[k + 1].x - g.x, goals[k + 1].y - g.y
            if math.hypot(nx, ny) > 1e-6:
                g.yaw = math.atan2(ny, nx)
                continue
        if k > 0:
            g.yaw = goals[k - 1].yaw


def path_length(goals: List[Goal], start: Optional[Tuple[float, float]] = None) -> float:
    pts = ([start] if start else []) + [(g.x, g.y) for g in goals]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def nearest_neighbour_order(points: List[Tuple[float, float]],
                            start: Tuple[float, float]) -> List[int]:
    """Greedy nearest-neighbour visiting order of `points` starting from `start`."""
    left = list(range(len(points)))
    order, cur = [], start
    while left:
        k = min(left, key=lambda i: math.hypot(points[i][0] - cur[0], points[i][1] - cur[1]))
        order.append(k)
        left.remove(k)
        cur = points[k]
    return order
