import os

import numpy as np
import pytest

from mrc_fleet.coverage_planner import (_split_equal, boustrophedon, build_cell_grid,
                                        partition_zones)

ARENA_PGM = os.path.join(os.path.dirname(__file__), '..', '..', 'mrc_gazebo', 'maps',
                         'coverage_arena.pgm')


def load_pgm_as_occ(path):
    """Same conversion as nav2_map_server in 'trinary' mode, using the map's YAML."""
    import yaml
    meta = yaml.safe_load(open(path[:-4] + '.yaml'))
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    img = np.flipud(img)                       # ROS grid: row 0 = lowest y
    p_occ = 1.0 - img / 255.0
    occ = np.full(img.shape, -1, np.int16)
    occ[p_occ > meta['occupied_thresh']] = 100
    occ[p_occ < meta['free_thresh']] = 0
    return occ


def test_arena_map_has_unknown_margin():
    occ = load_pgm_as_occ(ARENA_PGM)
    assert occ[0, 0] == -1 and occ[-1, -1] == -1, 'margin outside the walls must be unknown'
    assert (occ == 0).sum() > 0 and (occ == 100).sum() > 0


def test_map_edge_counts_as_obstacle():
    occ = np.zeros((40, 40), np.int16)          # all free, no walls at all
    g = build_cell_grid(occ, 0.05, (0.0, 0.0), 0.5, clearance=0.35)
    assert not g.coverable[0, :].any() and not g.coverable[:, -1].any()


def empty_room(w=100, h=80):
    occ = np.zeros((h, w), np.int16)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = 100
    return occ


def test_split_equal_monotonic_and_complete():
    for counts in (np.ones(10), np.array([5, 0, 0, 1, 1, 1, 9, 2]), np.ones(3)):
        for parts in (1, 2, 3):
            b = _split_equal(counts, parts)
            assert b[0] == 0 and b[-1] == len(counts)
            assert all(b2 > b1 for b1, b2 in zip(b, b[1:]))
            assert len(b) == parts + 1


def test_clearance_excludes_wall_cells():
    g = build_cell_grid(empty_room(), 0.05, (0.0, 0.0), 0.5, clearance=0.3)
    assert not g.coverable[0, :].any() and not g.coverable[:, 0].any()
    assert g.coverable[3:-3, 3:-3].all()


def test_zones_disjoint_and_cover_everything():
    g = build_cell_grid(empty_room(), 0.05, (0.0, 0.0), 0.5, clearance=0.3)
    zones = partition_zones(g, 3, 2)
    assert len(zones) == 6
    all_cells = [c for z in zones for c in z.cells]
    assert len(all_cells) == len(set(all_cells)), 'a cell belongs to two zones'
    assert set(all_cells) == set(zip(*np.nonzero(g.coverable)))
    sizes = [len(z.cells) for z in zones]
    assert max(sizes) - min(sizes) <= 0.35 * max(sizes), sizes


def test_boustrophedon_visits_every_cell_once():
    g = build_cell_grid(empty_room(), 0.05, (0.0, 0.0), 0.5, clearance=0.3)
    zone = partition_zones(g, 2, 1)[0]
    goals = boustrophedon(g, zone.cells)
    assert set(c for gl in goals for c in gl.cells) == set(zone.cells)
    # start and end goal of a lane carry the same (whole-lane) cells
    for a, b in zip(goals[::2], goals[1::2]):
        if len(a.cells) > 1:
            assert a.cells == b.cells
    # alternating lanes: consecutive goals never jump more than one lane + one run
    for a, b in zip(goals, goals[1:]):
        assert abs(a.x - b.x) < 6 or abs(a.y - b.y) < 0.51


def test_lanes_split_around_obstacle():
    occ = empty_room()
    occ[30:50, 45:55] = 100                       # a pillar in the middle
    g = build_cell_grid(occ, 0.05, (0.0, 0.0), 0.5, clearance=0.3)
    goals = boustrophedon(g, list(zip(*np.nonzero(g.coverable))), direction='x')
    for gl in goals:
        r, c = g.cell_of(gl.x, gl.y)
        assert g.coverable[r, c]


@pytest.mark.skipif(not os.path.exists(ARENA_PGM), reason='arena map not found')
def test_real_arena_map():
    occ = load_pgm_as_occ(ARENA_PGM)
    g = build_cell_grid(occ, 0.05, (-5.65, -4.65), 0.6, clearance=0.35)
    zones = partition_zones(g, 3, 2)
    assert len(zones) == 6
    n = int(g.coverable.sum())
    assert 120 < n < 260, n                      # ~10x8 m arena at 0.6 m cells
    for z in zones:
        goals = boustrophedon(g, z.cells)
        assert goals and all(g.coverable[g.cell_of(gl.x, gl.y)] for gl in goals)


def test_nearest_neighbour_order():
    from mrc_fleet.coverage_planner import nearest_neighbour_order
    pts = [(5, 0), (1, 0), (3, 0)]
    assert nearest_neighbour_order(pts, (0, 0)) == [1, 2, 0]
