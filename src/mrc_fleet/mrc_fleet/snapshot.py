"""Save documentation snapshots of the running system.

  ros2 run mrc_fleet snapshot <stage_name> [--out DIR] [--no-camera] [--no-ops]

Writes into DIR (default ~/multi_robot_ws/snapshots):
  <stage>_overhead.png  top-down Gazebo camera (/overhead_camera/image)
  <stage>_ops.png       'operations view': map, zones, coverage, robots, peer discs,
                        formation slots, live costmap of each robot
  <stage>_gazebo.png / <stage>_rviz.png   window captures (only with --windows; X11 +
                        xdotool + ImageMagick 'import' required)
"""
import argparse
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from mrc_fleet.fleet_config import load_fleet

ROBOT_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']


def grid_to_array(msg: OccupancyGrid) -> np.ndarray:
    return np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


def grid_extent(msg: OccupancyGrid):
    i = msg.info
    x0, y0 = i.origin.position.x, i.origin.position.y
    return [x0, x0 + i.width * i.resolution, y0, y0 + i.height * i.resolution]


class SnapshotNode(Node):
    def __init__(self, fleet):
        super().__init__('mrc_snapshot')
        self.fleet = fleet
        self.image = None
        self.map = None
        self.coverage = None
        self.poses = None
        self.status = None
        self.zones = None
        self.formation = None
        self.costmaps = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/overhead_camera/image', self._img, 1)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), latched)
        self.create_subscription(OccupancyGrid, '/fleet/coverage_grid',
                                 lambda m: setattr(self, 'coverage', m), 1)
        self.create_subscription(PoseArray, '/fleet/poses', lambda m: setattr(self, 'poses', m), 1)
        self.create_subscription(String, '/fleet/coverage_status',
                                 lambda m: setattr(self, 'status', m.data), latched)
        self.create_subscription(MarkerArray, '/fleet/zones',
                                 lambda m: setattr(self, 'zones', m), latched)
        self.create_subscription(MarkerArray, '/fleet/formation_markers',
                                 lambda m: setattr(self, 'formation', m), 1)
        for r in fleet:
            self.create_subscription(
                OccupancyGrid, f'/{r.name}/global_costmap/costmap',
                lambda m, n=r.name: self.costmaps.__setitem__(n, m), 1)

    def _img(self, msg):
        self.image = msg


def save_overhead(msg: Image, path: str):
    from PIL import Image as PILImage
    ch = 3 if msg.encoding in ('rgb8', 'bgr8') else 4
    arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    arr = arr.reshape(msg.height, msg.width, ch)[:, :, :3]
    if msg.encoding.startswith('bgr'):
        arr = arr[:, :, ::-1]
    PILImage.fromarray(arr).save(path)


def capture_windows(stage: str, out: str):
    """Grab the Gazebo GUI and RViz windows (X11 only). Returns the written paths."""
    import subprocess
    written = []
    for title, suffix in (('Gazebo Sim', 'gazebo'), ('RViz', 'rviz')):
        path = os.path.join(out, f'{stage}_{suffix}.png')
        for attempt in (1, 2):      # the desktop is live: a window can be briefly unmapped
            try:
                ids = subprocess.run(['xdotool', 'search', '--onlyvisible', '--name', title],
                                     capture_output=True, text=True, timeout=5).stdout.split()
                if not ids:
                    raise RuntimeError('window not visible')
                subprocess.run(['import', '-window', ids[0], path], check=True, timeout=20,
                               capture_output=True)
                written.append(path)
                break
            except (OSError, RuntimeError, subprocess.SubprocessError) as e:
                print(f'WINDOW CAPTURE {title} attempt {attempt} failed: {e}')
                time.sleep(1.0)
    return written


def save_ops_view(node: SnapshotNode, stage: str, path: str):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    fig, ax = plt.subplots(figsize=(11, 9), dpi=110)
    if node.map is not None:
        m = grid_to_array(node.map)
        img = np.full(m.shape + (3,), 0.55)
        img[m == 0] = 1.0
        img[m >= 65] = 0.1
        ax.imshow(img, origin='lower', extent=grid_extent(node.map), interpolation='nearest')
    # inflated costmap of the first robot (shows walls + peer discs + inflation)
    if node.costmaps:
        name = next(r.name for r in node.fleet if r.name in node.costmaps)
        c = grid_to_array(node.costmaps[name]).astype(float)
        c[c <= 0] = np.nan
        ax.imshow(c, origin='lower', extent=grid_extent(node.costmaps[name]), cmap='magma_r',
                  alpha=0.35, vmin=0, vmax=100, interpolation='nearest')
    if node.coverage is not None:
        cg = grid_to_array(node.coverage).astype(float)
        rgba = np.zeros(cg.shape + (4,))
        rgba[cg == 0] = (1.0, 0.85, 0.2, 0.25)   # coverable, not yet covered
        for k in range(len(node.fleet)):
            col = matplotlib.colors.to_rgb(ROBOT_COLORS[k % len(ROBOT_COLORS)])
            rgba[cg == 10 * (k + 1)] = (*col, 0.45)
        ax.imshow(rgba, origin='lower', extent=grid_extent(node.coverage), interpolation='nearest')
    if node.zones is not None:
        for mk in node.zones.markers:
            if mk.type == mk.LINE_STRIP and len(mk.points) > 1:
                xs = [p.x for p in mk.points]
                ys = [p.y for p in mk.points]
                ax.plot(xs, ys, color=(mk.color.r, mk.color.g, mk.color.b),
                        lw=2.2 if mk.ns == 'zone_outline' else 0.8,
                        alpha=0.9 if mk.ns == 'zone_outline' else 0.6)
            elif mk.type == mk.TEXT_VIEW_FACING:
                ax.text(mk.pose.position.x, mk.pose.position.y, mk.text, ha='center',
                        va='center', fontsize=9, weight='bold',
                        bbox=dict(fc='white', ec='none', alpha=0.7))
    if node.formation is not None:
        for mk in node.formation.markers:
            if mk.ns == 'slots':
                ax.add_patch(Circle((mk.pose.position.x, mk.pose.position.y), 0.15, fill=False,
                                    ls='--', lw=1.5, color='k'))
            elif mk.ns == 'links' and len(mk.points) == 2:
                ax.plot([mk.points[0].x, mk.points[1].x], [mk.points[0].y, mk.points[1].y],
                        'k:', lw=1)
    if node.poses is not None:
        for k, p in enumerate(node.poses.poses):
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            col = ROBOT_COLORS[k % len(ROBOT_COLORS)]
            ax.add_patch(Circle((p.position.x, p.position.y), 0.22, color=col, alpha=0.9))
            tip = (p.position.x + 0.3 * math.cos(yaw), p.position.y + 0.3 * math.sin(yaw))
            ax.add_patch(Polygon([tip, (p.position.x + 0.15 * math.cos(yaw + 2.4),
                                        p.position.y + 0.15 * math.sin(yaw + 2.4)),
                                  (p.position.x + 0.15 * math.cos(yaw - 2.4),
                                   p.position.y + 0.15 * math.sin(yaw - 2.4))], color='k'))
            if k < len(node.fleet):
                ax.text(p.position.x, p.position.y + 0.35, node.fleet[k].name, ha='center',
                        fontsize=9, color=col, weight='bold')
    title = f'Stage: {stage}'
    if node.status:
        title += f'\n{node.status[:160]}'
    ax.set_title(title, fontsize=10)
    ax.set_xlabel('x [m] (map)')
    ax.set_ylabel('y [m] (map)')
    ax.set_aspect('equal')
    if node.map is not None:
        ax.set_xlim(grid_extent(node.map)[:2])
        ax.set_ylim(grid_extent(node.map)[2:])
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage')
    parser.add_argument('--out', default=os.path.expanduser('~/multi_robot_ws/snapshots'))
    parser.add_argument('--fleet', default='')
    parser.add_argument('--wait', type=float, default=8.0, help='seconds to collect data')
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--no-ops', action='store_true')
    parser.add_argument('--windows', action='store_true',
                        help='also capture the Gazebo and RViz windows (X11)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SnapshotNode(load_fleet(args.fleet))
    os.makedirs(args.out, exist_ok=True)
    deadline = time.time() + args.wait
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        have_all = (args.no_camera or node.image is not None) and node.map is not None \
            and node.poses is not None and len(node.costmaps) == len(node.fleet)
        if have_all and time.time() > deadline - args.wait / 2:
            break
    written = []
    if not args.no_camera:
        if node.image is None:
            node.get_logger().warn('no /overhead_camera/image received')
        else:
            p = os.path.join(args.out, f'{args.stage}_overhead.png')
            save_overhead(node.image, p)
            written.append(p)
    if not args.no_ops:
        p = os.path.join(args.out, f'{args.stage}_ops.png')
        save_ops_view(node, args.stage, p)
        written.append(p)
    if args.windows:
        written += capture_windows(args.stage, args.out)
    for p in written:
        print(f'saved {p}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
