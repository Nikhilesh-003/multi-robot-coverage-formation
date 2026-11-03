"""End-to-end mission runner + integration test.

  ros2 run mrc_fleet demo [--snapshots DIR] [--skip-coverage] [--skip-formation]

1. waits until every robot is localized and its Nav2 is up
2. runs the coverage mission (/fleet/start_coverage) until complete + robots home
3. runs the formation mission (/fleet/start_formation) until done
4. takes a snapshot at every stage and writes plots (trajectories, formation error)
Exit code 0 only if every stage succeeded (coverage >= --min-coverage).
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from mrc_fleet.fleet_config import load_fleet
from mrc_fleet.snapshot import ROBOT_COLORS


class Demo(Node):
    def __init__(self, fleet):
        super().__init__('mrc_demo')
        self.fleet = fleet
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.health, self.cov_status, self.form_state = {}, '', ''
        self.form_err = []          # (t, [errors])
        self.traj = {r.name: [] for r in fleet}   # name -> [(t, x, y, phase)]
        self.phase = 'startup'
        self.t0 = time.time()
        self.create_subscription(String, '/fleet/health',
                                 lambda m: setattr(self, 'health', json.loads(m.data)), 1)
        self.create_subscription(String, '/fleet/coverage_status',
                                 lambda m: setattr(self, 'cov_status', m.data), latched)
        self.create_subscription(String, '/fleet/formation_state',
                                 lambda m: setattr(self, 'form_state', m.data), 10)
        self.create_subscription(Float32MultiArray, '/fleet/formation_error', self.on_err, 10)
        self.create_subscription(PoseArray, '/fleet/poses', self.on_poses, 1)
        self.start_cov = self.create_client(Trigger, '/fleet/start_coverage')
        self.start_form = self.create_client(Trigger, '/fleet/start_formation')
        self._last_pose_t = 0.0

    def on_err(self, msg):
        if self.form_state in ('march', 'settle'):
            self.form_err.append((time.time() - self.t0, list(msg.data)))

    def on_poses(self, msg):
        t = time.time() - self.t0
        if t - self._last_pose_t < 0.5:
            return
        self._last_pose_t = t
        for r, p in zip(self.fleet, msg.poses):
            self.traj[r.name].append((t, p.position.x, p.position.y, self.phase))

    # ------------------------------------------------------------------ helpers
    def spin_until(self, cond, timeout, what):
        end = time.time() + timeout
        last_log = 0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
            if time.time() - last_log > 15:
                last_log = time.time()
                self.get_logger().info(f'waiting for {what} ... {self.short_status()}')
        self.get_logger().error(f'TIMEOUT ({timeout:.0f}s) waiting for {what}')
        return False

    def short_status(self):
        return (self.cov_status[:140] if self.phase.startswith('coverage') or not self.form_state
                else f'formation={self.form_state}')

    def call(self, client, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'service {client.srv_name} not available'
        fut = client.call_async(Trigger.Request())
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done():
            return False, 'service call timed out'
        return fut.result().success, fut.result().message

    def coverage_pct(self):
        m = re.search(r'coverage ([\d.]+)%', self.cov_status)
        return float(m.group(1)) if m else 0.0


def snapshot(node, stage, out, extra=()):
    """Run the snapshot tool in a child process while this node keeps spinning
    (so formation-error / trajectory recording has no gaps)."""
    cmd = ['ros2', 'run', 'mrc_fleet', 'snapshot', stage, '--out', out, *extra]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    end = time.time() + 60
    while proc.poll() is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if proc.poll() is None:
        proc.kill()
    out_txt = proc.communicate()[0] or ''
    print(out_txt.strip()[-1500:])


def plot_results(node: Demo, out: str):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    written = []
    if node.form_err:
        t0 = node.form_err[0][0]
        fig, ax = plt.subplots(figsize=(9, 4), dpi=110)
        n = len(node.form_err[0][1])
        for k in range(n):
            ax.plot([t - t0 for t, e in node.form_err if len(e) > k],
                    [e[k] for t, e in node.form_err if len(e) > k],
                    color=ROBOT_COLORS[(k + 1) % len(ROBOT_COLORS)],
                    label=f'follower {node.fleet[k + 1].name if k + 1 < len(node.fleet) else k}')
        ax.set_xlabel('time since march start [s] (wall clock)')
        ax.set_ylabel('distance to formation slot [m]')
        ax.set_title('Leader-follower formation tracking error')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        p = os.path.join(out, '10_formation_tracking_error.png')
        fig.savefig(p)
        plt.close(fig)
        written.append(p)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=110)
    for ax, phase in zip(axes, ('coverage', 'formation')):
        for k, r in enumerate(node.fleet):
            pts = [(x, y) for _, x, y, ph in node.traj[r.name] if ph.startswith(phase)]
            if pts:
                ax.plot(*zip(*pts), color=ROBOT_COLORS[k % len(ROBOT_COLORS)], lw=1.5,
                        label=r.name)
                ax.plot(*pts[0], 'o', color=ROBOT_COLORS[k % len(ROBOT_COLORS)])
        ax.set_title(f'{phase} trajectories (map frame)')
        ax.set_xlim(-5.2, 5.2)
        ax.set_ylim(-4.2, 4.2)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    p = os.path.join(out, '11_trajectories.png')
    fig.savefig(p)
    plt.close(fig)
    written.append(p)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--snapshots', default=os.path.expanduser('~/multi_robot_ws/snapshots'))
    ap.add_argument('--fleet', default='')
    ap.add_argument('--skip-coverage', action='store_true')
    ap.add_argument('--skip-formation', action='store_true')
    ap.add_argument('--no-snapshots', action='store_true')
    ap.add_argument('--windows', action='store_true',
                    help='snapshots also capture the Gazebo/RViz windows (X11)')
    ap.add_argument('--min-coverage', type=float, default=90.0)
    ap.add_argument('--coverage-timeout', type=float, default=1500.0)
    ap.add_argument('--formation-timeout', type=float, default=600.0)
    args, ros_args = ap.parse_known_args()
    os.makedirs(args.snapshots, exist_ok=True)
    rclpy.init(args=ros_args)
    fleet = load_fleet(args.fleet)
    node = Demo(fleet)
    snap = (lambda *a, **k: None) if args.no_snapshots else \
        (lambda stage, *extra: snapshot(node, stage, args.snapshots,
                                        extra + (('--windows',) if args.windows else ())))
    log = node.get_logger()
    results = {}
    ok = True
    try:
        # ---------------------------------------------------------- readiness
        node.phase = 'startup'
        ready = node.spin_until(
            lambda: len(node.health) == len(fleet)
            and all(v is not None and v < 1.0 for v in node.health.values())
            and 'mission=ready' in node.cov_status
            and all(f'{r.name}:idle' in node.cov_status for r in fleet),
            300, 'all robots localized + Nav2 active')
        if not ready:
            raise RuntimeError('fleet not ready')
        log.info('fleet READY')
        snap('01_fleet_ready')

        # ----------------------------------------------------------- coverage
        if not args.skip_coverage:
            node.phase = 'coverage'
            success, msg = node.call(node.start_cov)
            log.info(f'start_coverage: {success} {msg}')
            if not success:
                raise RuntimeError(msg)
            t_cov = time.time()
            node.spin_until(lambda: time.time() - t_cov > 20, 25, 'zone assignment')
            snap('02_coverage_zones_assigned')
            node.spin_until(lambda: node.coverage_pct() >= 50 or
                            'mission=complete' in node.cov_status, args.coverage_timeout,
                            'coverage 50%')
            snap('03_coverage_midway')
            done = node.spin_until(lambda: 'mission=complete' in node.cov_status,
                                   args.coverage_timeout, 'coverage complete')
            snap('04_coverage_complete')
            pct = node.coverage_pct()
            results['coverage_pct'] = pct
            results['coverage_wall_s'] = round(time.time() - t_cov, 1)
            results['coverage_status'] = node.cov_status
            if not done or pct < args.min_coverage:
                ok = False
                log.error(f'coverage FAILED: done={done} pct={pct}')
            else:
                log.info(f'coverage OK: {pct}%')
            node.phase = 'coverage_return'
            homed = node.spin_until(
                lambda: all(f'{r.name}:done' in node.cov_status for r in fleet), 240,
                'robots back home')
            snap('05_robots_home')
            ok &= homed

        # ---------------------------------------------------------- formation
        if not args.skip_formation:
            node.phase = 'formation_assemble'
            success, msg = node.call(node.start_form)
            log.info(f'start_formation: {success} {msg}')
            if not success:
                raise RuntimeError(msg)
            t_form = time.time()
            if node.spin_until(lambda: node.form_state in ('march', 'done', 'fault'), 240,
                               'formation assembled'):
                node.phase = 'formation'
                node.spin_until(lambda: time.time() - t_form > 0, 1, '')
                snap('06_formation_assembled')
                t_m = time.time()
                node.spin_until(lambda: time.time() - t_m > 30 or node.form_state != 'march',
                                40, 'formation marching')
                snap('07_formation_march')
                node.spin_until(lambda: time.time() - t_m > 62 or node.form_state != 'march',
                                40, 'formation turning')
                snap('08_formation_uturn')
            done = node.spin_until(lambda: node.form_state in ('done', 'fault'),
                                   args.formation_timeout, 'formation done')
            snap('09_formation_complete')
            results['formation_state'] = node.form_state
            results['formation_wall_s'] = round(time.time() - t_form, 1)
            errs = [e for _, es in node.form_err for e in es if not math.isnan(e)]
            if errs:
                results['formation_mean_err_m'] = round(sum(errs) / len(errs), 3)
                results['formation_max_err_m'] = round(max(errs), 3)
            if not done or node.form_state != 'done':
                ok = False
                log.error(f'formation FAILED: state={node.form_state}')
            else:
                log.info('formation OK')
    except (RuntimeError, KeyboardInterrupt) as e:
        ok = False
        log.error(f'demo aborted: {e}')
    finally:
        for p in plot_results(node, args.snapshots):
            print(f'saved {p}')
        results['success'] = bool(ok)
        with open(os.path.join(args.snapshots, 'demo_results.json'), 'w') as f:
            json.dump(results, f, indent=2)
        print(json.dumps(results, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
