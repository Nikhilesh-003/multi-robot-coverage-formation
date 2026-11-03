"""Central coverage coordinator.

1. Reads /map, builds coverage cells (cell_size) with a clearance margin and splits
   them into zone_cols x zone_rows zones of ~equal area, each with boustrophedon lanes.
2. Dynamic assignment: whenever a robot is idle it gets the unassigned zone whose
   nearest end is closest to it (lane order reversed if the far end is nearer).
   When no zone is left, an idle robot steals the tail half of the busiest robot's
   remaining goals. A zone is owned by exactly one robot -> no duplicated work.
3. Goals are sent one by one through each robot's namespaced Nav2
   (/<robot>/navigate_to_pose). Goals whose cells were already covered (by anyone,
   e.g. while transiting) are skipped. Failed goals are retried later, then dropped.
4. Coverage is tracked from the robots' real TF poses, published as
   /fleet/coverage_grid (-1 n/a, 0 uncovered, 10*(k+1) covered by robot k).
5. Robots whose pose goes stale or whose Nav2 disappears are marked OFFLINE and
   their remaining goals go back into the pool for the others.

Services: /fleet/start_coverage, /fleet/stop_coverage (std_srvs/Trigger)
"""
import json
import math
import os
import time
from enum import Enum

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mrc_fleet.coverage_planner import (Goal, Zone, boustrophedon, build_cell_grid,
                                        nearest_neighbour_order, orient_goals,
                                        partition_zones, path_length)
from mrc_fleet.fleet_config import load_fleet
from mrc_fleet.fleet_tf import FleetPoses, quat_from_yaw, spin_node

COLORS = [(0.90, 0.10, 0.29), (0.24, 0.71, 0.29), (0.26, 0.39, 0.85),
          (0.96, 0.51, 0.19), (0.57, 0.12, 0.71), (0.26, 0.83, 0.96)]


class RState(Enum):
    WAITING = 'waiting'      # Nav2 / pose not available yet
    IDLE = 'idle'
    BUSY = 'busy'            # a goal is active
    CANCELING = 'canceling'  # waiting for Nav2 to confirm a cancel before the next goal
    HOMING = 'homing'
    DONE = 'done'
    OFFLINE = 'offline'


class Robot:
    def __init__(self, node, spec, index):
        self.spec = spec
        self.name = spec.name
        self.index = index
        self.client = ActionClient(node, NavigateToPose, f'/{spec.name}/navigate_to_pose')
        self.state = RState.WAITING
        self.queue = []          # list[Goal]
        self.zone = None         # Zone currently owned
        self.goal = None         # Goal being executed
        self.goal_handle = None
        self.goal_sent_at = 0.0
        self.goal_seq = 0        # guards against late callbacks of cancelled goals
        self.distance = 0.0
        self.last_xy = None
        self.cells_covered = 0
        self.goals_done = 0
        self.goals_failed = 0
        self.zones_done = []


class CoverageCoordinator(Node):
    def __init__(self):
        super().__init__('coverage_coordinator')
        p = self.declare_parameter
        p('fleet_config', '')
        p('cell_size', 0.6)          # lane spacing == effective tool width [m]
        p('clearance', 0.35)         # min distance of a lane to obstacles [m]
        p('coverage_radius', 0.32)   # cell counted covered if robot centre within this
        p('zone_cols', 3)
        p('zone_rows', 2)
        p('lane_direction', 'auto')
        p('auto_start', False)
        p('return_home', True)
        p('goal_timeout', 90.0)      # s per goal (sim time)
        p('max_attempts', 3)
        p('min_steal_goals', 4)
        p('offline_after', 5.0)      # s without pose -> robot offline
        p('sweep_up', True)          # final pass over cells that are still uncovered
        p('report_path', os.path.expanduser('~/multi_robot_ws/snapshots/coverage_report.json'))
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.cell_size = gp('cell_size')
        self.clearance = gp('clearance')
        self.cov_radius = gp('coverage_radius')
        self.zone_cols, self.zone_rows = gp('zone_cols'), gp('zone_rows')
        self.lane_dir = gp('lane_direction')
        self.return_home = gp('return_home')
        self.goal_timeout = gp('goal_timeout')
        self.max_attempts = gp('max_attempts')
        self.min_steal = gp('min_steal_goals')
        self.offline_after = gp('offline_after')
        self.report_path = gp('report_path')
        self.sweep_up = gp('sweep_up')
        self.sweep_done = False

        self.fleet = load_fleet(gp('fleet_config'))
        self.robots = [Robot(self, s, k) for k, s in enumerate(self.fleet)]
        # poses come from fleet_monitor (/fleet/poses), not a private TF listener
        self.tf = FleetPoses(self, [r.name for r in self.robots])

        self.grid = None
        self.zones = []
        self.covered_by = None       # int [rows, cols]: -1 none, k robot index
        self.visits = None           # how many different robots drove over each cell
        self.visited_sets = None
        self.mission = 'waiting_for_map'
        self.t_start = None
        self.t_end = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/fleet/coverage_grid', 1)
        self.pub_zones = self.create_publisher(MarkerArray, '/fleet/zones', latched)
        self.pub_status = self.create_publisher(String, '/fleet/coverage_status', latched)
        self.create_service(Trigger, '/fleet/start_coverage', self.srv_start)
        self.create_service(Trigger, '/fleet/stop_coverage', self.srv_stop)
        self.auto_start = gp('auto_start')

        self.create_timer(0.2, self.track_coverage)      # 5 Hz coverage bookkeeping
        self.create_timer(0.5, self.schedule)            # 2 Hz task allocation
        self.create_timer(1.0, self.publish_viz)
        self.get_logger().info('coverage coordinator up; waiting for /map')

    # ------------------------------------------------------------------ setup
    def on_map(self, msg: OccupancyGrid):
        if self.grid is not None:
            return
        occ = np.array(msg.data, np.int16).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        self.grid = build_cell_grid(
            occ, msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y),
            self.cell_size, self.clearance)
        self.zones = partition_zones(self.grid, self.zone_cols, self.zone_rows)
        for z in self.zones:
            z.goals = boustrophedon(self.grid, z.cells, self.lane_dir)
        self.covered_by = np.full(self.grid.shape, -1, np.int16)
        self.visited_sets = [[set() for _ in range(self.grid.shape[1])]
                             for _ in range(self.grid.shape[0])]
        n = int(self.grid.coverable.sum())
        self.get_logger().info(
            f'map {msg.info.width}x{msg.info.height}: {n} coverable cells '
            f'({n * self.cell_size ** 2:.1f} m^2) in {len(self.zones)} zones: '
            + ', '.join(f'Z{z.zid}={len(z.cells)}c/{len(z.goals)}g' for z in self.zones))
        self.mission = 'ready'
        self.publish_status()

    # --------------------------------------------------------------- services
    def srv_start(self, _req, res):
        if self.grid is None:
            res.success, res.message = False, 'no map yet'
        elif self.mission in ('running',):
            res.success, res.message = False, 'already running'
        else:
            self.start_mission()
            res.success, res.message = True, f'coverage started with {len(self.zones)} zones'
        return res

    def srv_stop(self, _req, res):
        for r in self.robots:
            self.cancel(r)
            self.release(r)
            if r.state not in (RState.OFFLINE, RState.WAITING):
                r.state = RState.IDLE
        self.mission = 'stopped'
        self.publish_status()
        res.success, res.message = True, 'coverage stopped; call start to resume'
        return res

    def start_mission(self):
        if self.mission != 'stopped':      # fresh start; 'stopped' resumes
            self.t_start = self.now()
        self.mission = 'running'
        self.sweep_done = False
        for r in self.robots:
            if r.state in (RState.DONE, RState.HOMING):
                r.state = RState.IDLE
        self.get_logger().info('=== coverage mission started ===')
        self.publish_status()

    # ---------------------------------------------------------------- helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def is_covered(self, cell) -> bool:
        return self.covered_by[cell] >= 0

    def goal_needed(self, g: Goal) -> bool:
        return any(not self.is_covered(c) for c in g.cells)

    def remaining_cells(self) -> int:
        return int((self.grid.coverable & (self.covered_by < 0)).sum())

    # ------------------------------------------------------- coverage tracking
    def track_coverage(self):
        if self.grid is None:
            return
        r_cells = int(math.ceil(self.cov_radius / self.cell_size))
        for rb in self.robots:
            p = self.tf.fresh(rb.name, max_age=1.0)
            if p is None:
                continue
            if rb.last_xy is not None and self.mission == 'running':
                rb.distance += math.hypot(p.x - rb.last_xy[0], p.y - rb.last_xy[1])
            rb.last_xy = (p.x, p.y)
            if self.mission != 'running':
                continue
            cell = self.grid.cell_of(p.x, p.y)
            if cell is None:
                continue
            for dr in range(-r_cells, r_cells + 1):
                for dc in range(-r_cells, r_cells + 1):
                    r, c = cell[0] + dr, cell[1] + dc
                    if not (0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]):
                        continue
                    if not self.grid.coverable[r, c]:
                        continue
                    cx, cy = self.grid.center(r, c)
                    if math.hypot(cx - p.x, cy - p.y) > self.cov_radius:
                        continue
                    self.visited_sets[r][c].add(rb.index)
                    if self.covered_by[r, c] < 0:
                        self.covered_by[r, c] = rb.index
                        rb.cells_covered += 1

    # -------------------------------------------------------------- scheduling
    def schedule(self):
        if self.grid is None:
            return
        if self.auto_start and self.mission == 'ready' and \
                all(r.state == RState.IDLE for r in self.robots):
            self.start_mission()
        self.update_robot_health()
        if self.mission != 'running':
            return

        for r in self.robots:
            if r.state == RState.BUSY:
                if self.now() - r.goal_sent_at > self.goal_timeout:
                    self.get_logger().warn(f'{r.name}: goal timed out -> cancel & retry later')
                    self.cancel(r)
                    self.goal_failed(r)
                elif r.goal is not None and not self.goal_needed(r.goal) and \
                        len(r.goal.cells) > 1:
                    # lane already covered (by another robot or on the way): skip it, but
                    # only send the next goal once Nav2 has confirmed the cancel - sending
                    # it immediately races the cancel and bt_navigator aborts the new goal
                    self.get_logger().info(f'{r.name}: lane already covered, skipping')
                    r.goal = None
                    self.cancel(r, then_idle=True)
            elif r.state == RState.CANCELING and self.now() - r.goal_sent_at > 3.0:
                r.state = RState.IDLE      # cancel never confirmed: carry on anyway

        # robots that need a new zone right now are assigned jointly (optimal matching)
        needy = [r for r in self.robots if r.state == RState.IDLE and
                 not any(self.goal_needed(g) for g in r.queue)]
        if len(needy) > 1:
            self.assign_zones_jointly(needy)
        for r in self.robots:
            if r.state == RState.IDLE:
                self.dispatch(r)

        active = [r for r in self.robots
                  if r.state in (RState.BUSY, RState.IDLE, RState.CANCELING)]
        pending = any(z.owner is None and not z.done and z.goals for z in self.zones)
        if not pending and all(not r.queue and r.state not in (RState.BUSY, RState.CANCELING)
                               for r in active):
            if self.sweep_up and not self.sweep_done:
                self.sweep_done = True
                if self.plan_sweep_up([r for r in active if r.state == RState.IDLE]):
                    return
            self.finish_mission()

    def plan_sweep_up(self, robots) -> bool:
        """Send each robot to the uncovered cells nearest to it (one-off clean-up pass)."""
        robots = [r for r in robots if self.tf.last.get(r.name) is not None]
        cells = [tuple(int(v) for v in c)
                 for c in zip(*np.nonzero(self.grid.coverable & (self.covered_by < 0)))]
        if not cells or not robots:
            return False
        groups = {r.name: [] for r in robots}
        for c in cells:
            cx, cy = self.grid.center(*c)
            best = min(robots, key=lambda r: math.hypot(self.tf.last[r.name].x - cx,
                                                        self.tf.last[r.name].y - cy))
            groups[best.name].append(c)
        for r in robots:
            if not groups[r.name]:
                continue
            pts = [self.grid.center(*c) for c in groups[r.name]]
            p = self.tf.last[r.name]
            order = nearest_neighbour_order(pts, (p.x, p.y))
            goals = [Goal(pts[k][0], pts[k][1], cells=[groups[r.name][k]]) for k in order]
            orient_goals(goals)
            z = Zone(len(self.zones), groups[r.name], (0, 0, 0, 0), goals=[], owner=r.name)
            self.zones.append(z)
            r.zone, r.queue = z, goals
        self.get_logger().info(
            f'sweep-up pass: {len(cells)} uncovered cells -> '
            + ', '.join(f'{n}:{len(g)}' for n, g in groups.items() if g))
        return True

    def update_robot_health(self):
        for r in self.robots:
            pose_ok = self.tf.age(self.tf.lookup(r.name)) < self.offline_after
            nav_ok = r.client.server_is_ready()
            if r.state == RState.WAITING:
                if pose_ok and nav_ok:
                    r.state = RState.IDLE
                    self.get_logger().info(f'{r.name}: Nav2 + pose ready')
            elif r.state == RState.OFFLINE:
                if pose_ok and nav_ok:
                    r.state = RState.IDLE
                    self.get_logger().info(f'{r.name}: back ONLINE, rejoining')
            elif not (pose_ok and nav_ok):
                self.get_logger().error(
                    f'{r.name}: OFFLINE (pose_ok={pose_ok}, nav_ok={nav_ok}); '
                    f'releasing {len(r.queue) + (1 if r.goal else 0)} goals')
                self.release(r)
                r.state = RState.OFFLINE

    def release(self, r: Robot):
        """Give a robot's unfinished work back to the pool as a new zone."""
        goals = ([r.goal] if r.goal else []) + r.queue
        r.goal_seq += 1
        r.goal, r.goal_handle, r.queue = None, None, []
        if r.zone is not None:
            r.zone.owner = None
            r.zone.goals = [g for g in goals if self.goal_needed(g)]
            r.zone.done = not r.zone.goals
            r.zone = None

    def dispatch(self, r: Robot):
        # drop goals that are already covered
        while r.queue and not self.goal_needed(r.queue[0]):
            r.queue.pop(0)
        if not r.queue:
            self.close_zone(r)
            if not self.assign_zone(r) and not self.steal_work(r):
                return
            self.publish_status()
            while r.queue and not self.goal_needed(r.queue[0]):
                r.queue.pop(0)
            if not r.queue:
                return
        self.send_goal(r, r.queue.pop(0))

    def close_zone(self, r: Robot):
        r.queue = []
        if r.zone is not None:
            r.zone.done = True
            r.zones_done.append(r.zone.zid)
            self.get_logger().info(f'{r.name}: finished zone Z{r.zone.zid}')
            r.zone = None

    def assign_zones_jointly(self, robots):
        """Hungarian assignment of idle robots to free zones (min total travel to the
        nearest end of each zone) - avoids the greedy 'first robot grabs the best' bias."""
        from scipy.optimize import linear_sum_assignment
        free = []
        for z in self.zones:
            if z.owner is None and not z.done:
                z.goals = [g for g in z.goals if self.goal_needed(g)]
                if z.goals:
                    free.append(z)
                else:
                    z.done = True
        robots = [r for r in robots if self.tf.last.get(r.name) is not None]
        if not free or not robots:
            return
        cost = np.zeros((len(robots), len(free)))
        for i, r in enumerate(robots):
            p = self.tf.last[r.name]
            for j, z in enumerate(free):
                cost[i, j] = min(math.hypot(z.goals[0].x - p.x, z.goals[0].y - p.y),
                                 math.hypot(z.goals[-1].x - p.x, z.goals[-1].y - p.y))
        rows, cols = linear_sum_assignment(cost)
        for i, j in zip(rows, cols):
            self.close_zone(robots[i])
            self.take_zone(robots[i], free[j])

    def take_zone(self, r: Robot, z: Zone):
        p = self.tf.last[r.name]
        rev = math.hypot(z.goals[-1].x - p.x, z.goals[-1].y - p.y) < \
            math.hypot(z.goals[0].x - p.x, z.goals[0].y - p.y)
        goals = list(reversed(z.goals)) if rev else list(z.goals)
        orient_goals(goals)
        z.owner, r.zone, r.queue = r.name, z, goals
        self.get_logger().info(
            f'{r.name}: assigned zone Z{z.zid} ({len(goals)} goals, '
            f'{path_length(goals, (p.x, p.y)):.1f} m{" reversed" if rev else ""})')

    def assign_zone(self, r: Robot) -> bool:
        p = self.tf.last.get(r.name)
        if p is None:
            return False
        best, best_d = None, math.inf
        for z in self.zones:
            if z.owner is not None or z.done:
                continue
            z.goals = [g for g in z.goals if self.goal_needed(g)]
            if not z.goals:
                z.done = True
                continue
            d = min(math.hypot(z.goals[0].x - p.x, z.goals[0].y - p.y),
                    math.hypot(z.goals[-1].x - p.x, z.goals[-1].y - p.y))
            if d < best_d:
                best, best_d = z, d
        if best is None:
            return False
        self.take_zone(r, best)
        return True

    def steal_work(self, r: Robot) -> bool:
        """Split the busiest robot's remaining goals: the idle robot takes the tail."""
        donor = max((o for o in self.robots if o is not r and o.state == RState.BUSY),
                    key=lambda o: len(o.queue), default=None)
        if donor is None or len(donor.queue) < self.min_steal:
            return False
        p = self.tf.last.get(r.name)
        if p is None:
            return False
        cut = len(donor.queue) // 2
        tail = donor.queue[cut:]
        donor.queue = donor.queue[:cut]
        # the idle robot starts from whichever end of the stolen piece is nearer
        if math.hypot(tail[-1].x - p.x, tail[-1].y - p.y) < \
                math.hypot(tail[0].x - p.x, tail[0].y - p.y):
            tail.reverse()
        orient_goals(tail)
        z = Zone(len(self.zones), [c for g in tail for c in g.cells], donor.zone.bounds
                 if donor.zone else (0, 0, 0, 0), goals=[], owner=r.name)
        self.zones.append(z)
        r.zone, r.queue = z, tail
        self.get_logger().info(
            f'{r.name}: took {len(tail)} goals from {donor.name} (work stealing, new Z{z.zid})')
        return True

    # ---------------------------------------------------------------- actions
    def send_goal(self, r: Robot, g: Goal):
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = g.x, g.y
        msg.pose.pose.orientation = quat_from_yaw(g.yaw)
        r.goal, r.state = g, RState.BUSY
        r.goal_seq += 1
        seq = r.goal_seq
        r.goal_sent_at = self.now()
        fut = r.client.send_goal_async(msg)
        fut.add_done_callback(lambda f, rb=r, s=seq: self.on_goal_response(rb, s, f))

    def on_goal_response(self, r: Robot, seq: int, fut):
        if seq != r.goal_seq:
            return
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'{r.name}: goal rejected')
            self.goal_failed(r)
            return
        r.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, rb=r, s=seq: self.on_result(rb, s, f))

    def on_result(self, r: Robot, seq: int, fut):
        if seq != r.goal_seq or r.state != RState.BUSY:
            return
        status = fut.result().status
        r.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            r.goals_done += 1
            r.goal = None
            r.state = RState.IDLE
        else:
            self.get_logger().warn(f'{r.name}: goal ended with status {status}')
            self.goal_failed(r)

    def goal_failed(self, r: Robot):
        g = r.goal
        r.goal, r.goal_handle = None, None
        r.goal_seq += 1
        if r.state == RState.BUSY:
            r.state = RState.IDLE
        if g is None:
            return
        g.attempts += 1
        if g.attempts < self.max_attempts:
            r.queue.append(g)            # retry after the rest (blockage may clear)
        else:
            r.goals_failed += 1
            self.get_logger().error(
                f'{r.name}: dropping goal ({g.x:.2f},{g.y:.2f}) after {g.attempts} attempts')

    def cancel(self, r: Robot, then_idle: bool = False):
        handle, r.goal_handle = r.goal_handle, None
        r.goal_seq += 1
        if not then_idle:
            if handle is not None:
                handle.cancel_goal_async()
            return
        if handle is None:
            r.state = RState.IDLE
            return
        r.state = RState.CANCELING
        r.goal_sent_at = self.now()
        seq = r.goal_seq

        def done(_fut, rb=r, s=seq):
            if rb.state == RState.CANCELING and rb.goal_seq == s:
                rb.state = RState.IDLE
        handle.cancel_goal_async().add_done_callback(done)

    # ----------------------------------------------------------------- finish
    def finish_mission(self):
        if self.mission != 'running':
            return
        self.mission = 'complete'
        self.t_end = self.now()
        self.get_logger().info('=== coverage COMPLETE ===  ' + self.summary())
        self.write_report()
        for r in self.robots:
            r.zone = None
            if r.state == RState.IDLE and self.return_home:
                r.state = RState.HOMING
                self.send_home(r)
            elif r.state == RState.IDLE:
                r.state = RState.DONE
        self.publish_status()

    def send_home(self, r: Robot):
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = r.spec.x, r.spec.y
        msg.pose.pose.orientation = quat_from_yaw(r.spec.yaw)
        r.goal_seq += 1
        seq = r.goal_seq

        def done(f, rb=r, s=seq):
            h = f.result()
            if s != rb.goal_seq or h is None or not h.accepted:
                return
            rb.goal_handle = h
            h.get_result_async().add_done_callback(
                lambda _f, rb2=rb, s2=s: self._homed(rb2, s2))
        r.client.send_goal_async(msg).add_done_callback(done)

    def _homed(self, r, seq):
        if seq == r.goal_seq and r.state == RState.HOMING:
            r.state = RState.DONE
            r.goal_handle = None
            self.get_logger().info(f'{r.name}: back home')
            self.publish_status()

    def stats(self):
        total = int(self.grid.coverable.sum())
        covered = int((self.grid.coverable & (self.covered_by >= 0)).sum())
        multi = sum(1 for row in self.visited_sets for s in row if len(s) > 1)
        t_end = self.t_end if self.t_end is not None else self.now()
        return {
            'coverable_cells': total, 'covered_cells': covered,
            'coverage_pct': round(100.0 * covered / max(total, 1), 1),
            'cells_driven_over_by_2plus_robots': multi,
            'overlap_pct': round(100.0 * multi / max(covered, 1), 1),
            'cell_size_m': self.cell_size,
            'elapsed_s': round(t_end - self.t_start, 1) if self.t_start else 0.0,
            'robots': {r.name: {'state': r.state.value, 'cells_covered': r.cells_covered,
                                'distance_m': round(r.distance, 1),
                                'goals_done': r.goals_done, 'goals_failed': r.goals_failed,
                                'zones': r.zones_done,
                                'queue': len(r.queue)} for r in self.robots},
        }

    def summary(self):
        s = self.stats()
        return (f"coverage {s['coverage_pct']}% ({s['covered_cells']}/{s['coverable_cells']} "
                f"cells), overlap {s['overlap_pct']}%, t={s['elapsed_s']}s")

    def write_report(self):
        try:
            os.makedirs(os.path.dirname(self.report_path), exist_ok=True)
            s = self.stats()
            s['zones'] = [{'id': z.zid, 'cells': len(z.cells), 'bounds': z.bounds}
                          for z in self.zones]
            s['uncovered_cells_xy'] = [
                [round(v, 2) for v in self.grid.center(int(r), int(c))]
                for r, c in zip(*np.nonzero(self.grid.coverable & (self.covered_by < 0)))]
            s['written_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
            with open(self.report_path, 'w') as f:
                json.dump(s, f, indent=2)
            self.get_logger().info(f'report written to {self.report_path}')
        except OSError as e:
            self.get_logger().error(f'could not write report: {e}')

    # ------------------------------------------------------------ visualisation
    def publish_status(self):
        if self.grid is None:
            txt = f'mission={self.mission}'
        else:
            robots = ' '.join(
                f'{r.name}:{r.state.value}'
                + (f'[Z{r.zone.zid},{len(r.queue)}g]' if r.zone is not None else '')
                for r in self.robots)
            txt = f'mission={self.mission} | {self.summary()} | {robots}'
        self.pub_status.publish(String(data=txt))

    def publish_viz(self):
        if self.grid is None:
            return
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = self.cell_size
        g.info.width, g.info.height = self.grid.shape[1], self.grid.shape[0]
        g.info.origin.position.x, g.info.origin.position.y = \
            self.grid.origin_x, self.grid.origin_y
        g.info.origin.orientation.w = 1.0
        data = np.where(self.grid.coverable, 0, -1).astype(np.int16)
        cov = self.covered_by >= 0
        data[cov] = 10 * (self.covered_by[cov] + 1)
        g.data = data.flatten().astype(np.int8).tolist()
        self.pub_grid.publish(g)
        self.publish_zones()
        self.publish_status()

    def publish_zones(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        owners = {r.name: r.index for r in self.robots}
        for z in self.zones[:self.zone_cols * self.zone_rows]:
            owner_idx = owners.get(z.owner)
            col = COLORS[owner_idx % len(COLORS)] if owner_idx is not None else (0.4, 0.4, 0.4)
            x0, y0, x1, y1 = z.bounds
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id, m.type = 'zone_outline', z.zid, Marker.LINE_STRIP
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = (*col, 1.0)
            m.pose.orientation.w = 1.0
            m.points = [Point(x=x, y=y, z=0.02) for x, y in
                        ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0))]
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type = 'zone_label', z.zid, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = \
                (x0 + x1) / 2, (y0 + y1) / 2, 0.3
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r, t.color.g, t.color.b, t.color.a = (*col, 1.0)
            state = 'done' if z.done else (z.owner or 'free')
            t.text = f'Z{z.zid} {state}'
            arr.markers.append(t)
        # planned lanes of each robot's remaining queue
        for r in self.robots:
            pts = ([r.goal] if r.goal else []) + r.queue
            if len(pts) < 2:
                continue
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id, m.type = 'lanes', r.index, Marker.LINE_STRIP
            m.scale.x = 0.03
            m.pose.orientation.w = 1.0
            c = COLORS[r.index % len(COLORS)]
            m.color.r, m.color.g, m.color.b, m.color.a = (*c, 0.8)
            m.points = [Point(x=g.x, y=g.y, z=0.03) for g in pts]
            arr.markers.append(m)
        self.pub_zones.publish(arr)

    def on_shutdown(self):
        for r in self.robots:
            self.cancel(r)


def main():
    spin_node(CoverageCoordinator)
