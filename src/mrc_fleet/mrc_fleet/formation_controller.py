"""Leader-follower formation control.

Leader  : drives a route with its own Nav2 (NavigateThroughPoses); its speed is
          throttled through /<leader>/speed_limit so followers can keep up
          (and slowed further when a follower falls behind).
Followers: each tracks a slot rigidly attached to the leader,
             slot = T(map->leader) * offset          (offset in leader frame)
          from TF2 poses (map -> <robot>/base_footprint). The control law is
          look-ahead-point feedback linearisation with the leader's rigid-body
          velocity as feed-forward; precise parking in ALIGN/SETTLE. Commands go to
          /<follower>/cmd_vel_nav -> velocity_smoother -> collision_monitor -> robot,
          i.e. the same safety chain Nav2 uses.

States: IDLE -> ASSEMBLE (all robots Nav2 to formation start) -> ALIGN (park precisely)
        -> MARCH -> SETTLE -> DONE
Services: /fleet/start_formation, /fleet/stop_formation (std_srvs/Trigger)
"""
import math
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mrc_fleet.fleet_tf import FleetTF, quat_from_yaw, spin_node, wrap


class FState(Enum):
    IDLE = 'idle'
    ASSEMBLE = 'assemble'
    ALIGN = 'align'          # precise parking into the slots before the leader moves
    MARCH = 'march'
    SETTLE = 'settle'
    DONE = 'done'
    FAULT = 'fault'


def compose(x, y, yaw, dx, dy):
    """Point (dx, dy) given in the frame (x, y, yaw), expressed in the parent frame."""
    return x + math.cos(yaw) * dx - math.sin(yaw) * dy, y + math.sin(yaw) * dx + math.cos(yaw) * dy


def lookahead_cmd(lead, vl, wl, dx, dy, me, l, k_p, v_max, w_max):
    """Look-ahead-point (feedback linearisation) tracking of the slot.

    The point p = position + l*heading of a unicycle is fully actuated:
        p_dot = [[cos th, -l sin th], [sin th, l cos th]] [v, w]^T
    so with u = p_d_dot + K (p_d - p) and [v, w] = M^-1 u the point converges
    exponentially to its target p_d (the same look-ahead point of the slot pose,
    rigidly attached to the leader). p_d_dot is the rigid-body velocity of that
    point: v_L * heading_L + w_L x r. No mode switching, no singularity (l > 0),
    and it still converges when the leader is slow or stopped.
    """
    c, s = math.cos(lead[2]), math.sin(lead[2])
    # Direction the slot is actually moving, in the leader frame (rigid body:
    # v_slot = (v_L - w_L*dy, w_L*dx)). In a turn each slot moves along its own circle,
    # so aiming the look-ahead point along the LEADER heading would leave an error of
    # l*|heading difference|; aiming along the slot's own velocity lets the axle sit
    # exactly on the slot. Blend toward the leader heading when the slot is ~static.
    ux, uy = max(vl - wl * dy, 0.0) + 0.02, wl * dx
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n
    # target point in leader frame: slot offset + l along the slot's direction of motion
    rx, ry = dx + l * ux, dy + l * uy
    pdx, pdy = lead[0] + c * rx - s * ry, lead[1] + s * rx + c * ry
    rwx, rwy = c * rx - s * ry, s * rx + c * ry         # r in world frame
    ffx, ffy = vl * c - wl * rwy, vl * s + wl * rwx         # rigid-body velocity
    px, py = me[0] + l * math.cos(me[2]), me[1] + l * math.sin(me[2])
    ux = ffx + k_p * (pdx - px)
    uy = ffy + k_p * (pdy - py)
    v = math.cos(me[2]) * ux + math.sin(me[2]) * uy
    w = (-math.sin(me[2]) * ux + math.cos(me[2]) * uy) / l
    # scale both together when saturating so the path curvature is preserved
    scale = max(1.0, abs(v) / v_max, abs(w) / w_max)
    return v / scale, w / scale


class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        p = self.declare_parameter
        p('leader', 'robot1')
        p('followers', ['robot2', 'robot3'])
        p('offsets', [-0.8, 0.7, -0.8, -0.7])     # dx,dy per follower in leader frame
        p('route', [-3.0, -0.9, 2.5, -0.9])       # leader waypoints x,y,...  (map)
        p('leader_speed_pct', 50.0)
        p('control_rate', 20.0)
        p('lookahead', 0.20)            # look-ahead point distance l [m]
        p('k_p', 0.8)                   # look-ahead point gain [1/s]
        p('v_max', 0.26)
        p('w_max', 1.5)
        p('slow_error', 0.6)            # leader slows down as follower error approaches this
        p('min_separation', 0.50)       # follower stops moving toward a robot closer than this
        p('settle_tolerance', 0.12)
        p('settle_timeout', 20.0)
        p('assemble_timeout', 120.0)
        p('pose_timeout', 1.0)
        p('auto_start', False)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.leader = gp('leader')
        self.followers = list(gp('followers'))
        off = list(gp('offsets'))
        if len(off) != 2 * len(self.followers):
            raise ValueError('offsets must hold one (dx, dy) pair per follower')
        self.offsets = {f: (off[2 * i], off[2 * i + 1]) for i, f in enumerate(self.followers)}
        rt = list(gp('route'))
        if len(rt) < 4 or len(rt) % 2:
            raise ValueError('route needs at least two x,y waypoints')
        self.route = [(rt[i], rt[i + 1]) for i in range(0, len(rt), 2)]
        self.leader_pct = gp('leader_speed_pct')
        self.lookahead, self.k_p = gp('lookahead'), gp('k_p')
        self.v_max, self.w_max = gp('v_max'), gp('w_max')
        self.slow_error = gp('slow_error')
        self.min_sep = gp('min_separation')
        self.settle_tol = gp('settle_tolerance')
        self.settle_timeout = gp('settle_timeout')
        self.assemble_timeout = gp('assemble_timeout')
        self.pose_timeout = gp('pose_timeout')

        everyone = [self.leader] + self.followers
        self.tf = FleetTF(self, everyone)
        self.nav = {n: ActionClient(self, NavigateToPose, f'/{n}/navigate_to_pose')
                    for n in everyone}
        self.nav_through = ActionClient(self, NavigateThroughPoses,
                                        f'/{self.leader}/navigate_through_poses')
        self.cmd_pub = {f: self.create_publisher(Twist, f'/{f}/cmd_vel_nav', 10)
                        for f in self.followers}
        self.speed_pub = self.create_publisher(SpeedLimit, f'/{self.leader}/speed_limit', 1)
        self.err_pub = self.create_publisher(Float32MultiArray, '/fleet/formation_error', 10)
        self.state_pub = self.create_publisher(String, '/fleet/formation_state', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/formation_markers', 1)
        self.leader_twist = (0.0, 0.0)
        self.leader_twist_t = None
        self.create_subscription(Odometry, f'/{self.leader}/odom', self.on_leader_odom, 10)
        self.create_service(Trigger, '/fleet/start_formation', self.srv_start)
        self.create_service(Trigger, '/fleet/stop_formation', self.srv_stop)

        self.state = FState.IDLE
        self.state_since = self.now()
        self.handles = {}
        self.pending = {}        # robot -> 'active' | 'ok' | 'fail'
        self.seq = 0
        self.max_err_seen = 0.0
        self.err_sum, self.err_n = 0.0, 0
        self.last_speed_pct = None
        self.heading_err = {}
        self.auto_start = gp('auto_start')
        self.create_timer(1.0 / gp('control_rate'), self.control)
        self.create_timer(0.5, self.supervise)
        self.get_logger().info(
            f'formation: leader={self.leader} followers={self.offsets} route={self.route}')

    # ----------------------------------------------------------------- utils
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s: FState, why=''):
        if s != self.state:
            self.get_logger().info(f'formation {self.state.value} -> {s.value} {why}')
        self.state, self.state_since = s, self.now()
        self.state_pub.publish(String(data=s.value))

    def on_leader_odom(self, msg: Odometry):
        self.leader_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.leader_twist_t = self.now()

    def stop_followers(self):
        for pub in self.cmd_pub.values():
            pub.publish(Twist())

    def set_leader_speed(self, pct):
        pct = max(10.0, min(100.0, pct))   # 0 would mean "no limit" in Nav2!
        if self.last_speed_pct is None or abs(pct - self.last_speed_pct) > 2.0:
            self.speed_pub.publish(SpeedLimit(percentage=True, speed_limit=float(pct)))
            self.last_speed_pct = pct

    def pose_goal(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation = quat_from_yaw(yaw)
        return ps

    def send(self, robot, client, goal_msg):
        self.pending[robot] = 'active'
        seq = self.seq

        def on_resp(fut):
            h = fut.result()
            if seq != self.seq:
                return
            if h is None or not h.accepted:
                self.pending[robot] = 'fail'
                return
            self.handles[robot] = h
            h.get_result_async().add_done_callback(on_result)

        def on_result(fut):
            if seq != self.seq:
                return
            ok = fut.result().status == GoalStatus.STATUS_SUCCEEDED
            self.pending[robot] = 'ok' if ok else 'fail'
            self.handles.pop(robot, None)
        client.send_goal_async(goal_msg).add_done_callback(on_resp)

    def cancel_all(self):
        self.seq += 1
        for h in self.handles.values():
            h.cancel_goal_async()
        self.handles.clear()
        self.pending.clear()

    # -------------------------------------------------------------- services
    def srv_start(self, _req, res):
        missing = [n for n, c in self.nav.items() if not c.server_is_ready()]
        if missing:
            res.success, res.message = False, f'Nav2 not ready for {missing}'
            return res
        self.cancel_all()
        self.max_err_seen, self.err_sum, self.err_n = 0.0, 0.0, 0
        self.start_assembly()
        res.success, res.message = True, 'formation: assembling'
        return res

    def srv_stop(self, _req, res):
        self.cancel_all()
        self.stop_followers()
        self.set_leader_speed(100.0)
        self.set_state(FState.IDLE, '(stop requested)')
        res.success, res.message = True, 'formation stopped'
        return res

    # ------------------------------------------------------------ supervisor
    def start_assembly(self):
        (x0, y0), (x1, y1) = self.route[0], self.route[1]
        yaw0 = math.atan2(y1 - y0, x1 - x0)
        self.set_leader_speed(100.0)
        g = NavigateToPose.Goal()
        g.pose = self.pose_goal(x0, y0, yaw0)
        self.send(self.leader, self.nav[self.leader], g)
        for f, (dx, dy) in self.offsets.items():
            sx, sy = compose(x0, y0, yaw0, dx, dy)
            g = NavigateToPose.Goal()
            g.pose = self.pose_goal(sx, sy, yaw0)
            self.send(f, self.nav[f], g)
        self.set_state(FState.ASSEMBLE, f'(start pose {x0:.1f},{y0:.1f})')

    def start_march(self):
        self.seq += 1
        self.pending.clear()
        self.set_leader_speed(self.leader_pct)
        g = NavigateThroughPoses.Goal()
        pts = self.route[1:]
        prev = self.route[0]
        for (x, y) in pts:
            g.poses.append(self.pose_goal(x, y, math.atan2(y - prev[1], x - prev[0])))
            prev = (x, y)
        self.send(self.leader, self.nav_through, g)
        self.set_state(FState.MARCH, f'({len(pts)} waypoints @ {self.leader_pct:.0f}% speed)')

    def supervise(self):
        if self.auto_start and self.state == FState.IDLE and \
                all(c.server_is_ready() for c in self.nav.values()) and \
                all(self.tf.fresh(n) for n in self.nav):
            self.auto_start = False
            self.start_assembly()
        elapsed = self.now() - self.state_since
        if self.state == FState.ASSEMBLE:
            if any(v == 'fail' for v in self.pending.values()):
                failed = [k for k, v in self.pending.items() if v == 'fail']
                self.get_logger().warn(f'assembly goal failed for {failed}; retrying')
                self.cancel_all()
                self.start_assembly()
            elif self.pending and all(v == 'ok' for v in self.pending.values()):
                self.seq += 1
                self.pending.clear()
                self.set_state(FState.ALIGN, '(parking followers into their slots)')
            elif elapsed > self.assemble_timeout:
                self.cancel_all()
                self.set_state(FState.FAULT, '(assembly timed out)')
        elif self.state == FState.MARCH:
            st = self.pending.get(self.leader)
            if st == 'ok':
                self.set_state(FState.SETTLE, '(leader reached end of route)')
            elif st == 'fail':
                self.get_logger().warn('leader route failed; re-sending remaining route')
                self.resend_remaining_route()
        elif self.state == FState.SETTLE and elapsed > self.settle_timeout:
            self.finish('(settle timeout)')
        elif self.state == FState.ALIGN and elapsed > self.settle_timeout:
            self.get_logger().warn('align timeout: starting march anyway')
            self.start_march()

    def resend_remaining_route(self):
        p = self.tf.last.get(self.leader)
        if p is None:
            self.set_state(FState.FAULT, '(leader lost)')
            return
        # continue from the first waypoint that is still ahead of the leader
        best = min(range(1, len(self.route)),
                   key=lambda i: math.hypot(self.route[i][0] - p.x, self.route[i][1] - p.y))
        self.route = [(p.x, p.y)] + self.route[best:]
        self.start_march()

    def finish(self, why=''):
        self.stop_followers()
        self.set_leader_speed(100.0)
        mean = self.err_sum / max(self.err_n, 1)
        self.get_logger().info(
            f'=== formation DONE {why}: mean slot error {mean:.3f} m, '
            f'max {self.max_err_seen:.3f} m ===')
        self.set_state(FState.DONE)

    # --------------------------------------------------------------- control
    def control(self):
        if self.state not in (FState.ALIGN, FState.MARCH, FState.SETTLE):
            return
        lead = self.tf.fresh(self.leader, self.pose_timeout)
        if lead is None:
            self.stop_followers()
            self.get_logger().warn('leader pose stale: holding followers',
                                   throttle_duration_sec=2.0)
            return
        vl, wl = self.leader_twist
        if self.leader_twist_t is None or self.now() - self.leader_twist_t > 0.5:
            vl, wl = 0.0, 0.0
        poses = {n: self.tf.last.get(n) for n in [self.leader] + self.followers}
        errors, markers = [], MarkerArray()
        for k, (f, (dx, dy)) in enumerate(self.offsets.items()):
            me = self.tf.fresh(f, self.pose_timeout)
            if me is None:
                self.cmd_pub[f].publish(Twist())
                errors.append(float('nan'))
                continue
            xd, yd = compose(lead.x, lead.y, lead.yaw, dx, dy)
            thd = lead.yaw
            eth = wrap(thd - me.yaw)
            self.heading_err[f] = eth
            ex_w, ey_w = xd - me.x, yd - me.y
            dist = math.hypot(ex_w, ey_w)
            errors.append(dist)
            if self.state in (FState.ALIGN, FState.SETTLE):
                ex = math.cos(me.yaw) * ex_w + math.sin(me.yaw) * ey_w
                ey = -math.sin(me.yaw) * ex_w + math.cos(me.yaw) * ey_w
                v, w = self.park(ex, ey, eth, dist)
            else:
                v, w = self.lookahead_control(lead, vl, wl, dx, dy, me)
            v = self.separation_guard(f, me, v, poses)
            cmd = Twist()
            cmd.linear.x = max(-0.10, min(self.v_max, v))
            cmd.angular.z = max(-self.w_max, min(self.w_max, w))
            self.cmd_pub[f].publish(cmd)
            markers.markers += self.slot_markers(k, xd, yd, thd, me)

        valid = [e for e in errors if not math.isnan(e)]
        worst = max(valid) if valid else 0.0
        if self.state == FState.MARCH and valid:
            self.max_err_seen = max(self.max_err_seen, worst)
            self.err_sum += sum(valid) / len(valid)
            self.err_n += 1
            # leader waits for the slowest follower
            self.set_leader_speed(self.leader_pct * max(0.2, 1.0 - worst / self.slow_error))
        parked = valid and worst < self.settle_tol and \
            all(abs(h) < 0.1 for h in self.heading_err.values())
        if self.state == FState.SETTLE and parked:
            self.finish('(all followers in slot)')
        elif self.state == FState.ALIGN and parked:
            self.stop_followers()
            self.start_march()
        self.err_pub.publish(Float32MultiArray(data=[float(e) for e in errors]))
        self.marker_pub.publish(markers)

    def lookahead_control(self, lead, vl, wl, dx, dy, me):
        return lookahead_cmd((lead.x, lead.y, lead.yaw), vl, wl, dx, dy, (me.x, me.y, me.yaw),
                             self.lookahead, self.k_p, self.v_max, self.w_max)

    def park(self, ex, ey, eth, dist):
        """Final parking into the slot with the leader stopped.

        With the leader at rest the tracking law has no feed-forward, so a lateral error
        cannot be removed; instead drive straight at the slot (backwards if it is behind
        us), then turn to the formation heading.
        """
        if dist > 0.6 * self.settle_tol:
            bearing = math.atan2(ey, ex)
            reverse = abs(bearing) > math.pi / 2
            if reverse:
                bearing = wrap(bearing + math.pi)
            w = 2.0 * bearing
            v = 0.0 if abs(bearing) > 0.35 else min(0.12, 0.8 * dist)
            return (-v if reverse else v), w
        if abs(eth) > 0.06:
            return 0.0, max(-0.6, min(0.6, 1.5 * eth))
        return 0.0, 0.0

    def separation_guard(self, me_name, me, v, poses):
        """Do not drive toward a robot that is closer than min_separation."""
        for n, p in poses.items():
            if n == me_name or p is None:
                continue
            d = math.hypot(p.x - me.x, p.y - me.y)
            if d < self.min_sep:
                ahead = math.cos(me.yaw) * (p.x - me.x) + math.sin(me.yaw) * (p.y - me.y)
                if (ahead > 0 and v > 0) or (ahead < 0 and v < 0):
                    self.get_logger().warn(f'{me_name}: too close to {n} ({d:.2f} m), holding',
                                           throttle_duration_sec=2.0)
                    return 0.0
        return v

    def slot_markers(self, k, xd, yd, thd, me):
        stamp = self.get_clock().now().to_msg()
        slot = Marker()
        slot.header.frame_id, slot.header.stamp = 'map', stamp
        slot.ns, slot.id, slot.type = 'slots', k, Marker.ARROW
        slot.pose.position.x, slot.pose.position.y = xd, yd
        slot.pose.orientation = quat_from_yaw(thd)
        slot.scale.x, slot.scale.y, slot.scale.z = 0.3, 0.06, 0.06
        slot.color.r, slot.color.g, slot.color.b, slot.color.a = 1.0, 0.8, 0.0, 0.9
        link = Marker()
        link.header = slot.header
        link.ns, link.id, link.type = 'links', k, Marker.LINE_LIST
        link.pose.orientation.w = 1.0
        link.scale.x = 0.02
        link.color.r, link.color.g, link.color.b, link.color.a = 1.0, 0.8, 0.0, 0.7
        link.points = [Point(x=me.x, y=me.y, z=0.05), Point(x=xd, y=yd, z=0.05)]
        return [slot, link]

    def on_shutdown(self):
        self.cancel_all()
        self.stop_followers()


def main():
    spin_node(FormationController)
