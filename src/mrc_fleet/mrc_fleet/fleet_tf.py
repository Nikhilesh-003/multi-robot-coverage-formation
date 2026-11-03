"""Fleet pose lookup over the shared TF tree (map -> <robot>/base_footprint)."""
import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    stamp: Time

    def dist(self, other) -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class FleetTF:
    """Looks up every robot's pose in the global frame via TF2."""

    def __init__(self, node, robot_names, global_frame='map', base_frame='base_footprint'):
        self.node = node
        self.names = list(robot_names)
        self.global_frame = global_frame
        self.base_frame = base_frame
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        self.last: Dict[str, Pose2D] = {}

    def lookup(self, name: str) -> Optional[Pose2D]:
        try:
            t = self.buffer.lookup_transform(
                self.global_frame, f'{name}/{self.base_frame}', Time())
        except TransformException:
            return None
        p = Pose2D(t.transform.translation.x, t.transform.translation.y,
                   yaw_from_quat(t.transform.rotation), Time.from_msg(t.header.stamp))
        self.last[name] = p
        return p

    def age(self, pose: Optional[Pose2D]) -> float:
        if pose is None:
            return math.inf
        return (self.node.get_clock().now() - pose.stamp).nanoseconds * 1e-9

    def fresh(self, name: str, max_age: float = 1.0) -> Optional[Pose2D]:
        p = self.lookup(name)
        return p if self.age(p) <= max_age else None


class FleetPoses:
    """Same interface as FleetTF, fed by fleet_monitor's /fleet/poses + /fleet/health
    instead of a private TF listener (cheaper: rclpy TF ingestion of the whole fleet's
    /tf costs a noticeable share of a CPU core per listening node)."""

    def __init__(self, node, robot_names):
        import json
        from geometry_msgs.msg import PoseArray
        from std_msgs.msg import String
        self.node = node
        self.names = list(robot_names)
        self.last: Dict[str, Pose2D] = {}
        self._ages: Dict[str, Optional[float]] = {}
        node.create_subscription(
            String, '/fleet/health', lambda m: self._ages.update(json.loads(m.data)), 10)
        node.create_subscription(PoseArray, '/fleet/poses', self._on_poses, 10)

    def _on_poses(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        for name, p in zip(self.names, msg.poses):
            age = self._ages.get(name)
            if age is None:
                continue          # never localized: no pose to report
            self.last[name] = Pose2D(p.position.x, p.position.y, yaw_from_quat(p.orientation),
                                     stamp - Duration(seconds=age))

    def lookup(self, name: str) -> Optional[Pose2D]:
        return self.last.get(name)

    def age(self, pose: Optional[Pose2D]) -> float:
        if pose is None:
            return math.inf
        return (self.node.get_clock().now() - pose.stamp).nanoseconds * 1e-9

    def fresh(self, name: str, max_age: float = 1.0) -> Optional[Pose2D]:
        p = self.lookup(name)
        return p if self.age(p) <= max_age else None


def spin_node(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if hasattr(node, 'on_shutdown'):
            node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
