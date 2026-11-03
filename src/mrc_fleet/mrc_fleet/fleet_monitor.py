"""Fleet monitor: shares every robot's pose with the others.

Publishes (10 Hz):
  /fleet/poses          PoseArray (map), one entry per robot in fleet.yaml order
                        (last known pose if a robot is temporarily lost)
  /fleet/health         String, JSON {robot: pose_age_s}
  /<robot>/peer_poses   PoseArray (map) of the OTHER robots with fresh poses
                        -> consumed by the PeerLayer in that robot's costmaps
  /fleet/robot_markers  MarkerArray (name labels + footprint discs for RViz)
"""
import json

from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mrc_fleet.fleet_config import load_fleet
from mrc_fleet.fleet_tf import FleetTF, quat_from_yaw, spin_node

COLORS = [(0.90, 0.10, 0.29), (0.24, 0.71, 0.29), (0.26, 0.39, 0.85),
          (0.96, 0.51, 0.19), (0.57, 0.12, 0.71), (0.26, 0.83, 0.96)]


class FleetMonitor(Node):
    def __init__(self):
        super().__init__('fleet_monitor')
        self.declare_parameter('fleet_config', '')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_pose_age', 1.0)
        self.declare_parameter('robot_radius', 0.24)
        self.fleet = load_fleet(self.get_parameter('fleet_config').value)
        self.max_age = self.get_parameter('max_pose_age').value
        self.radius = self.get_parameter('robot_radius').value
        self.tf = FleetTF(self, [r.name for r in self.fleet])

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_all = self.create_publisher(PoseArray, '/fleet/poses', 1)
        self.pub_health = self.create_publisher(String, '/fleet/health', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/fleet/robot_markers', 1)
        self.pub_peers = {r.name: self.create_publisher(PoseArray, f'/{r.name}/peer_poses',
                                                        best_effort) for r in self.fleet}
        self.lost = set()
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info(f'tracking {[r.name for r in self.fleet]}')

    def tick(self):
        now = self.get_clock().now()
        poses, health = {}, {}
        for r in self.fleet:
            p = self.tf.lookup(r.name)
            age = self.tf.age(p)
            health[r.name] = round(age, 2) if age != float('inf') else None
            if age <= self.max_age:
                poses[r.name] = p
                if r.name in self.lost:
                    self.lost.discard(r.name)
                    self.get_logger().info(f'{r.name}: pose recovered')
            elif r.name not in self.lost and now.nanoseconds > 40e9:
                self.lost.add(r.name)
                self.get_logger().warn(f'{r.name}: no fresh pose (age={age:.1f}s)')

        def to_pose(p):
            msg = Pose()
            msg.position.x, msg.position.y = p.x, p.y
            msg.orientation = quat_from_yaw(p.yaw)
            return msg

        stamp = now.to_msg()
        all_msg = PoseArray()
        all_msg.header.frame_id, all_msg.header.stamp = 'map', stamp
        for r in self.fleet:
            last = self.tf.last.get(r.name)
            if last is None:   # not localized yet: report the configured start pose
                pose = Pose()
                pose.position.x, pose.position.y = r.x, r.y
                pose.orientation = quat_from_yaw(r.yaw)
            else:
                pose = to_pose(last)
            all_msg.poses.append(pose)
        self.pub_all.publish(all_msg)

        for r in self.fleet:
            msg = PoseArray()
            msg.header.frame_id, msg.header.stamp = 'map', stamp
            msg.poses = [to_pose(p) for n, p in poses.items() if n != r.name]
            self.pub_peers[r.name].publish(msg)

        self.pub_health.publish(String(data=json.dumps(health)))
        self.publish_markers(poses, stamp)

    def publish_markers(self, poses, stamp):
        arr = MarkerArray()
        for k, r in enumerate(self.fleet):
            p = poses.get(r.name)
            c = COLORS[k % len(COLORS)]
            for mid, mtype in ((0, Marker.CYLINDER), (1, Marker.TEXT_VIEW_FACING)):
                m = Marker()
                m.header.frame_id, m.header.stamp = 'map', stamp
                m.ns, m.id, m.type = r.name, mid, mtype
                if p is None:
                    m.action = Marker.DELETE
                    arr.markers.append(m)
                    continue
                m.pose.position.x, m.pose.position.y = p.x, p.y
                m.color.r, m.color.g, m.color.b = c
                if mtype == Marker.CYLINDER:
                    m.scale.x = m.scale.y = 2 * self.radius
                    m.scale.z = 0.02
                    m.color.a = 0.35
                else:
                    m.pose.position.z = 0.5
                    m.scale.z = 0.25
                    m.color.a = 1.0
                    m.text = r.name
                arr.markers.append(m)
        self.pub_markers.publish(arr)


def main():
    spin_node(FleetMonitor)
