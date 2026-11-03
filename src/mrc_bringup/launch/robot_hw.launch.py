"""Run ON EACH REAL TurtleBot3 (Raspberry Pi), e.g.:

  ROS_DOMAIN_ID=42 ros2 launch mrc_bringup robot_hw.launch.py namespace:=robot1 lidar:=LDS-02

Starts the same interface the simulation provides for one robot:
  /<ns>/odom, /<ns>/scan, /<ns>/imu, /<ns>/joint_states, /<ns>/cmd_vel  (topics)
  <ns>/odom -> <ns>/base_footprint -> <ns>/base_scan ...                (frames on /tf)

Requires the official TurtleBot3 Jazzy packages on the robot
(turtlebot3_node + ld08_driver for LDS-02 or hls_lfcd_lds_driver for LDS-01).
UNTESTED ON HARDWARE IN THIS PROJECT - verify parameter names against your
turtlebot3 release (see docs/USER_GUIDE.md, 'Hardware deployment').
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    prefix = f'{ns}/'
    lidar = LaunchConfiguration('lidar').perform(context)
    port_cr = LaunchConfiguration('opencr_port').perform(context)
    port_lds = LaunchConfiguration('lidar_port').perform(context)
    with open(os.path.join(get_package_share_directory('mrc_description'), 'urdf',
                           'turtlebot3_waffle.urdf')) as f:
        urdf = f.read()
    tb3_params = {
        'opencr.id': 200, 'opencr.baud_rate': 1000000, 'opencr.protocol_version': 2.0,
        'wheels.separation': 0.287, 'wheels.radius': 0.033,
        'motors.profile_acceleration_constant': 214.577, 'motors.profile_acceleration': 0.0,
        'sensors.bumper_1': 0, 'sensors.bumper_2': 0, 'sensors.illumination': 0,
        'sensors.ir': 0, 'sensors.sonar': 0,
        # frames get the robot prefix -> one shared TF tree for the whole fleet
        'odometry.publish_tf': True, 'odometry.use_imu': True,
        'odometry.frame_id': f'{prefix}odom', 'odometry.child_frame_id': f'{prefix}base_footprint',
    }
    if lidar == 'LDS-02':
        lidar_node = Node(package='ld08_driver', executable='ld08_driver', namespace=ns,
                          name='lidar', output='screen',
                          parameters=[{'frame_id': f'{prefix}base_scan', 'port': port_lds}])
    else:
        lidar_node = Node(package='hls_lfcd_lds_driver', executable='hlds_laser_publisher',
                          namespace=ns, name='lidar', output='screen',
                          parameters=[{'port': port_lds, 'frame_id': f'{prefix}base_scan'}])
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
             output='screen', parameters=[{'robot_description': urdf, 'frame_prefix': prefix}]),
        Node(package='turtlebot3_node', executable='turtlebot3_ros', namespace=ns,
             output='screen', arguments=['-i', port_cr],
             # a dict is applied with a '/**' wildcard, so it also reaches the
             # diff_drive_controller node that turtlebot3_ros creates internally
             parameters=[tb3_params]),
        lidar_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('lidar', default_value='LDS-02', choices=['LDS-01', 'LDS-02']),
        DeclareLaunchArgument('opencr_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        OpaqueFunction(function=setup),
    ])
