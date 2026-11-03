"""Gazebo Harmonic + N TurtleBot3 Waffles, one per entry in fleet.yaml.

Per robot <name>:
  * Gazebo model <name>, topics /<name>/{scan,odom,imu,cmd_vel,joint_states}
  * TF frames prefixed '<name>/' and published on the shared /tf
  * robot_state_publisher in namespace /<name> with frame_prefix '<name>/'
Shared: a single /clock bridge (one per robot would triple-publish the clock)
and the overhead documentation camera /overhead_camera/image.
"""
import os
import tempfile
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mrc_fleet.fleet_config import default_fleet_file, load_fleet


# id-disc colours seen by the overhead camera (same order as snapshot.ROBOT_COLORS)
MARKER_COLORS = ['0.90 0.10 0.29 1', '0.24 0.71 0.29 1', '0.26 0.39 0.85 1',
                 '0.96 0.51 0.19 1', '0.57 0.12 0.71 1', '0.26 0.83 0.96 1']


def robot_bridge_config(name: str) -> str:
    """Write the ros_gz_bridge YAML for one robot and return its path."""
    topics = [
        ('scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', 'GZ_TO_ROS'),
        ('odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
        ('imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'),
        ('joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS'),
        ('cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
    ]
    cfg = [{'ros_topic_name': f'/{name}/{t}', 'gz_topic_name': f'/{name}/{t}',
            'ros_type_name': r, 'gz_type_name': g, 'direction': d} for t, r, g, d in topics]
    # odom->base_footprint from the DiffDrive plugin goes onto the SHARED /tf
    cfg.append({'ros_topic_name': '/tf', 'gz_topic_name': f'/{name}/tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS'})
    fd, path = tempfile.mkstemp(prefix=f'mrc_bridge_{name}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


def launch_setup(context):
    desc_dir = get_package_share_directory('mrc_description')
    fleet = load_fleet(LaunchConfiguration('fleet_config').perform(context))
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes')

    gz_args = f'-r -v 2 {world}' if gui else f'-r -s -v 2 --headless-rendering {world}'
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]

    with open(os.path.join(desc_dir, 'urdf', 'turtlebot3_waffle.urdf')) as f:
        urdf = f.read()
    sdf_xacro = os.path.join(desc_dir, 'urdf', 'tb3_waffle.sdf.xacro')

    for i, r in enumerate(fleet):
        sdf = xacro.process_file(sdf_xacro, mappings={
            'namespace': r.name, 'frame_prefix': r.prefix,
            'color': MARKER_COLORS[i % len(MARKER_COLORS)]}).toxml()
        robot_actions = [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=r.name, output='screen',
                 parameters=[{'robot_description': urdf, 'frame_prefix': r.prefix,
                              'use_sim_time': True}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 namespace=r.name, name='gz_bridge', output='screen',
                 parameters=[{'config_file': robot_bridge_config(r.name), 'use_sim_time': True}]),
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', r.name, '-string', sdf,
                            '-x', str(r.x), '-y', str(r.y), '-z', '0.01', '-Y', str(r.yaw)]),
        ]
        # stagger spawns slightly: simultaneous creates occasionally race the world load
        actions.append(TimerAction(period=2.0 + 1.0 * i, actions=robot_actions))
    return actions


def generate_launch_description():
    gz_dir = get_package_share_directory('mrc_gazebo')
    tb3_sim = get_package_share_directory('nav2_minimal_tb3_sim')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            gz_dir, 'worlds', 'coverage_arena.sdf')),
        DeclareLaunchArgument('fleet_config', default_value=default_fleet_file()),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo GUI (false = server only)'),
        # meshes are referenced as package://nav2_minimal_tb3_sim/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(Path(tb3_sim).parent.resolve())),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_sim, 'models')),
        OpaqueFunction(function=launch_setup),
    ])
