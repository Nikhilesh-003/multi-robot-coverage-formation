"""Nav2 stack (AMCL + navigation) for ONE fleet robot.

Topics live in /<namespace>, TF frames are '<namespace>/...' on the shared /tf
(so unlike nav2_bringup we do NOT remap /tf -> tf).

  ros2 launch mrc_navigation robot_nav.launch.py namespace:=robot1 x:=-4.2 y:=0 yaw:=0
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_NODES = ['amcl', 'controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'velocity_smoother', 'collision_monitor',
             'bt_navigator', 'waypoint_follower']


def render_params(template: str, ns: str, x: float, y: float, yaw: float) -> str:
    """Fill the template for one robot and write it, nested under its namespace."""
    with open(template) as f:
        text = f.read().replace('<prefix>', f'{ns}/').replace('<ns>', ns)
    params = yaml.safe_load(text)
    params['amcl']['ros__parameters']['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    fd, path = tempfile.mkstemp(prefix=f'mrc_nav2_{ns}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump({ns: params}, f, sort_keys=False)
    return path


def launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    respawn = LaunchConfiguration('use_respawn').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    params = render_params(
        LaunchConfiguration('params_file').perform(context), ns,
        float(LaunchConfiguration('x').perform(context)),
        float(LaunchConfiguration('y').perform(context)),
        float(LaunchConfiguration('yaw').perform(context)))
    common = dict(namespace=ns, output='screen', respawn=respawn, respawn_delay=2.0,
                  parameters=[params, {'use_sim_time': use_sim_time}],
                  arguments=['--ros-args', '--log-level', log_level])
    nav = 'cmd_vel_nav'
    nodes = [
        Node(package='nav2_amcl', executable='amcl', name='amcl', **common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             remappings=[('cmd_vel', nav)], **common),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             remappings=[('cmd_vel', nav)], **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', remappings=[('cmd_vel', nav)], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns, output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': NAV_NODES,
                          # generous bond: 3 full stacks + Gazebo can starve a node briefly
                          'bond_timeout': 10.0,
                          'attempt_respawn_reconnection': True,
                          'bond_respawn_max_duration': 30.0}]),
    ]
    return nodes


def generate_launch_description():
    nav_dir = get_package_share_directory('mrc_navigation')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            nav_dir, 'params', 'nav2_fleet_params.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
