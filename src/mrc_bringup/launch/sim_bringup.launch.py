"""ONE command for the whole simulation:

  ros2 launch mrc_bringup sim_bringup.launch.py [gui:=true] [rviz:=true]

Gazebo + 3 TurtleBot3s -> shared map server -> one Nav2 stack per robot
(staggered so they don't all activate in the same instant) -> fleet nodes -> RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mrc_fleet.fleet_config import default_fleet_file, load_fleet


def launch_file(pkg, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), 'launch', name))


def nav_stacks(context):
    fleet = load_fleet(LaunchConfiguration('fleet_config').perform(context))
    delay = float(LaunchConfiguration('nav_start_delay').perform(context))
    return [TimerAction(period=delay + 4.0 * i, actions=[IncludeLaunchDescription(
        launch_file('mrc_navigation', 'robot_nav.launch.py'),
        launch_arguments={'namespace': r.name, 'x': str(r.x), 'y': str(r.y),
                          'yaw': str(r.yaw), 'use_sim_time': 'true',
                          'log_level': LaunchConfiguration('log_level')}.items())])
        for i, r in enumerate(fleet)]


def generate_launch_description():
    bringup = get_package_share_directory('mrc_bringup')
    fleet_cfg = LaunchConfiguration('fleet_config')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('fleet_config', default_value=default_fleet_file()),
        DeclareLaunchArgument('nav_start_delay', default_value='8.0',
                              description='s after Gazebo start before Nav2 stacks launch'),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(launch_file('mrc_gazebo', 'sim.launch.py'),
                                 launch_arguments={'gui': LaunchConfiguration('gui'),
                                                   'fleet_config': fleet_cfg}.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            launch_file('mrc_navigation', 'map_server.launch.py'),
            launch_arguments={'use_sim_time': 'true'}.items())]),
        OpaqueFunction(function=nav_stacks),
        TimerAction(period=6.0, actions=[IncludeLaunchDescription(
            launch_file('mrc_bringup', 'fleet_nodes.launch.py'),
            launch_arguments={'use_sim_time': 'true', 'fleet_config': fleet_cfg}.items())]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(bringup, 'rviz', 'fleet.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
