"""Run on the BASE STATION (laptop) for REAL robots - everything except Gazebo:

  ROS_DOMAIN_ID=42 ros2 launch mrc_bringup base_station.launch.py map:=/path/to/real_map.yaml

shared map server + one Nav2 stack per robot (use_sim_time:=false) + fleet nodes + RViz.
fleet.yaml x/y/yaw are used as each robot's AMCL initial pose: place the robots there
(or correct with RViz '2D Pose Estimate' on /<robot>/initialpose).
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
    return [TimerAction(period=3.0 + 4.0 * i, actions=[IncludeLaunchDescription(
        launch_file('mrc_navigation', 'robot_nav.launch.py'),
        launch_arguments={'namespace': r.name, 'x': str(r.x), 'y': str(r.y),
                          'yaw': str(r.yaw), 'use_sim_time': 'false'}.items())])
        for i, r in enumerate(fleet)]


def generate_launch_description():
    bringup = get_package_share_directory('mrc_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='map yaml of the real environment'),
        DeclareLaunchArgument('fleet_config', default_value=default_fleet_file()),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(launch_file('mrc_navigation', 'map_server.launch.py'),
                                 launch_arguments={'map': LaunchConfiguration('map'),
                                                   'use_sim_time': 'false'}.items()),
        OpaqueFunction(function=nav_stacks),
        TimerAction(period=2.0, actions=[IncludeLaunchDescription(
            launch_file('mrc_bringup', 'fleet_nodes.launch.py'),
            launch_arguments={'use_sim_time': 'false',
                              'fleet_config': LaunchConfiguration('fleet_config')}.items())]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(bringup, 'rviz', 'fleet.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
