"""Base-station fleet nodes: fleet_monitor, coverage_coordinator, formation_controller.
Identical for simulation (use_sim_time:=true) and real robots (use_sim_time:=false)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mrc_fleet.fleet_config import default_fleet_file


def generate_launch_description():
    bringup = get_package_share_directory('mrc_bringup')
    params = LaunchConfiguration('fleet_params')
    common = [params, {'use_sim_time': LaunchConfiguration('use_sim_time'),
                       'fleet_config': LaunchConfiguration('fleet_config')}]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fleet_config', default_value=default_fleet_file()),
        DeclareLaunchArgument('fleet_params', default_value=os.path.join(
            bringup, 'config', 'fleet_params.yaml')),
        DeclareLaunchArgument('coverage', default_value='true'),
        DeclareLaunchArgument('formation', default_value='true'),
        Node(package='mrc_fleet', executable='fleet_monitor', name='fleet_monitor',
             output='screen', parameters=common, respawn=True, respawn_delay=2.0),
        Node(package='mrc_fleet', executable='coverage_coordinator',
             name='coverage_coordinator', output='screen', parameters=common,
             condition=IfCondition(LaunchConfiguration('coverage'))),
        Node(package='mrc_fleet', executable='formation_controller',
             name='formation_controller', output='screen',
             parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('formation'))),
    ])
