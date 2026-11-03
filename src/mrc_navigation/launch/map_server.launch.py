"""One shared map server publishing /map (frame 'map') for every robot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('mrc_gazebo'), 'maps', 'coverage_arena.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{'yaml_filename': LaunchConfiguration('map'),
                          'topic_name': 'map', 'frame_id': 'map',
                          'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server'], 'bond_timeout': 10.0}]),
    ])
