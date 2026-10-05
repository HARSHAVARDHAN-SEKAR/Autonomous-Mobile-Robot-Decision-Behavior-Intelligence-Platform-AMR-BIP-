import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    env_file = os.path.join(get_package_share_directory('bip_navigation'),
                            'config', 'environment.yaml')
    return LaunchDescription([
        Node(package='bip_navigation', executable='nav_server',
             output='screen',
             parameters=[{'environment_file': env_file,
                          'use_sim_time': True}]),
        Node(package='bip_battery', executable='battery_node',
             output='screen',
             parameters=[{'environment_file': env_file,
                          'use_sim_time': True}]),
        Node(package='bip_chaos', executable='chaos_node',
             output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
