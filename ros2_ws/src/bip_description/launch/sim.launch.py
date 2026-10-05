import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('bip_description')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    world = os.path.join(pkg, 'worlds', 'bip_world.world')
    xacro_file = os.path.join(pkg, 'urdf', 'bip_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
                                       value_type=str)

    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = gzserver only (no GUI)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world, 'pause': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')),
            condition=UnlessCondition(headless)),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}]),

        Node(package='gazebo_ros', executable='spawn_entity.py',
             output='screen',
             arguments=['-topic', 'robot_description', '-entity', 'bip_robot',
                        '-x', '-4.0', '-y', '3.0', '-z', '0.05']),
    ])
