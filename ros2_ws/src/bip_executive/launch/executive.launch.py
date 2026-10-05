import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('bip_executive')
    missions = os.path.join(pkg, 'config', 'missions.yaml')
    engine = LaunchConfiguration('engine')
    wait_for_start = ParameterValue(LaunchConfiguration('wait_for_start'),
                                    value_type=bool)

    common = [{'missions_file': missions,
               'dock_x': -5.2, 'dock_y': 3.2,
               'wait_for_start': wait_for_start,
               'use_sim_time': True}]

    def exe(name):
        from launch.conditions import LaunchConfigurationEquals
        return Node(package='bip_executive', executable=f'{name}_executive',
                    output='screen', parameters=common,
                    condition=LaunchConfigurationEquals('engine', name))

    return LaunchDescription([
        DeclareLaunchArgument('engine', default_value='bt',
                              description='bt | fsm | utility'),
        DeclareLaunchArgument(
            'wait_for_start', default_value='false',
            description='true = wait for deterministic /benchmark/start gate'),
        exe('bt'), exe('fsm'), exe('utility'),
    ])
