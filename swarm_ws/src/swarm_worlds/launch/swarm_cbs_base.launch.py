"""Etapa 7B.1: inicia Gazebo, tres Summit XLS y sus seguidores.

Sin basura, depósitos ni planificador CBS. No modifica sim_summit.launch.py.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    original = os.path.join(
        get_package_share_directory('swarm_worlds'),
        'launch', 'sim_summit.launch.py'
    )
    actions = [
        DeclareLaunchArgument('world', default_value='tugbot_warehouse'),
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(original),
            launch_arguments={
                'n_robots': '3',
                'n_drones': '0',
                'gripper': 'false',
                'n_trash': '0',
                'n_deposits': '0',
                'world': LaunchConfiguration('world'),
                'headless': LaunchConfiguration('headless'),
                'seed': LaunchConfiguration('seed'),
            }.items(),
        ),
    ]
    for idx in range(3):
        actions.append(Node(
            package='swarm_behavior',
            executable='swarm_path_follower',
            namespace=f'summit{idx}',
            name='path_follower',
            parameters=[{
                'max_speed': 0.60,
                'kp': 1.2,
                'waypoint_tolerance': 0.08,
            }],
            output='screen',
        ))
    return LaunchDescription(actions)
