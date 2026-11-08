"""Etapa 7B.2: Gazebo + 3 seguidores + 6 basuras y 3 depositos; SIN CBS."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('swarm_worlds')
    base = os.path.join(share, 'launch', 'swarm_cbs_base.launch.py')
    preparer = os.path.join(share, 'launch', 'swarm_scenario_prepare.py')
    seed = LaunchConfiguration('seed')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('world', default_value='tugbot_warehouse'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('scenario', default_value='/tmp/swarm_cbs_escenario.json'),
        DeclareLaunchArgument('scripts_dir', default_value=os.path.expanduser('~/swarm_ws/scripts')),
        DeclareLaunchArgument('ready_timeout', default_value='180'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base), launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items()),
        ExecuteProcess(cmd=[
            'python3', '-u', preparer,
            '--seed', seed,
            '--scenario', LaunchConfiguration('scenario'),
            '--scripts-dir', LaunchConfiguration('scripts_dir'),
            '--ready-timeout', LaunchConfiguration('ready_timeout'),
        ], output='screen', emulate_tty=True),
    ])
