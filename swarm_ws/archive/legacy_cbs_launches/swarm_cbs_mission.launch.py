"""7B.3: sim + followers + scenario + 6E, sequentially and without modifying 7B.1/7B.2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('swarm_worlds')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='tugbot_warehouse', description='SDF filename, not Gazebo internal world name'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('scenario', default_value='/tmp/swarm_cbs_escenario.json'),
        DeclareLaunchArgument('scripts_dir', default_value=os.path.expanduser('~/swarm_ws/scripts')),
        DeclareLaunchArgument('ready_timeout', default_value='180'),
        DeclareLaunchArgument('mission_state_file', default_value='/tmp/swarm_refactor_7b3.json'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'swarm_cbs_base.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'headless': LaunchConfiguration('headless'),
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                'python3', '-u', os.path.join(share, 'launch', 'swarm_mission_start.py'),
                '--world-file', LaunchConfiguration('world'),
                '--seed', LaunchConfiguration('seed'),
                '--scenario', LaunchConfiguration('scenario'),
                '--scripts-dir', LaunchConfiguration('scripts_dir'),
                '--ready-timeout', LaunchConfiguration('ready_timeout'),
                '--mission-state-file', LaunchConfiguration('mission_state_file'),
            ],
            output='screen', emulate_tty=True,
        ),
    ])
