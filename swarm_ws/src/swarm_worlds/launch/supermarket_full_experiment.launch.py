#!/usr/bin/env python3
"""One launch for scenario seed100, support, and unattended supervised mission.

Requires an unused Gazebo/ROS graph and existing certified prepared contracts.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration('assignment_mode').perform(context).strip().lower()
    seed_string = LaunchConfiguration('seed').perform(context).strip()
    if mode not in ('fixed', 'milp'):
        raise ValueError('assignment_mode debe ser fixed o milp')
    if seed_string != '100':
        raise ValueError('Sólo seed:=100: no reutilizar NPZ/V3 de seed100 en otras semillas. Primero generar y validar rutas para esa semilla.')
    ws = Path(LaunchConfiguration('workspace').perform(context)).expanduser().resolve()
    scene = ws/'config/supermarket/seed_100.json'
    prepared = (ws/'scripts/supermarket_auto_experiment/fixed_seed100_prepared' if mode=='fixed'
                else ws/'scripts/supermarket_milp_phase5/results/milp_seed100_01')
    runner = ws/'scripts/supermarket_one_launch/run_unique_mission.py'
    for p in (scene, prepared/'phase5_static_preflight.json', runner):
        if not p.is_file():
            raise FileNotFoundError(f'Falta archivo requerido: {p}')
    pkg = Path(get_package_share_directory('swarm_worlds'))/'launch'
    scene_launch = pkg/'sim_summit_supermarket.launch.py'
    support_launch = pkg/'supermarket_support.launch.py'
    for p in (scene_launch, support_launch):
        if not p.is_file():raise FileNotFoundError(f'Falta launch: {p}')
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene_launch)), launch_arguments={
        'world':'supermarket','scenario_json':str(scene),'seed':'100','n_robots':'3',
        'n_drones':'0','n_trash':'6','n_deposits':'3','gripper':'true',
        'headless':LaunchConfiguration('headless')}.items())
    support=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(support_launch)),
        launch_arguments={'n_robots':'3','world_name':'world_demo'}.items())
    worker=ExecuteProcess(cmd=['python3',str(runner),'--workspace',str(ws),'--prepared',str(prepared),
        '--assignment-mode',mode,'--seed','100','--results-root',LaunchConfiguration('results_root'),
        '--readiness-seconds',LaunchConfiguration('readiness_seconds')],
        cwd=str(ws), output='screen', name='supermarket_experiment_controller')
    # ROS/Gazebo startup is asynchronous; worker itself verifies readiness and initial poses.
    return [gazebo,TimerAction(period=5.0,actions=[support]),TimerAction(period=8.0,actions=[worker])]


def generate_launch_description():
    home=os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('assignment_mode',default_value='fixed'),
        DeclareLaunchArgument('seed',default_value='100'),
        DeclareLaunchArgument('workspace',default_value=os.path.join(home,'swarm_ws')),
        DeclareLaunchArgument('results_root',default_value=os.path.join(home,'swarm_ws','scripts','resultados','experiments')),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('readiness_seconds',default_value='180'),
        OpaqueFunction(function=_setup),
    ])
