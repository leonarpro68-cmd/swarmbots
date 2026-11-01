#!/usr/bin/env python3
"""Independent, supervised new-seed supermarket launch. Does NOT plan while Gazebo runs.

Unlike supermarket_full_experiment.launch.py, this reads seed-scoped, complete
and verified offline artifacts. The old, validated launch is untouched.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def setup(context,*args,**kwargs):
    seedstr=LaunchConfiguration('seed').perform(context)
    if not seedstr.isdigit():raise ValueError('Se requiere seed no negativa')
    seed=int(seedstr)
    if seed==100:raise ValueError('Para seed100 utiliza el lanzamiento ya validado supermarket_full_experiment.launch.py')
    if LaunchConfiguration('supervised').perform(context).lower()!='true':
        raise ValueError('Nuevas semillas requieren supervised:=true y un operador presente durante toda la misión')
    ws=Path(LaunchConfiguration('workspace').perform(context)).expanduser().resolve()
    scene=ws/'config/supermarket'/f'seed_{seed}.json'
    prepared=ws/'scripts/supermarket_seed_pipeline/plans'/f'seed_{seed}'/'prepared'
    runner=ws/'scripts/supermarket_seed_pipeline/run_prepared_mission.py'
    for f in (scene,prepared/'phase5_static_preflight.json',runner,ws/'scripts/supermarket_seed_pipeline/run_auto_experiment_seed.py'):
        if not f.is_file():raise FileNotFoundError(f'No se puede lanzar una misión sin plan preparado: {f}')
    import sys
    sys.path.insert(0,str(ws/'scripts/supermarket_seed_pipeline'))
    from run_prepared_mission import validate
    validate(ws,seed)  # blocking gate BEFORE starting Gazebo
    pkg=Path(get_package_share_directory('swarm_worlds'))/'launch'
    launch_world=pkg/'sim_summit_supermarket.launch.py'
    launch_support=pkg/'supermarket_support.launch.py'
    if not launch_world.is_file() or not launch_support.is_file():raise FileNotFoundError('Launch de mundo/soporte no encontrado')
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_world)), launch_arguments={
        'world':'supermarket','scenario_json':str(scene),'seed':str(seed),'n_robots':'3','n_drones':'0',
        'n_trash':'6','n_deposits':'3','gripper':'true','headless':LaunchConfiguration('headless')}.items())
    support=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_support)),
        launch_arguments={'n_robots':'3','world_name':'world_demo'}.items())
    worker=ExecuteProcess(cmd=['python3',str(runner),'--workspace',str(ws),'--seed',str(seed),
        '--results-root',LaunchConfiguration('results_root'),'--readiness-seconds',LaunchConfiguration('readiness_seconds')],
        cwd=str(ws),output='screen',name='supermarket_prepared_seed_controller')
    return [gazebo,TimerAction(period=5.,actions=[support]),TimerAction(period=8.,actions=[worker])]


def generate_launch_description():
    home=os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('seed',default_value='42'),
        DeclareLaunchArgument('supervised',default_value='false'),
        DeclareLaunchArgument('workspace',default_value=os.path.join(home,'swarm_ws')),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('readiness_seconds',default_value='180'),
        DeclareLaunchArgument('results_root',default_value=os.path.join(home,'swarm_ws','scripts','resultados','experiments')),
        OpaqueFunction(function=setup),
    ])
