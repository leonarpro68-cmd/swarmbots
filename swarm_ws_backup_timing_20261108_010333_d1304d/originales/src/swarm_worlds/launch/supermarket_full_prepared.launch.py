#!/usr/bin/env python3
"""Supervised multimapa launch using verified, seed-scoped offline artifacts.

Does NOT run planners while Gazebo executes the prepared mission.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import re


def setup(context,*args,**kwargs):
    seedstr=LaunchConfiguration('seed').perform(context)
    mode=LaunchConfiguration('assignment_mode').perform(context).lower()
    map_name=LaunchConfiguration('map').perform(context).lower()
    if not re.fullmatch(r'[a-z][a-z0-9_]*',map_name):
        raise ValueError('Nombre de mapa inválido')
    if mode not in ('fixed','milp'):raise ValueError('assignment_mode debe ser fixed o milp')
    if not seedstr.isdigit():raise ValueError('Se requiere seed no negativa')
    seed=int(seedstr)
    ws=Path(LaunchConfiguration('workspace').perform(context)).expanduser().resolve()
    import sys
    sys.path.insert(0, str(ws/'scripts'))
    from swarm_maps import scenario_file, plans_dir, map_files
    scene=scenario_file(ws,map_name,seed)
    base=plans_dir(ws,map_name,seed)
    map_graph,map_geometry=map_files(ws,map_name)
    prepared=(base if mode=='milp' else base/'fixed')/'prepared'
    runner=ws/'scripts/supermarket_seed_pipeline/run_prepared_mission.py'
    for f in (scene,map_graph,map_geometry,prepared/'phase5_static_preflight.json',runner,ws/'scripts/supermarket_seed_pipeline/run_auto_experiment_seed.py'):
        if not f.is_file():raise FileNotFoundError(f'No se puede lanzar una misión sin plan preparado: {f}')
    import sys
    sys.path.insert(0,str(ws/'scripts/supermarket_seed_pipeline'))
    from run_prepared_mission import validate
    validate(ws,seed,mode,map_name)  # blocking gate BEFORE starting Gazebo
    pkg=Path(get_package_share_directory('swarm_worlds'))/'launch'
    launch_world=pkg/'sim_summit_supermarket.launch.py'
    launch_support=pkg/'supermarket_support.launch.py'
    if not launch_world.is_file() or not launch_support.is_file():raise FileNotFoundError('Launch de mundo/soporte no encontrado')
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_world)), launch_arguments={
        'world':map_name,'map_geometry_json':str(map_geometry),'scenario_json':str(scene),'seed':str(seed),'n_robots':'3','n_drones':'0',
        'n_trash':'6','n_deposits':'3','gripper':'true','headless':LaunchConfiguration('headless')}.items())
    support=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_support)),
        launch_arguments={'n_robots':'3','world_name':'world_demo'}.items())
    worker=ExecuteProcess(cmd=['python3',str(runner),'--workspace',str(ws),'--seed',str(seed),'--map',map_name,'--assignment-mode',mode,
        '--results-root',LaunchConfiguration('results_root'),'--readiness-seconds',LaunchConfiguration('readiness_seconds')],
        cwd=str(ws),output='screen',name='supermarket_prepared_seed_controller')
    return [gazebo,TimerAction(period=5.,actions=[support]),TimerAction(period=8.,actions=[worker])]


def generate_launch_description():
    home=os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('seed',default_value='42'),
        DeclareLaunchArgument('map',default_value='supermarket'),
        DeclareLaunchArgument('assignment_mode',default_value='milp'),
        DeclareLaunchArgument('workspace',default_value=os.path.join(home,'swarm_ws')),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('readiness_seconds',default_value='180'),
        DeclareLaunchArgument('results_root',default_value=os.path.join(home,'swarm_ws','scripts','resultados','experiments')),
        OpaqueFunction(function=setup),
    ])
