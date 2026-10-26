"""Offline tests: scenario spawn/odometry consistency (no Gazebo/ROS required)."""
import ast
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent.parent
WORKSPACE=HERE/'swarm_ws'
sys.path.insert(0, str(WORKSPACE/'src/swarm_behavior'))
sys.path.insert(0, str(WORKSPACE/'scripts'))
import grafo_cbs_125 as graph
from escenario_aleatorio_swarm import build
from d8_verify_initial_positions import expected_from_json,validate_positions

launch=WORKSPACE/'src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py'
module=ast.parse(launch.read_text())
f=next(n for n in module.body if isinstance(n,ast.FunctionDef) and n.name=='_scenario_robot_specs')
code=compile(ast.Module(body=[f],type_ignores=[]),str(launch),'exec')
ns={'math':math,'random':__import__('random')}
exec(code, ns)
scenario_robot_specs=ns['_scenario_robot_specs']

class Seeds(unittest.TestCase):
  def test_grid_valid(self):
    for seed in (42,123,7,2026,100,789):
      with self.subTest(seed=seed):
        data=build(seed)
        specs=scenario_robot_specs(data,seed)
        self.assertEqual([n for n, *_ in specs], ['summit0','summit1','summit2'])
        self.assertEqual([(x,y) for _,x,y,_ in specs],[tuple(x['xy_m']) for x in data['robots']])
        self.assertEqual(specs,scenario_robot_specs(data,seed))
        self.assertTrue(all(graph.free_position((x,y)) for _,x,y,_ in specs))
        obs={name:{'xy':(x+0.02,y),'speed':0.01,'frame':f'{name}/odom'} for name,x,y,_ in specs}
        detail,issues=validate_positions(expected_from_json(data), obs)
        self.assertFalse(issues,issues)
        self.assertLess(max(d['distance_m'] for d in detail.values()),0.25)
  def test_seed42_specific(self):
    specs=scenario_robot_specs(build(42),42)
    self.assertEqual((specs[2][1],specs[2][2]),(3.75,-21.25))
  def test_wrong_spawn_rejected(self):
    data=build(42)
    exp=expected_from_json(data)
    obs={name:{'xy':p,'speed':0.0,'frame':f'{name}/odom'} for name,p in exp.items()}
    obs['summit2']['xy']=(0.0,0.0)
    detail,issues=validate_positions(exp,obs)
    self.assertTrue(any('summit2' in x for x in issues))
  def test_frame_rejected(self):
    exp=expected_from_json(build(42))
    obs={name:{'xy':p,'speed':0.0,'frame':'wrong'} for name,p in exp.items()}
    _,issues=validate_positions(exp,obs)
    self.assertEqual(len(issues),3)
  def test_robot_on_blocked_vertex_rejected(self):
    data=build(42)
    data['robots'][2]['grid']=[0,0]
    data['robots'][2]['xy_m']=list(graph.node_position((0,0)))
    with self.assertRaises(ValueError): scenario_robot_specs(data,42)
  def test_robot_vs_trash_rejected(self):
    data=build(42)
    data['robots'][2]['grid']=data['elements'][0]['grid'][:]
    data['robots'][2]['xy_m']=data['elements'][0]['xy_m'][:]
    with self.assertRaises(ValueError): scenario_robot_specs(data,42)

if __name__=='__main__':unittest.main(verbosity=2)
