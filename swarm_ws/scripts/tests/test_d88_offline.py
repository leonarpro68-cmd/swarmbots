"""Regression: seed 42, delivery1, odometry copied from original failure.

Run inside ~/swarm_ws after sourcing ROS, optionally with:
  PYTHONPATH=src/swarm_behavior:./scripts python3 tests/test_d88_offline.py
This test plans but never sends commands to Gazebo.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
try:
    from swarm_behavior.swarm_planning.bridge import PlanningBridge
    from swarm_behavior.swarm_planning import geometry as grid
    from swarm_behavior.swarm_planning.scenario import Scenario
    from swarm_behavior.swarm_planning.objects import ELEMENTS, Element
    from d88_margin_graph import margin_compatible_graph
    from swarm_behavior.swarm_planning.hybrid_v2.live_start import exact_start_inputs
    from swarm_behavior.swarm_planning.hybrid_v2.geometry_corridors import corridors
    from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as geom
    from swarm_behavior.swarm_planning.hybrid_v2 import geometry_corridors as gc
except ImportError:
    sys.path.insert(0,'/tmp/d88_lib')
    from swarm_behavior.swarm_planning.bridge import PlanningBridge
    from swarm_behavior.swarm_planning import geometry as grid
    from swarm_behavior.swarm_planning.scenario import Scenario
    from swarm_behavior.swarm_planning.objects import ELEMENTS, Element
    from d88_margin_graph import margin_compatible_graph
    from swarm_behavior.swarm_planning.hybrid_v2.live_start import exact_start_inputs
    from swarm_behavior.swarm_planning.hybrid_v2.geometry_corridors import corridors
    from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as geom
    from swarm_behavior.swarm_planning.hybrid_v2 import geometry_corridors as gc

COORDS={
 'trash_V1_verde':(0,-16.25),'trash_V2_verde':(-3.75,-1.25),
 'trash_N1_naranja':(1.25,21.25),'trash_N2_naranja':(-6.25,8.75),
 'trash_A1_azul':(-8.75,-13.75),'trash_A2_azul':(0,10),
 'deposit_verde':(7.5,-5),'deposit_naranja':(6.25,-12.5),
 'deposit_azul':(10,20)
}
ROBOTS=((-6.25,20),(-7.5,-5),(3.75,-21.25))
POS=((.05371641145284157,20.052647292579845),
 (-4.946597805841393,-2.444120149050859),
 (1.1973953022857207,-17.4498661437327))

def node(p):
 return (round((p[0]-grid.ORIGIN_X)/grid.STEP_M),
         round((p[1]-grid.ORIGIN_Y)/grid.STEP_M))

def case():
 catalog={e.name:e for e in ELEMENTS}
 sc=Scenario(42,tuple(Element(n,catalog[n].color,node(xy),catalog[n].kind)
                       for n,xy in COORDS.items()),tuple(map(node,ROBOTS)))
 br=PlanningBridge(sc)
 initial=br.plan_pickup(tuple(map(node,ROBOTS)))
 names=tuple(n for n,_ in initial.assigned)
 for n in names:br.confirm_collected(n)
 starts=tuple(map(node,POS))
 plan=br.plan_delivery(starts,names)
 adapted,points,_=exact_start_inputs(plan,br.graph,POS,max_snap_m=.75,stage='delivery1')
 return sc,br,starts,names,adapted,points

def test_filter():
 sc,br,starts,names,adapted,points=case()
 originals=(br.graph.vertices,br.graph.edges,br.graph.neighbors)
 r,c=geom.CLEARANCE_WALL_M,gc.R
 try:
  geom.CLEARANCE_WALL_M=gc.R=.55
  try:
   corridors(points,adapted.obstacles,operational_margin_m=.05)
   raise AssertionError('Expected seed 42 original rejection')
  except ValueError as e:
   assert 'd=0.884, r=0.896' in str(e),str(e)
  with margin_compatible_graph(br.graph,adapted.obstacles,.05) as info:
   assert info['removed_edges'] == 10, info
   new=br.plan_delivery(starts,names)
   a2,p2,_=exact_start_inputs(new,br.graph,POS,max_snap_m=.75,stage='delivery1')
   corridors(p2,a2.obstacles,operational_margin_m=.05)
  assert all(getattr(br.graph,key) is ref for key,ref in zip(('vertices','edges','neighbors'),originals))
  print('PASS: original route rejected; 10 edges removed; alternative corridors valid; graph restored')
 finally:
  geom.CLEARANCE_WALL_M,gc.R=r,c


def test_script():
 sc,br,starts,names,_,_=case()
 with tempfile.TemporaryDirectory() as t:
  root=Path(t);scenario=root/'scenario.json';scenario.write_text(json.dumps(sc.to_dict()))
  state=root/'state.json'; state.write_text(json.dumps({
   'version':'swarmbots.d6.v1','scenario':str(scenario),
   'scenario_sha256':hashlib.sha256(scenario.read_bytes()).hexdigest(),
   'completed':['pickup1'], 'round1_names':list(names),'collected':list(names)
  }))
  # ROS is only imported, not used. Replace snapshot with the recorded odometry.
  try:
   import rclpy
  except ImportError:
   from types import ModuleType
   def m(name):
    mod=ModuleType(name);sys.modules[name]=mod;return mod
   rclpy=m('rclpy');node=m('rclpy.node');node.Node=object
   nav=m('nav_msgs');msg=m('nav_msgs.msg');msg.Odometry=object
   rg=m('rosgraph_msgs');clock=m('rosgraph_msgs.msg');clock.Clock=object
  spec=importlib.util.spec_from_file_location('d88_prepare',ROOT/'prepare_hybrid_v2_d6_stage.py')
  module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  module.snapshot=lambda *_args: (POS,321.0,{f'summit{i}':f'summit{i}/odom' for i in range(3)})
  oldargv=sys.argv
  try:
   sys.argv=['prepare','--scenario',str(scenario),'--state',str(state),
    '--stage','delivery1','--accept-namespaced-world-odom',
    '--output-dir',str(root/'results'),'--radius','.55','--margin','.05',
    '--max-speed','1.20','--max-accel','1.00','--iterations','2']
   assert module.main()==0
  finally:sys.argv=oldargv
  report=json.loads((root/'results/delivery1_live_report.json').read_text())
  v=report['stages']['delivery1']['hybrid_v2']
  assert v['certified'] and v['status']=='CERTIFIED'
  assert v['replanning_d88']['graph_filter']['removed_edges']==10
  assert v['operational_margin_m']==.05
  assert (root/'results/delivery1_live_controls.npz').exists()
  print('PASS: prepare script returns CERTIFIED; margin unchanged; outputs written')

if __name__=='__main__':
 test_filter();test_script()
