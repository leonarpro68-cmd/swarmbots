#!/usr/bin/env python3
"""Validaciones offline de la salida CBS y su continuidad."""
import json,unittest
from pathlib import Path
from supermarket_planning import cbs,geometry
ROOT=Path(__file__).resolve().parent
class StageTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.d=json.loads((ROOT/'results/milp_cbs_seed100.json').read_text())
 def test_four_stages(self):self.assertEqual(list(self.d['stages']),['pickup1','delivery1','pickup2','delivery2'])
 def test_graph(self):self.assertEqual(self.d['graph']['nodes'],588);self.assertEqual(self.d['graph']['edges'],1786)
 def test_unique_trash(self):self.assertEqual(len({a[s] for a in self.d['assignment'].values() for s in ('pickup1','pickup2')}),6)
 def test_safe_paths(self):
  for stage,data in self.d['stages'].items():
   paths=tuple(tuple(tuple(node) for node in p['grid']) for p in data['paths'])
   self.assertIsNone(cbs.first_conflict(paths),stage)
   for p,path in zip(data['paths'],paths):
    self.assertEqual(tuple(p['start_grid']),path[0]);self.assertEqual(tuple(p['goal_grid']),path[-1]);self.assertEqual(p['ticks'],len(path)-1)
 def test_continuity(self):
  sequence=list(self.d['stages'].values())
  for earlier,later in zip(sequence,sequence[1:]):self.assertEqual(earlier['goals_grid'],later['starts_grid'])
 def test_unexecutable(self):self.assertFalse(self.d['execution_authorized']);self.assertEqual(self.d['status'],'OFFLINE_CBS_ONLY_NOT_EXECUTABLE')
if __name__=='__main__':unittest.main()
