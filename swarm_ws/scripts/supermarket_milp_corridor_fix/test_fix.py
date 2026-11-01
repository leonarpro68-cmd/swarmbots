#!/usr/bin/env python3
"""Pruebas offline sobre MILP, CBS multi-meta dinámico y gate geométrico."""
import json, math, tempfile, unittest
from pathlib import Path
from supermarket_planning import cbs, geometry
from supermarket_planning.dynamic_map import DynamicTrashGraph
from plan_milp_multigoal import plan
from audit_phase4 import audit
ROOT=Path(__file__).resolve().parent
SCENE=json.loads((ROOT/'seed_100_reference.json').read_text())
ASSIGN=json.loads((ROOT/'assignment_milp_reference.json').read_text())
class IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result=plan(ASSIGN,SCENE)
    def test_four_stages(self):
        self.assertEqual(list(self.result['stages']), ['pickup1','delivery1','pickup2','delivery2'])
        self.assertFalse(self.result['execution_authorized'])
    def test_milp_assignment_preserved(self):
        self.assertEqual(self.result['assignment'],ASSIGN['chosen'])
    def test_dynamic_object_exclusions(self):
        for stage,record in self.result['stages'].items():
            pending=set(record['pending_before'])
            graph=DynamicTrashGraph(SCENE,pending=pending)
            for p in record['paths']:
                path=[tuple(x) for x in p['grid']]
                self.assertTrue(all(v in graph.vertices for v in path),stage)
                self.assertTrue(all(a==b or b in graph.neighbors[a] for a,b in zip(path,path[1:])),stage)
    def test_robots_conflict_free(self):
        for st,rec in self.result['stages'].items():
            paths=[tuple(tuple(v) for v in p['grid']) for p in rec['paths']]
            self.assertIsNone(cbs.first_conflict(paths),st)
    def test_continuity(self):
        stages=list(self.result['stages'].values())
        for a,b in zip(stages,stages[1:]):self.assertEqual(a['goals_grid'],b['starts_grid'])
    def test_all_goals_are_allowed(self):
        for st,rec in self.result['stages'].items():
            for goal,options in zip(rec['goals_grid'],rec['goal_options']):
                self.assertIn(goal,options,st)
    def test_geometry_passes(self):
        with tempfile.TemporaryDirectory() as d:
            report=Path(d)/'cbs.json';report.write_text(json.dumps(self.result))
            out=Path(d)/'audit.json'
            self.assertEqual(audit(report,ROOT/'seed_100_reference.json',out,ROOT/'supermarket_obstaculos.json'),0)
            a=json.loads(out.read_text())
            self.assertEqual(a['status'],'PASS_CORRIDOR_PREFLIGHT_ONLY')
            self.assertFalse(a['execution_authorized'])
    def test_box_dimensions_modelled_09(self):
        from clearance_config import TRASH_BOX_HALF_DIAGONAL_M
        self.assertAlmostEqual(TRASH_BOX_HALF_DIAGONAL_M, math.hypot(.09,.09)/2)
if __name__=='__main__':unittest.main(verbosity=2)
