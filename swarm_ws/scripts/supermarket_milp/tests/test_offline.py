#!/usr/bin/env python3
from __future__ import annotations
import copy
import json
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT.parent))
from swarm_maps import map_files
from build_costs import Graph,build,STAGES
from solve_assignment import solve,BASELINE

class TestPhases12(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario=json.loads((ROOT/'data/seed_100.json').read_text())
        cls.gdata=json.loads(map_files(ROOT.parents[1], 'supermarket')[0].read_text())
        cls.costs=build(cls.scenario,cls.gdata)
        cls.solution=solve(cls.costs)

    def test_graph_and_astar_metric(self):
        g=Graph(self.gdata)
        self.assertEqual(len(g.nodes),588)
        self.assertEqual(g.edges_count,1786)
        d,pts=g.astar((-5,22.5),(-5,22.5))
        self.assertEqual(d,0)
        self.assertEqual(len(pts),1)
        a,b=(-13.75,-23.75),(-12.5,-23.75)
        dist,_=g.astar(a,b)
        self.assertAlmostEqual(dist,1.25)

    def test_matrices(self):
        c=self.costs
        self.assertEqual(len(c['pairs']),90)
        self.assertEqual(len(c['robots']),3)
        self.assertEqual(len(c['trash']),6)
        self.assertEqual(len(c['deposit_stands']),3)
        self.assertTrue(all(len(p['distance_by_stage_m'])==4 for p in c['pairs']))
        self.assertTrue(all(all(p['time_by_stage_s'][s]>=0 for s in STAGES) for p in c['pairs']))

    def test_assignment_exhaustive(self):
        o=self.solution
        self.assertEqual(o['status'],'OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE')
        self.assertEqual(o['independent_exhaustive_verification']['evaluated_full_assignments'],720)
        self.assertTrue(o['independent_exhaustive_verification']['global_proxy_makespan_optimum_verified'])
        first=[p['pickup1'] for p in o['chosen'].values()]
        second=[p['pickup2'] for p in o['chosen'].values()]
        self.assertEqual(set(first+second),set(self.costs['trash']))
        self.assertLess(o['optimized_proxy']['sum_stage_makespan_s'],o['validated_fixed_assignment']['proxy']['sum_stage_makespan_s'])

    def test_color_policy(self):
        o=solve(self.costs,True)
        self.assertEqual(o['independent_exhaustive_verification']['evaluated_full_assignments'],288)
        for s in ('pickup1','pickup2'):
            cols=[self.costs['trash'][p[s]]['color'] for p in o['chosen'].values()]
            self.assertEqual(len(set(cols)),3)
        self.assertAlmostEqual(o['optimized_proxy']['sum_stage_makespan_s'],self.solution['optimized_proxy']['sum_stage_makespan_s'])

    def test_speed_proxy_scaling(self):
        half=build(self.scenario,self.gdata,speed=1.)
        self.assertAlmostEqual(half['pairs'][0]['time_by_stage_s']['pickup1'],2*self.costs['pairs'][0]['time_by_stage_s']['pickup1'],places=5)

    def test_missing_node_rejected(self):
        broken=copy.deepcopy(self.scenario)
        broken['trash'][0]['approach_node_xy_m']=[900,900]
        with self.assertRaises(ValueError):build(broken,self.gdata)

    def test_integrity_no_robot_execution(self):
        self.assertFalse(self.solution['execution_authorized'])
        self.assertEqual(self.costs['status'],'OFFLINE_COSTS_NOT_CBS_NOT_EXECUTABLE')
        self.assertEqual(set(BASELINE),set(self.costs['robots']))

if __name__=='__main__':unittest.main()
