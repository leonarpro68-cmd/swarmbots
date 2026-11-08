"""Depositos: barrera de centro 1.10 m en CBS, Hybrid y D8 ajeno.

No modifica mapas ni requiere ejecutar ROS/Gazebo.
"""
from __future__ import annotations
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

WS=Path(__file__).resolve().parents[1]
SCRIPTS=WS/'scripts/supermarket_seed_pipeline'
sys.path.insert(0,str(SCRIPTS))
sys.path.insert(0,str(WS/'scripts'))

from clearance_config import (SUMMIT_RADIUS_M, FREE_GAP_M,
                              VISUAL_DEPOSIT_RADIUS_M,
                              VISUAL_DEPOSIT_NAV_EXCLUSION_M)
from supermarket_planning.dynamic_map import (DEPOSIT_RADIUS_M, DynamicTrashGraph)
from stage_adapter import build_stage
from original_hybrid.geometry_corridors import disk_halfspace
from prepare_phase5 import OTHER_DEPOSIT_EXCLUSION, audit_local
from run_prepared_mission import validate

class DepositClearanceTest(unittest.TestCase):
    def test_single_explicit_center_clearance(self):
        target=VISUAL_DEPOSIT_RADIUS_M+SUMMIT_RADIUS_M+FREE_GAP_M
        self.assertAlmostEqual(target,1.10,places=10)
        self.assertAlmostEqual(VISUAL_DEPOSIT_NAV_EXCLUSION_M,target)
        self.assertAlmostEqual(DEPOSIT_RADIUS_M,target)
        self.assertAlmostEqual(OTHER_DEPOSIT_EXCLUSION,target)

    def test_cbs_graph_rejects_segment_at_one_meter(self):
        scenario={'trash':[],'deposits':[{'name':'deposit_azul','xy_m':[5.0,15.0],
                                         'grid':[16,32],'color':'azul'}]}
        graph=DynamicTrashGraph(scenario,pending=set())
        self.assertFalse(graph.clear_segment((6.0,14.0),(6.0,16.0)))
        self.assertTrue(graph.clear_segment((6.25,14.0),(6.25,16.0)))
        self.assertEqual(len(tuple(graph.obstacle_circles())),1)
        # Un nodo a 1.25 m permanece permitido como meta de entrega.
        self.assertIn(((5.0,15.0),DEPOSIT_RADIUS_M),
                      {(p,r) for _,p,r in graph.obstacle_circles()})

    def test_hybrid_stage_inflates_all_deposits(self):
        scenario={'trash':[], 'deposits':[{'name':'deposit_azul','color':'azul',
                                           'xy_m':[5.0,15.0]}]}
        stage={'paths_grid':[[[0,0],[1,0]]]*3,
               'selected_goals':[[1,0]]*3,'goal_options':[[[1,0]]]*3,
               'pending_before':[]}
        geo={'graph':{'origin':[-15,-25],'step_m':1.25}}
        _,circles,_=build_stage(scenario,stage,geo)
        self.assertEqual(len(circles),3)
        for private in circles:
            self.assertIn(((5.0,15.0),1.10),private)

    def test_regression_old_pickup2_curve_is_outside_new_corridor(self):
        # Punto de la antigua curva Summit1 a 0.511 m del deposito azul,
        # segun el escenario rubber_factory/seed_2026 aportado por el usuario.
        old_point=(4.540,14.777)
        self.assertAlmostEqual(math.dist(old_point,(5.0,15.0)),0.511,delta=0.005)
        with self.assertRaisesRegex(ValueError,'No hay separacion espacial'):
            disk_halfspace(np.array([old_point,old_point]),(5.0,15.0),1.10)
        # El nodo CBS de entrega mas cercano (distancia 1.25) es separable.
        disk_halfspace(np.array([(6.25,15.0),(6.25,15.0)]),(5.0,15.0),1.10)

    def test_local_d8_screens_other_deposit_but_not_its_target(self):
        scene={'trash':[], 'deposits':[
            {'name':'deposit_azul','xy_m':[0.,0.]},
            {'name':'deposit_verde','xy_m':[1.0,0.]},
        ]}
        geom={'obstacles':[]}
        # Segmento a deposito azul cruza deposito verde.
        result=audit_local((2.,0.),(0.,0.),'delivery1','deposit_azul',[],scene,geom)
        self.assertEqual([h['name'] for h in result['other_deposit_hits']],['deposit_verde'])
        self.assertFalse(result['straight_line_geometric_screen_pass'])

    def test_prepared_runner_rejects_old_certificates(self):
        with tempfile.TemporaryDirectory() as temp:
            ws=Path(temp);seed=41
            paths={
                'scene':ws/'config/scenarios/supermarket/seed_41.json',
                'graph':ws/'config/maps/supermarket/grafo.json',
                'geometry':ws/'config/maps/supermarket/obstaculos.json',
                'assignment':ws/'scripts/supermarket_milp/results/supermarket/seed_41/assignment_milp_one_color_per_round.json',
                'manifest':ws/'scripts/supermarket_seed_pipeline/plans/supermarket/seed_41/inputs_manifest.json',
                'pre':ws/'scripts/supermarket_seed_pipeline/plans/supermarket/seed_41/prepared/phase5_static_preflight.json',
                'cbs':ws/'scripts/supermarket_seed_pipeline/plans/supermarket/seed_41/cbs_multigoal.json',
            }
            for key,p in paths.items():
                p.parent.mkdir(parents=True,exist_ok=True)
                p.write_text(json.dumps({'seed':seed,'world':'supermarket'}))
            with self.assertRaisesRegex(ValueError,'PLAN OBSOLETO'):
                validate(ws,seed,mode='milp',map_name='supermarket')

if __name__=='__main__':unittest.main()
