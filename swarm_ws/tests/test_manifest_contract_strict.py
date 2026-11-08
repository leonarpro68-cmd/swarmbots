"""Regresiones sin ROS: manifiestos estrictos MILP/FIXED y rutas sin fallback."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS/'scripts'))
sys.path.insert(0, str(WS/'scripts/supermarket_seed_pipeline'))
sys.path.insert(0, str(WS/'scripts/supermarket_auto_experiment'))
import run_full_offline
import run_prepared_mission
from plot_experiment import find_scenario
from swarm_maps import scenario_file, milp_results_dir, plans_dir, map_files


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data)+'\n', encoding='utf-8')
    return path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class StrictManifestContractTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.ws = Path(self.temp.name)/'swarm_ws'
        self.seed = 90123
        self.map = 'supermarket'
        self.scene = write(scenario_file(self.ws,self.map,self.seed),
                           {'seed':self.seed,'world':self.map})
        self.graph,self.geometry = self.ws/'config/maps/supermarket/grafo.json', self.ws/'config/maps/supermarket/obstaculos.json'
        write(self.graph, {'map':'supermarket'})
        write(self.geometry, {'obstacles':[]})

    def tearDown(self):
        self.temp.cleanup()

    def make_plan(self, mode='milp'):
        base = plans_dir(self.ws,self.map,self.seed)
        root = base if mode=='milp' else base/'fixed'
        if mode=='milp':
            asn = milp_results_dir(self.ws,self.map,self.seed)/'assignment_milp_one_color_per_round.json'
            data={'seed':self.seed,'world':self.map,'execution_authorized':False,
                  'status':'OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE',
                  'independent_exhaustive_verification':{'global_proxy_makespan_optimum_verified':True}}
        else:
            asn = root/'fixed_assignment.json'
            data={'seed':self.seed,'world':self.map,'execution_authorized':False,
                  'status':'FIXED_DETERMINISTIC_OFFLINE_NOT_EXECUTABLE'}
        write(asn,data)
        manifest={'scenario_sha256':digest(self.scene),'assignment_sha256':digest(asn),
                  'geometry_sha256':digest(self.geometry),'graph_sha256':digest(self.graph),
                  'seed':self.seed,'assignment_mode':mode,'max_ct':10000,
                  'max_cbs_time':110,'maxiter':450,'refinements':5,
                  'max_speed':2.0,'max_acceleration':2.0,'map':self.map}
        write(root/'inputs_manifest.json', manifest)
        cbs=write(root/'cbs_multigoal.json',
                  {'seed':self.seed,'assignment_mode':mode,
                   'status':'OFFLINE_CBS_ONLY_NOT_EXECUTABLE',
                   'inputs_sha256':{'scenario':digest(self.scene),'assignment':digest(asn)}})
        return root, manifest, cbs

    def test_offline_milp_manifest_complete(self):
        root, m, _=self.make_plan()
        with patch.object(run_full_offline,'validate_map'):
            with patch.object(sys,'argv',['run_full_offline.py','--seed',str(self.seed),'--map',self.map,
                                             '--workspace',str(self.ws),'--until','cbs']):
                self.assertEqual(run_full_offline.main(),0)

    def test_offline_fixed_manifest_complete(self):
        self.make_plan('fixed')
        with patch.object(run_full_offline,'validate_map'):
            with patch.object(sys,'argv',['run_full_offline.py','--seed',str(self.seed),'--map',self.map,
                                             '--assignment-mode','fixed','--workspace',str(self.ws),'--until','cbs']):
                self.assertEqual(run_full_offline.main(),0)

    def test_offline_rejects_legacy_missing_map_and_mode(self):
        root, m, _=self.make_plan()
        m.pop('map');m.pop('assignment_mode')
        write(root/'inputs_manifest.json',m)
        with patch.object(run_full_offline,'validate_map'):
            with patch.object(sys,'argv',['run_full_offline.py','--seed',str(self.seed),'--map',self.map,
                                             '--workspace',str(self.ws),'--until','cbs']):
                with self.assertRaisesRegex(ValueError,'BLOCKED'):
                    run_full_offline.main()

    def test_offline_does_not_guess_missing_max_ct(self):
        root, m, _=self.make_plan()
        m.pop('max_ct')
        write(root/'inputs_manifest.json',m)
        with patch.object(run_full_offline,'validate_map'):
            with patch.object(sys,'argv',['run_full_offline.py','--seed',str(self.seed),'--map',self.map,
                                             '--workspace',str(self.ws),'--until','cbs']):
                with self.assertRaisesRegex(ValueError,'max_ct'):
                    run_full_offline.main()

    def make_prepared_valid(self, mode='milp'):
        root,m,cbs=self.make_plan(mode)
        prepared=root/'prepared'
        pre={'status':'PASS_STATIC_PRECHECK_NOT_EXECUTABLE','errors':[],
             'sha256':{'cbs':digest(cbs),'scenario':digest(self.scene),'geometry':digest(self.geometry)},
             'stages':{}}
        for st in run_prepared_mission.STAGES:
            folder=root/'phase4'/st
            traj=folder/(st+'_MILP_TIMED_OFFLINE.npz')
            traj.parent.mkdir(parents=True,exist_ok=True)
            traj.write_bytes((st+'_test_npy_bytes').encode())
            clock=write(folder/(st+'_MILP_V3_clock.json'),{'input_sha256':digest(traj)})
            write(folder/(st+'_report.json'),{
                'status':'PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE',
                'refinements_accepted':5,'continuous_certificate':{'pass_certificate':True}})
            pre['stages'][st]={'trajectory':str(traj),'successive_clock':str(clock),'sha256':digest(traj)}
            write(prepared/(f'assignment_{st}_{mode.upper()}.json'),
                  {'seed':self.seed,'navigation_sha256':digest(traj)})
        write(prepared/'phase5_static_preflight.json', pre)
        write(prepared/f'scenario_for_original_D8_seed{self.seed}_{mode.upper()}.json',{'seed':self.seed})
        return root,m,cbs

    def test_prepared_valid_milp_and_fixed(self):
        for mode in ('milp','fixed'):
            with self.subTest(mode=mode):
                self.make_prepared_valid(mode)
                actual_scene, actual_prepared=run_prepared_mission.validate(self.ws,self.seed,mode,self.map)
                self.assertEqual(actual_scene,self.scene)
                self.assertTrue(actual_prepared.is_dir())

    def test_prepared_rejects_omitted_map_and_mode(self):
        root,m,_=self.make_prepared_valid()
        m.pop('map');m.pop('assignment_mode')
        write(root/'inputs_manifest.json',m)
        with self.assertRaisesRegex(ValueError,'Manifiesto'):
            run_prepared_mission.validate(self.ws,self.seed,'milp',self.map)

    def test_prepared_rejects_missing_cbs_mode(self):
        root,m,cbs=self.make_prepared_valid()
        x=json.loads(cbs.read_text());x.pop('assignment_mode')
        write(cbs,x)
        with self.assertRaisesRegex(ValueError,'CBS pertenece a otro modo'):
            run_prepared_mission.validate(self.ws,self.seed,'milp',self.map)

    def test_plot_rejects_old_plan_layout(self):
        scripts=self.ws/'scripts'
        old=scripts/'supermarket_seed_pipeline/plans/seed_100/prepared'
        old.mkdir(parents=True)
        with self.assertRaisesRegex(ValueError,'Ruta de plan no uniforme'):
            find_scenario(old,scripts)


if __name__=='__main__':
    unittest.main()
