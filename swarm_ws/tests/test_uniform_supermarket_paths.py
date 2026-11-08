"""Regresión de rutas uniformes y manifiestos antiguos de supermarket."""
from pathlib import Path
import sys
import tempfile
import unittest

WS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS/'scripts'))
sys.path.insert(0,str(WS/'scripts/supermarket_seed_pipeline'))
from swarm_maps import (MAPS, scenario_file, generation_dir, milp_results_dir, plans_dir)
from run_full_offline import compatible_manifest

class UniformSupermarketPathsTest(unittest.TestCase):
    def test_seven_maps_have_same_folder_contract(self):
        for name in MAPS:
            with self.subTest(name=name):
                self.assertEqual(scenario_file(WS,name,321),WS/'config/scenarios'/name/'seed_321.json')
                self.assertEqual(generation_dir(WS,name,321),WS/'scripts/supermarket_escenarios/resultados'/name/'seed_321')
                self.assertEqual(milp_results_dir(WS,name,321),WS/'scripts/supermarket_milp/results'/name/'seed_321')
                self.assertEqual(plans_dir(WS,name,321),WS/'scripts/supermarket_seed_pipeline/plans'/name/'seed_321')

    def test_manifest_new_is_strict(self):
        expected={'seed':321,'map':'supermarket','assignment_mode':'milp','max_ct':10000}
        self.assertTrue(compatible_manifest(dict(expected),expected,'supermarket','milp'))
        self.assertFalse(compatible_manifest({**expected,'max_ct':6000},expected,'supermarket','milp'))
        self.assertFalse(compatible_manifest({**expected,'map':'parking_lot'},expected,'supermarket','milp'))
        self.assertFalse(compatible_manifest({k:v for k,v in expected.items() if k!='map'},expected,'rubber_factory','milp'))

    def test_historical_supermarket_manifests_remain_readable(self):
        milp={'seed':777,'map':'supermarket','assignment_mode':'milp','max_ct':6000}
        old={k:v for k,v in milp.items() if k not in ('map','assignment_mode')}
        self.assertTrue(compatible_manifest(old,milp,'supermarket','milp'))
        fixed={'seed':777,'map':'supermarket','assignment_mode':'fixed','max_ct':6000}
        old_fixed={k:v for k,v in fixed.items() if k!='map'}
        self.assertTrue(compatible_manifest(old_fixed,fixed,'supermarket','fixed'))

    def test_relative_legacy_symlink_preserves_internal_absolute_files(self):
        with tempfile.TemporaryDirectory() as td:
            plans=Path(td)/'plans';plans.mkdir()
            canonical=plans/'supermarket/seed_777/fixed/phase4/pickup1'
            canonical.mkdir(parents=True)
            data=canonical/'curve.npz';data.write_bytes(b'unchanged')
            (plans/'seed_777').symlink_to('supermarket/seed_777',target_is_directory=True)
            legacy=plans/'seed_777/fixed/phase4/pickup1/curve.npz'
            self.assertEqual(legacy.read_bytes(),data.read_bytes())
            self.assertEqual(legacy.resolve(),data.resolve())

if __name__=='__main__':unittest.main()
