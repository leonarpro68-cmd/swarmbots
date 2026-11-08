"""Regresión: aislar los launches 7B heredados del flujo preparado multimapa."""
import unittest
from pathlib import Path

WS=Path(__file__).resolve().parents[1]
LAUNCH=WS/'src/swarm_worlds/launch'
LEGACY=WS/'archive/legacy_cbs_launches'
MOVED=('swarm_cbs_scenario.launch.py','swarm_cbs_mission.launch.py',
       'swarm_cbs_grasp_base.launch.py','swarm_scenario_prepare.py','swarm_mission_start.py')

class TestCBSLaunchCleanup(unittest.TestCase):
    def test_historic_launches_not_installed_from_source(self):
        for name in MOVED:
            with self.subTest(name=name):
                self.assertFalse((LAUNCH/name).exists())
                self.assertTrue((LEGACY/name).is_file())

    def test_keep_base_for_independent_follower_smoke(self):
        src=(LAUNCH/'swarm_cbs_base.launch.py').read_text()
        self.assertIn('swarm_path_follower',src)
        self.assertIn('sim_summit.launch.py',src)

    def test_main_launch_no_historic_dependency(self):
        src=(LAUNCH/'supermarket_full_prepared.launch.py').read_text()
        self.assertIn('sim_summit_supermarket.launch.py',src)
        self.assertNotIn('swarm_cbs_',src)
        self.assertNotIn('swarm_mission_start',src)
        self.assertNotIn('swarm_scenario_prepare',src)

    def test_archived_helpers_have_no_active_imports(self):
        for py in LAUNCH.glob('*.py'):
            if py.name=='swarm_cbs_base.launch.py':continue
            txt=py.read_text()
            for legacy in ('swarm_mission_start.py','swarm_scenario_prepare.py',
                           'swarm_cbs_mission.launch.py','swarm_cbs_scenario.launch.py',
                           'swarm_cbs_grasp_base.launch.py'):
                with self.subTest(source=py.name,ref=legacy):
                    self.assertNotIn(legacy,txt)

if __name__=='__main__':unittest.main()
