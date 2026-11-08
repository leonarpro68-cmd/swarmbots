"""Regresión de la limpieza del ejecutor CBS en vivo 6B–6E."""
import unittest
from pathlib import Path

WS=Path(__file__).resolve().parents[1]
PKG=WS/'src/swarm_behavior'
SRC=PKG/'swarm_behavior/swarm_planning'
LEG=WS/'archive/legacy_ros_mission'
FILES=('ros_full_mission.py','ros_pickup_delivery.py','ros_pickup_removal.py',
       'ros_pickup_executor.py','gazebo_removal.py')
TESTS=('test_full_mission.py','test_pickup_delivery.py',
       'test_pickup_executor.py','test_gazebo_removal.py')

class TestLegacyLiveMissionCleanup(unittest.TestCase):
    def test_archived_modules_and_tests(self):
        for name in FILES:
            with self.subTest(module=name):
                self.assertFalse((SRC/name).exists())
                self.assertTrue((LEG/'swarm_planning'/name).is_file())
        for name in TESTS:
            with self.subTest(test=name):
                self.assertFalse((PKG/'test'/name).exists())
                self.assertTrue((LEG/'test'/name).is_file())

    def test_remove_old_console_entrypoint_only(self):
        src=(PKG/'setup.py').read_text()
        self.assertNotIn('"swarm_mission =',src)
        for name in ('swarm_path_follower','swarm_path_follower_grasp',
                     'visual_grasp','grasp_manager','swarm_hybrid_ros_demo'):
            with self.subTest(entry=name):
                self.assertIn(f'"{name} =',src)

    def test_shared_helpers_and_offline_pipeline_retained(self):
        for name in ('ros_plan_helpers.py','ros_planning_probe.py','bridge.py',
                     'mission_core.py','cbs.py','geometry.py'):
            self.assertTrue((SRC/name).is_file(),name)
        hybrid=(SRC/'hybrid_refinement/ros_hybrid_demo.py').read_text()
        self.assertIn('from ..ros_plan_helpers import',hybrid)
        self.assertTrue((WS/'scripts/supermarket_seed_pipeline/run_full_offline.py').is_file())
        self.assertTrue((WS/'src/swarm_worlds/launch/supermarket_full_prepared.launch.py').is_file())
        self.assertTrue((WS/'src/swarm_worlds/launch/swarm_cbs_base.launch.py').is_file())

if __name__=='__main__':unittest.main()
