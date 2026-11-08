"""Unit tests for optional fast waiting profile, no ROS/Gazebo dependency."""
import importlib.util
import sys
import unittest
from pathlib import Path

WS=Path(__file__).resolve().parents[1]
PIPE=WS/'scripts/supermarket_seed_pipeline'
sys.path.insert(0,str(PIPE))
from timing_profiles import get_profile

class TestTimingProfiles(unittest.TestCase):
    def test_profile_values(self):
        normal=get_profile('normal')
        fast=get_profile('fast')
        self.assertEqual(tuple(vars(normal).values()),(8.0,15.0,8.0,3.0))
        self.assertEqual(tuple(vars(fast).values()),(3.0,5.0,4.0,1.0))
        self.assertGreaterEqual(fast.start_delay,5.0)  # Coordinator safety gate

    def test_reject_unknown(self):
        for bad in ('', 'speed', 'FAST', None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                get_profile(bad)

    def test_nav_preserves_safety_controls(self):
        from run_auto_experiment_seed import build_nav
        nav={'trajectory':'trajectory.npz','successive_clock':'clock.json','sha256':'sha'}
        for name in ('normal','fast'):
            with self.subTest(profile=name):
                cmds=dict(build_nav('pickup1',nav,Path('/tmp/run'),get_profile(name)))
                args=cmds['nav']
                self.assertEqual(args[args.index('--start-delay')+1],str(get_profile(name).start_delay))
                self.assertEqual(args[args.index('--observe-seconds')+1],str(get_profile(name).observe_seconds))
                self.assertIn('--confirm-runtime-v2',args)
                self.assertIn('--expected-sha256',args)
                self.assertIn('--nav-speed-limit',args)
                self.assertIn('--nav-accel-limit',args)

    def test_preflight_uses_runtime_value_and_keeps_checks(self):
        source=(PIPE/'ros_preflight.py').read_text()
        self.assertIn('str(observe_seconds)',source)
        self.assertIn('PASS_SUPPORT_GRAPH_NOT_EXECUTION_AUTHORIZATION',source)
        self.assertIn('PASS_READ_ONLY_NOT_EXECUTION_AUTHORIZATION',source)
        self.assertIn('timeout=45',source)
        self.assertIn('time.sleep(3)',source)  # timeout retry, not a fixed success wait

    def test_main_pipeline_propagates_profile(self):
        runner=(PIPE/'run_prepared_mission.py').read_text()
        launch=(WS/'src/swarm_worlds/launch/supermarket_full_prepared.launch.py').read_text()
        self.assertIn('timing.preflight_observe_seconds',runner)
        self.assertIn("'--timing-profile',a.timing_profile",runner)
        self.assertIn("DeclareLaunchArgument('timing_profile',default_value='normal'",launch)
        self.assertIn("'--timing-profile',timing_profile",launch)

    def test_critical_watchdogs_untouched(self):
        coordinator=(WS/'scripts/d9913_stage_coordinator.py').read_text()
        self.assertIn('START_DELAY_OUT_OF_RANGE',coordinator)
        self.assertIn('ARM_BARRIER_TIMEOUT',coordinator)
        self.assertIn('FOLLOWER_REJECTED_START',coordinator)
        self.assertIn('SIM_CLOCK_JUMPED_BACKWARD',coordinator)
        self.assertIn('CLOCK_NOT_ADVANCING',coordinator)
        self.assertIn('ODOM_STALE',coordinator)
        self.assertIn('CMD_VEL_OWNER_MISMATCH',coordinator)

if __name__=='__main__':unittest.main()
