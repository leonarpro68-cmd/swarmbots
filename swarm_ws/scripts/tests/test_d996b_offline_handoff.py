"""Unit tests for inert D9.9.6B handoff; no Gazebo/ROS dependencies."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d996b_preflight_readonly import evaluate_live, static_check, sha256
from d996b_plan_from_live_odom import check_inputs


class TestGate(unittest.TestCase):
    def setUp(self):
        self.refs={'summit0':[0.,0.], 'summit1':[4.,0.], 'summit2':[8.,0.]}
        self.frames={n:(n+'/odom',n+'/base_footprint') for n in self.refs}
        self.stamps={n:10.04 for n in self.refs}
        self.speeds={n:0. for n in self.refs}
        self.clocks=[9.0,9.4,10.0]

    def check(self, positions=None, **kwargs):
        return evaluate_live(positions or self.refs,self.stamps,self.speeds,self.frames,self.clocks,
                             self.refs,.05,.03,.25)

    def test_healthy_live_pose(self):
        errs,robots=self.check()
        self.assertEqual(errs,[])
        self.assertEqual(robots['summit0']['error_m'],0.)

    def test_mismatch_is_rejected(self):
        bad=dict(self.refs);bad['summit0']=[.125,0.]
        errs,_=self.check(bad)
        self.assertIn('START_POSE_MISMATCH_summit0',errs)

    def test_clock_stopped_is_rejected(self):
        self.clocks=[10.,10.,10.]
        errs,_=self.check()
        self.assertIn('CLOCK_NOT_ADVANCING',errs)

    def test_odom_stale_is_rejected(self):
        self.stamps['summit1']=8.0
        errs,_=self.check()
        self.assertIn('ODOM_STALE_summit1',errs)

    def test_frame_mismatch_is_rejected(self):
        self.frames['summit2']=('map','summit2/base_link')
        errs,_=self.check()
        self.assertIn('ODOM_FRAME_MISMATCH_summit2',errs)

    def test_robot_motion_is_rejected(self):
        self.speeds['summit1']=.15
        errs,_=self.check()
        self.assertIn('ROBOT_NOT_STOPPED_summit1',errs)

    def test_static_tamper_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'converted.npz'; f.write_bytes(b'original')
            p=Path(d)/'pf.json';p.write_text(json.dumps({'status':'PASS_OFFLINE_READ_ONLY','failures':[]}))
            ref={'schema':'swarm.d996b.fresh_start.offline.v1','mode':'live',
                 'status':'PASS_OFFLINE_FRESH_START_NOT_EXECUTABLE',
                 'offline_preflight_status':'PASS_OFFLINE_READ_ONLY',
                 'converted_start_xy_m':self.refs,
                 'artifacts':{'converted_trajectory':str(f),'offline_preflight':str(p)},
                 'sha256':{'converted_trajectory':sha256(f),'offline_preflight':sha256(p)}}
            self.assertFalse(static_check(ref))
            f.write_bytes(b'mutated')
            self.assertIn('FILE_CHANGED_CONVERTED_TRAJECTORY',static_check(ref))

    def test_historical_report_is_not_fresh_live(self):
        self.assertIn('NO_FRESH_LIVE_OFFLINE_PLAN',static_check({
            'schema':'swarm.d996b.fresh_start.offline.v1',
            'mode':'existing','status':'PASS_OFFLINE_REGRESSION_NOT_EXECUTABLE'}))


if __name__=='__main__':unittest.main()
