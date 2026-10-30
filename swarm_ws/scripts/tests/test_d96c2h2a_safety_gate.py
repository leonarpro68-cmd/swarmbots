import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96c2h2a_safety_gate import assess_live, validate_manifest, CMD_TOPICS


def valid_manifest():
    return {'status':'PASS_READ_ONLY','ros_executable':False,'publisher_enabled':False,
            'checks':{'polynomial_and_joins':True,'separation':True,'d94_approximation':True},
            'separation':{'conservative_lower_m':1.25},
            'limits':{'vmax_m_s':1.2,'amax_m_s2':1.0,'min_interrobot_m':1.15},
            'robots':{r:{'checks_passed':True,'speed_upper_m_s':1.15,'accel_upper_m_s2':0.95}
                      for r in ('summit0','summit1','summit2')}}


def valid_live():
    starts={r:{'x_m':float(i)*5,'y_m':float(i)*2} for i,r in enumerate(('summit0','summit1','summit2'))}
    odoms={r:{'x_m':starts[r]['x_m'],'y_m':starts[r]['y_m'],
              'vx_m_s':0.,'vy_m_s':0.,'quaternion_norm':1.,
              'age_wall_s':0.01,'stamp_sim_s':99.98,'frame_id':f'{r}/odom','child_frame_id':f'{r}/base_footprint'}
              for r in starts}
    observed={'clock':{'current_sim_s':100.,'progressing':True,'rewound':False},
              'graph_queries':4,'odoms':odoms,'command_publishers':{t:[] for t in CMD_TOPICS}}
    return starts,observed


class SafetyGateTests(unittest.TestCase):
    def test_pass_inert(self):
        starts,observed=valid_live()
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertEqual(reasons,[])
        self.assertEqual(validate_manifest(valid_manifest()),[])
    def test_stale(self):
        starts,observed=valid_live()
        reasons,_=assess_live({'snapshot_time_s':-100.},starts,observed)
        self.assertIn('CLOCK_OR_CAPTURE_STALE',reasons)
    def test_clock_rewind(self):
        starts,observed=valid_live();observed['clock']['rewound']=True
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('CLOCK_OR_CAPTURE_STALE',reasons)
    def test_other_controller(self):
        starts,observed=valid_live();observed['command_publishers']['/summit0/cmd_vel']=['/d8_executor']
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('OTHER_CMD_VEL_PUBLISHER_PRESENT',reasons)
    def test_unobserved_graph(self):
        starts,observed=valid_live();observed['graph_queries']=0
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('ROS_GRAPH_NOT_OBSERVED',reasons)
    def test_missing_odom(self):
        starts,observed=valid_live();del observed['odoms']['summit2']
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('MISSING_ODOMETRY',reasons)
    def test_shifted_robot(self):
        starts,observed=valid_live();observed['odoms']['summit0']['x_m']+=0.1
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('summit0:INVALID_INITIAL_ODOMETRY',reasons)
    def test_bad_frame(self):
        starts,observed=valid_live();observed['odoms']['summit0']['frame_id']='map'
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('summit0:INVALID_INITIAL_ODOMETRY',reasons)
    def test_live_robot_collision(self):
        starts,observed=valid_live()
        observed['odoms']['summit1']['x_m']=observed['odoms']['summit0']['x_m']
        observed['odoms']['summit1']['y_m']=observed['odoms']['summit0']['y_m']
        starts['summit1']=dict(starts['summit0'])
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('summit0-summit1:LIVE_SEPARATION',reasons)
    def test_old_odom_stamp(self):
        starts,observed=valid_live();observed['odoms']['summit1']['stamp_sim_s']=99.
        reasons,_=assess_live({'snapshot_time_s':95.},starts,observed)
        self.assertIn('summit1:INVALID_INITIAL_ODOMETRY',reasons)
    def test_uncertified_manifest(self):
        m=valid_manifest();m['status']='REJECT'
        self.assertIn('MANIFEST_NOT_PASSED',validate_manifest(m))
    def test_low_plan_sep(self):
        m=valid_manifest();m['separation']['conservative_lower_m']=1.14
        self.assertIn('PLAN_MIN_SEPARATION',validate_manifest(m))
    def test_bad_plan_speed(self):
        m=valid_manifest();m['robots']['summit2']['speed_upper_m_s']=1.3
        self.assertIn('summit2:PLAN_ENVELOPE',validate_manifest(m))
    def test_unexpected_ros_exec(self):
        m=valid_manifest();m['ros_executable']=True
        self.assertIn('MANIFEST_EXECUTABLE',validate_manifest(m))

if __name__=='__main__':unittest.main()
