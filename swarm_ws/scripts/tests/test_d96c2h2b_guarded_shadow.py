import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96c2h2b_guarded_shadow import (
    ROBOTS, CMD_TOPICS, ShadowLifecycle, GraphScanner, angle_delta, check_continuous,
)

NOW = 100.0

def healthy():
    starts={name:{'x_m':float(i*7),'y_m':float(i*5)} for i,name in enumerate(ROBOTS)}
    init={name:0.1*i for i,name in enumerate(ROBOTS)}
    odoms={}
    for i,name in enumerate(ROBOTS):
        odoms[name]={'x_m':starts[name]['x_m'], 'y_m':starts[name]['y_m'],
                'vx_m_s':0., 'vy_m_s':0.,'yaw_rad':init[name],
                'quaternion_norm':1.,'frame_id':f'{name}/odom',
                'child_frame_id':f'{name}/base_footprint',
                'stamp_sim_s':200.,'last_received_wall':NOW-.03}
    snapshot={'odoms':odoms,
              'clock':{'stamp_s':200.,'last_received_wall':NOW-.02,'last_progress_wall':NOW-.02,'rewound':False},
              'graph':{'last_checked_wall':NOW-.1,'query_count':6,
                       'publishers':{topic:[] for topic in CMD_TOPICS}}}
    return snapshot,starts,init

class ContinuousChecks(unittest.TestCase):
    def test_good(self):
        self.assertEqual(check_continuous(*healthy(),NOW),[])
    def test_clock_stall(self):
        s,x,y=healthy();s['clock']['last_received_wall']=NOW-1.
        self.assertIn('CLOCK_WATCHDOG',check_continuous(s,x,y,NOW))
    def test_clock_rewind(self):
        s,x,y=healthy();s['clock']['rewound']=True
        self.assertIn('CLOCK_REWIND',check_continuous(s,x,y,NOW))
    def test_clock_paused_but_messages_arrive(self):
        s,x,y=healthy();s['clock']['last_progress_wall']=NOW-2.
        self.assertIn('CLOCK_NOT_PROGRESSING',check_continuous(s,x,y,NOW))
    def test_clock_missing(self):
        s,x,y=healthy();s['clock']['stamp_s']=None
        self.assertIn('CLOCK_WATCHDOG',check_continuous(s,x,y,NOW))
    def test_clock_future(self):
        s,x,y=healthy();s['clock']['last_received_wall']=NOW+.01
        self.assertIn('CLOCK_WATCHDOG',check_continuous(s,x,y,NOW))
    def test_odom_lost(self):
        s,x,y=healthy();s['odoms'].pop('summit2')
        self.assertIn('MISSING_ODOMETRY',check_continuous(s,x,y,NOW))
    def test_odom_timeout(self):
        s,x,y=healthy();s['odoms']['summit0']['last_received_wall']=NOW-1.
        self.assertIn('summit0:ODOM_WATCHDOG',check_continuous(s,x,y,NOW))
    def test_odom_clock_skew(self):
        s,x,y=healthy();s['odoms']['summit2']['stamp_sim_s']=199.1
        self.assertIn('summit2:ODOM_WATCHDOG',check_continuous(s,x,y,NOW))
    def test_frame_mismatch(self):
        s,x,y=healthy();s['odoms']['summit1']['frame_id']='map'
        self.assertIn('summit1:FRAME',check_continuous(s,x,y,NOW))
    def test_child_frame_mismatch(self):
        s,x,y=healthy();s['odoms']['summit1']['child_frame_id']='base_link'
        self.assertIn('summit1:FRAME',check_continuous(s,x,y,NOW))
    def test_quaternion(self):
        s,x,y=healthy();s['odoms']['summit1']['quaternion_norm']=0.8
        self.assertIn('summit1:QUATERNION',check_continuous(s,x,y,NOW))
    def test_invalid_number(self):
        s,x,y=healthy();s['odoms']['summit0']['x_m']=float('nan')
        self.assertIn('summit0:NONFINITE_ODOM',check_continuous(s,x,y,NOW))
    def test_held_position_drift(self):
        s,x,y=healthy();s['odoms']['summit2']['x_m']+=.06
        self.assertIn('summit2:LEFT_START',check_continuous(s,x,y,NOW))
    def test_speed(self):
        s,x,y=healthy();s['odoms']['summit1']['vx_m_s']=.05
        self.assertIn('summit1:MOVING',check_continuous(s,x,y,NOW))
    def test_yaw(self):
        s,x,y=healthy();s['odoms']['summit0']['yaw_rad']+=math.radians(4)
        self.assertIn('summit0:YAW_CHANGED',check_continuous(s,x,y,NOW))
    def test_wrap_yaw(self):
        self.assertLess(abs(angle_delta(-math.pi+.01,math.pi-.01)),.03)
    def test_separation(self):
        s,x,y=healthy()
        s['odoms']['summit1']['x_m']=s['odoms']['summit0']['x_m']+.5
        s['odoms']['summit1']['y_m']=s['odoms']['summit0']['y_m']
        self.assertIn('summit0-summit1:SEPARATION',check_continuous(s,x,y,NOW))
    def test_other_pub(self):
        s,x,y=healthy();s['graph']['publishers']['/summit0/cmd_vel']=['/d8_follower']
        self.assertIn('OTHER_VELOCITY_PUBLISHER',check_continuous(s,x,y,NOW))
    def test_graph_stall(self):
        s,x,y=healthy();s['graph']['last_checked_wall']=NOW-3.
        self.assertIn('GRAPH_WATCHDOG',check_continuous(s,x,y,NOW))
    def test_graph_unobserved(self):
        s,x,y=healthy();s['graph']['query_count']=0
        self.assertIn('GRAPH_NOT_OBSERVED',check_continuous(s,x,y,NOW))
    def test_graph_partial(self):
        s,x,y=healthy();s['graph']['publishers'].pop('/summit2/cmd_vel')
        self.assertIn('GRAPH_UNAVAILABLE',check_continuous(s,x,y,NOW))
    def test_graph_absent(self):
        s,x,y=healthy();s['graph']['publishers']={}
        self.assertIn('GRAPH_UNAVAILABLE',check_continuous(s,x,y,NOW))
    def test_threshold_exact(self):
        s,x,y=healthy();s['odoms']['summit1']['vx_m_s']=.03
        s['odoms']['summit1']['x_m']+=.05
        self.assertEqual(check_continuous(s,x,y,NOW),[])

class GraphScannerTests(unittest.TestCase):
    class Info:
        node_namespace = '/demo'
        node_name = 'competitor'

    class FakeNode:
        def __init__(self, publishers=(), fail_topic=None):
            self.calls=[]
            self.publishers=publishers
            self.fail_topic=fail_topic
        def get_publishers_info_by_topic(self, topic):
            self.calls.append(topic)
            if self.fail_topic == topic:
                raise RuntimeError('DDS graph query failed')
            return [GraphScannerTests.Info()] if topic in self.publishers else []

    def test_sweep_completion_timestamp_and_all_topics(self):
        n=self.FakeNode()
        sc=GraphScanner(n)
        sc.scan_once()
        s=sc.snapshot()
        self.assertEqual(s['query_count'],1)
        self.assertEqual(set(n.calls),set(CMD_TOPICS))
        self.assertIsNotNone(s['last_checked_wall'])
        self.assertGreaterEqual(s['last_checked_wall'],s['last_scan_duration_wall_s'])
        self.assertEqual(s['publishers'],{t:[] for t in CMD_TOPICS})
    def test_publishers_retained_across_scans(self):
        n=self.FakeNode(['/summit1/cmd_vel'])
        sc=GraphScanner(n)
        sc.scan_once()
        n.publishers=()
        sc.scan_once()
        self.assertEqual(sc.snapshot()['publishers']['/summit1/cmd_vel'],['/demo/competitor'])
    def test_query_exception_is_not_success(self):
        n=self.FakeNode(fail_topic=CMD_TOPICS[3])
        sc=GraphScanner(n)
        sc.scan_once()
        s=sc.snapshot()
        self.assertEqual(s['query_count'],0)
        self.assertIsNone(s['last_checked_wall'])
        self.assertIn('RuntimeError',s['scan_error'])
    def test_graph_timestamp_after_successful_sweep(self):
        sc=GraphScanner(self.FakeNode())
        sc.scan_once()
        s=sc.snapshot()
        self.assertGreaterEqual(s['last_checked_wall'],sc.checked_wall)
        self.assertIsNone(s['scan_started_wall'])
    def test_snapshot_does_not_leak_mutable_lists(self):
        sc=GraphScanner(self.FakeNode(['/cmd_vel']))
        sc.scan_once()
        s=sc.snapshot();s['publishers']['/cmd_vel'].clear()
        self.assertTrue(sc.snapshot()['publishers']['/cmd_vel'])
    def test_failed_scanner_stays_fail_closed(self):
        s,x,y=healthy();s['graph']['scan_error']='timeout'
        self.assertIn('GRAPH_QUERY_ERROR',check_continuous(s,x,y,NOW))
    def test_scanner_dead_rejected(self):
        s,x,y=healthy();s['graph']['thread_alive']=False
        self.assertIn('GRAPH_SCANNER_STOPPED',check_continuous(s,x,y,NOW))
    def test_active_or_very_slow_scan_rejected(self):
        s,x,y=healthy();s['graph']['scan_started_wall']=NOW-2.5
        self.assertIn('GRAPH_SCAN_SLOW',check_continuous(s,x,y,NOW))
        s,x,y=healthy();s['graph']['max_scan_duration_wall_s']=2.1
        self.assertIn('GRAPH_SCAN_SLOW',check_continuous(s,x,y,NOW))

class LifecycleTests(unittest.TestCase):
    def test_valid_path(self):
        m=ShadowLifecycle();m.arm([]);m.begin();m.update([]);m.stop()
        self.assertEqual(m.state,'STOPPED')
        self.assertEqual([t['to'] for t in m.events],['DISARMED','ARMED','RUNNING_SHADOW','STOPPED'])
    def test_arm_fail_latched(self):
        m=ShadowLifecycle();m.arm(['NO_ODOM']);m.stop()
        self.assertEqual(m.state,'FAULT')
        self.assertEqual(m.faults,['NO_ODOM'])
    def test_runtime_fault_latched(self):
        m=ShadowLifecycle();m.arm([]);m.begin();m.update(['CLOCK_WATCHDOG']);m.stop()
        self.assertEqual(m.state,'FAULT')
        self.assertIn('CLOCK_WATCHDOG',m.faults)
    def test_no_arming_after_fault(self):
        m=ShadowLifecycle();m.arm(['NO_CLOCK'])
        with self.assertRaises(RuntimeError):m.arm([])
    def test_cannot_run_unarmed(self):
        with self.assertRaises(RuntimeError):ShadowLifecycle().begin()
    def test_fault_cannot_auto_clear(self):
        m=ShadowLifecycle();m.arm([]);m.begin();m.update(['MOVING']);m.update([])
        self.assertEqual(m.state,'FAULT')
    def test_stop_from_armed(self):
        m=ShadowLifecycle();m.arm([]);m.stop()
        self.assertEqual(m.state,'STOPPED')

if __name__=='__main__':unittest.main()
