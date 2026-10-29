import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import d96c2g_dynamic_frames as m


def pose_block(name='summit0', t=2., x=1., y=2., yaw=0.):
    return f'''pose {{
  header {{ stamp {{ sec: {int(t)} nsec: {round((t-int(t))*1e9)} }} }}
  name: "{name}"
  position {{ x: {x} y: {y} z: 0.009 }}
  orientation {{ x: 0 y: 0 z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}
}}\n'''


def sample_data(offset=0., static=False, yaw_offset=0.):
    p=m.GazeboStreamParser()
    o={r:[] for r in m.ROBOTS}
    for k in range(12):
        t=2.+.2*k
        for r in m.ROBOTS:
            rid=int(r[-1]);x=rid*4.+(0. if static else .11*k)
            y=rid*3.
            p.feed(pose_block(r,t,x+offset,y,yaw_offset))
            for step in (0, .1):
                if k==11 and step==.1:continue
                o[r].append({'stamp_s':t+step,'xyz':[x+(0. if static else .55*step),y,0.],
                             'yaw_rad':0.,'frame_id':r+'/odom',
                             'child_frame_id':r+'/base_footprint'})
    return p,o,[2.,3.,4.]


class DynamicFramesTests(unittest.TestCase):
    def test_long_clock_has_no_truncation(self):
        clock=m.ClockAccumulator()
        for i in range(60001):clock.append(i*.002)
        self.assertEqual(len(clock),60001)
        valid,bounds,details=m.summarize_clock(clock)
        self.assertTrue(valid)
        self.assertEqual(bounds,[0.0,120.0])
        self.assertEqual(details['reset_count'],0)

    def test_clock_reset_after_40000_detected(self):
        clock=m.ClockAccumulator()
        for i in range(45000):clock.append(i*.002)
        clock.append(0.0)
        valid,_,details=m.summarize_clock(clock)
        self.assertFalse(valid)
        self.assertEqual(details['reset_count'],1)

    def test_bad_clock_stamp_rejected(self):
        clock=m.ClockAccumulator()
        for i in [1.,2.,float('nan')]:clock.append(i)
        valid,_,details=m.summarize_clock(clock)
        self.assertFalse(valid)
        self.assertEqual(details['invalid_stamp_count'],1)


    def test_pose_timestamp_from_individual_header(self):
        p=m.GazeboStreamParser();p.feed(pose_block(t=3.125))
        self.assertAlmostEqual(p.samples['summit0'][0]['stamp_s'],3.125)
        self.assertEqual(p.samples['summit0'][0]['stamp_source'],'pose.header')
    def test_pose_vector_header_fallback(self):
        p=m.GazeboStreamParser()
        p.feed('header { stamp { sec: 8 nsec: 250000000 } }\n')
        block=pose_block().replace('  header { stamp { sec: 2 nsec: 0 } }\n','')
        p.feed(block)
        self.assertAlmostEqual(p.samples['summit0'][0]['stamp_s'],8.25)
    def test_pose_header_precedes_vector_header(self):
        p=m.GazeboStreamParser();p.feed('header { stamp { sec: 9 } }\n')
        p.feed(pose_block(t=3.5))
        self.assertAlmostEqual(p.samples['summit0'][0]['stamp_s'],3.5)
    def test_missing_stamps_rejected(self):
        p=m.GazeboStreamParser()
        p.feed(pose_block().replace('  header { stamp { sec: 2 nsec: 0 } }\n',''))
        self.assertEqual(p.unstamped,1)
        report,rows=m.analyze(p,{r:[] for r in m.ROBOTS},[1,2])
        self.assertEqual(report['status'],'REJECT_GAZEBO_POSE_TIMESTAMPS_MISSING')
    def test_interpolate_heading_wrap(self):
        a={'stamp_s':1,'xyz':[0,0,0],'yaw_rad':math.radians(179),
           'frame_id':'summit0/odom','child_frame_id':'summit0/base_footprint'}
        b=dict(a,stamp_s=1.02,xyz=[.02,0,0],yaw_rad=math.radians(-179))
        z=m.interpolate_odom([a,b],1.01,.03)
        self.assertAlmostEqual(abs(math.degrees(z['yaw_rad'])),180,places=6)
        self.assertAlmostEqual(z['xyz'][0],.01)
    def test_skew_rejected(self):
        z=m.interpolate_odom([{'stamp_s':1,'xyz':[0,0,0],'yaw_rad':0},
                               {'stamp_s':1.5,'xyz':[0,0,0],'yaw_rad':0}],1.25,.04)
        self.assertIsNone(z)
    def test_good_dynamic_samples(self):
        p,odom,clock=sample_data()
        report,rows=m.analyze(p,odom,clock)
        self.assertEqual(report['status'],'DYNAMIC_WORLD_ODOM_CONSISTENT_OBSERVED_NOT_TF_PROVEN')
        self.assertEqual(len(rows),36)
        self.assertFalse(report['execution_authorized'])
        self.assertFalse(report['coordinate_frame_equivalence_proven'])
        self.assertEqual(report['commands_sent'],0)
    def test_static_always_rejected(self):
        p,odom,clock=sample_data(static=True)
        report,_=m.analyze(p,odom,clock)
        self.assertEqual(report['status'],'REJECT_INSUFFICIENT_DYNAMIC_EVIDENCE')
    def test_mismatch_during_motion(self):
        p,odom,clock=sample_data(offset=.25)
        report,_=m.analyze(p,odom,clock)
        self.assertEqual(report['status'],'REJECT_DYNAMIC_FRAME_COMPARISON')
        self.assertTrue(report['robots']['summit0']['consistent_rigid_transform_observed'])
        self.assertFalse(report['robots']['summit0']['near_identity_observed'])
    def test_clock_reset_rejected(self):
        p,odom,clock=sample_data()
        report,_=m.analyze(p,odom,[2,4,1])
        self.assertEqual(report['status'],'REJECT_CLOCK_NOT_PROGRESSING_OR_RESET')
    def test_expected_frame_enforced(self):
        p,odom,clock=sample_data()
        odom['summit1'][0]['frame_id']='map'
        report,_=m.analyze(p,odom,clock)
        self.assertNotEqual(report['status'],'DYNAMIC_WORLD_ODOM_CONSISTENT_OBSERVED_NOT_TF_PROVEN')
    def test_pose_parser_nested(self):
        s='''pose {\nname: "summit0"\nheader {\nstamp {\nsec: 10\nnsec: 0\n}\n}\nposition {\nx: 1\ny: 2\nz: 0\n}\norientation {\nz: 0\nw: 1\n}\n}\n'''
        p=m.GazeboStreamParser()
        for line in s.splitlines(keepends=True):p.feed(line)
        self.assertEqual(len(p.samples['summit0']),1)
        self.assertAlmostEqual(p.samples['summit0'][0]['stamp_s'],10.)


if __name__=='__main__':unittest.main()
