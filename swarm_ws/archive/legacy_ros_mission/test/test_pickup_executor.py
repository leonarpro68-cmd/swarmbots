import pathlib
import sys
import unittest
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_planning.ros_pickup_executor import padded_paths, Barrier, barrier_complete
from swarm_planning.ros_plan_helpers import RobotSample, AGENTS

class TestPickupExecutor(unittest.TestCase):
    def test_padding(self):
        p = padded_paths((((1,1),(1,2)), ((2,2),), ((3,3),(3,4),(3,5))))
        self.assertEqual(tuple(map(len,p)), (3,3,3))
        self.assertEqual(p[0][-1], (1,2))
    def test_empty(self):
        with self.assertRaises(ValueError): padded_paths(((),))
    def test_barrier(self):
        samples={a:RobotSample(i*3.,0., 10.) for i,a in enumerate(AGENTS)}
        targets={a:(i*3.,0.) for i,a in enumerate(AGENTS)}
        b=Barrier('ALIGN',targets,10.,{a:None for a in AGENTS})
        speeds={a:0. for a in AGENTS}
        self.assertFalse(barrier_complete(b,samples,speeds,10.))
        self.assertTrue(barrier_complete(b,samples,speeds,10.7))
    def test_barrier_needs_all(self):
        samples={a:RobotSample(i*3.,0.,10.) for i,a in enumerate(AGENTS)}
        targets={a:(i*3.,0.) for i,a in enumerate(AGENTS)}
        b=Barrier('STEP',targets,10.,{a:None for a in AGENTS})
        speeds={a:0. for a in AGENTS}
        speeds['summit1']=.2
        self.assertFalse(barrier_complete(b,samples,speeds,10.))
        self.assertFalse(barrier_complete(b,samples,speeds,11.))
        speeds['summit1']=0.
        self.assertFalse(barrier_complete(b,samples,speeds,11.1))
        self.assertTrue(barrier_complete(b,samples,speeds,11.8))

if __name__=='__main__': unittest.main()
