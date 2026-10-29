import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98b_grasp_preflight import evaluate,parse_gz_pose_info
class TestPreflight(unittest.TestCase):
    def setUp(self):
        self.names=['trash_a','trash_b','trash_c']
        self.poses={'summit0':[0,0,0,0],'summit1':[4,0,0,0],'summit2':[8,0,0,0]}
        self.objects={'trash_a':[.5,0],'trash_b':[4.5,0],'trash_c':[8.5,0]}
    def test_ideal(self):
        self.assertEqual(evaluate(self.names,self.poses,self.objects)['status'],'READY_FOR_MANUAL_GRASP_TEST_ONLY')
    def test_not_stopped(self):
        self.poses['summit0'][3]=.2
        self.assertEqual(evaluate(self.names,self.poses,self.objects)['status'],'BLOCKED')
    def test_wrong_nearest(self):
        self.objects['trash_other']=[.46,0]
        self.assertEqual(evaluate(self.names,self.poses,self.objects)['status'],'BLOCKED')
    def test_distance(self):
        self.objects['trash_a']=[1.5,0]
        self.assertEqual(evaluate(self.names,self.poses,self.objects)['status'],'BLOCKED')
    def test_parser(self):
        self.assertEqual(parse_gz_pose_info('name: "trash_a"\nposition {\nx: 1.0\ny: 2.0\nz: 0\n}'),{'trash_a':[1.,2.]})
    def test_missing(self):
        del self.poses['summit1']
        self.assertEqual(evaluate(self.names,self.poses,self.objects)['status'],'BLOCKED')
if __name__=='__main__':unittest.main()
