import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d97d_evaluate import evaluate,load_csv
class TestD97D(unittest.TestCase):
    def test_pass(self):
        rows=[{'t_s':0.,'peak_err_0_m':0.,'peak_err_1_m':0.,'peak_err_2_m':0.,'separation_m':18.},
              {'t_s':1.,'peak_err_0_m':.01,'peak_err_1_m':.02,'peak_err_2_m':.03,'separation_m':17.}]
        follower={'status':'FINISHED_TRAJECTORY_NOT_MISSION','faults':[],'final_stop_confirmed':True,'end_signal_sent':True,'horizon_s':1.,'peak_tracking_error_m':{'summit0':.01,'summit1':.02,'summit2':.03},'min_measured_separation_m':17.,'final_observed_speed_m_s':dict.fromkeys(('summit0','summit1','summit2'),0.)}
        relay={'status':'NORMAL_END_STOP_CONFIRMED','faults':[],'normal_end':True}
        self.assertEqual(evaluate(follower,relay,rows)['status'],'PASS_RECORDED_FOLLOWER_AUDIT')
        self.assertEqual(evaluate(follower,None,rows)['status'],'REVIEW_REQUIRED')
        rows[1]['separation_m']=1.0
        self.assertIn('SEPARATION_BELOW_1_15M',evaluate(follower,relay,rows)['issues'])
class TestClockDuplicates(unittest.TestCase):
    def test_equal_timestamp_with_different_observation_allowed(self):
        import csv,tempfile
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'data.csv'
            with path.open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=['t_s','peak_err_0_m','peak_err_1_m','peak_err_2_m','separation_m'])
                w.writeheader()
                for t,e in [(0,0),(.02,.01),(.02,.015),(.04,.02)]:
                    w.writerow(dict(t_s=t,peak_err_0_m=e,peak_err_1_m=e,peak_err_2_m=e,separation_m=20))
            rows=load_csv(path)
            self.assertEqual(len(rows),4)
    def test_backward_timestamp_rejected(self):
        import csv,tempfile
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'data.csv'
            with path.open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=['t_s','peak_err_0_m','peak_err_1_m','peak_err_2_m','separation_m'])
                w.writeheader()
                for t in [0,.04,.02]:
                    w.writerow(dict(t_s=t,peak_err_0_m=.01,peak_err_1_m=.01,peak_err_2_m=.01,separation_m=20))
            with self.assertRaisesRegex(ValueError,'TIME_WENT_BACKWARDS'):
                load_csv(path)
    def test_all_duplicate_timestamps_not_valid_for_evaluation(self):
        rows=[dict(t_s=1.,peak_err_0_m=.0,peak_err_1_m=.0,peak_err_2_m=.0,separation_m=20.) for _ in range(3)]
        with self.assertRaisesRegex(ValueError,'NO_TIME_PROGRESS'):
            evaluate({},None,rows)

if __name__=='__main__':unittest.main()
