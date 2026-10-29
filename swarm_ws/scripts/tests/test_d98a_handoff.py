import importlib.util, pathlib, tempfile, unittest, json, hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('h98',ROOT/'d98a_mission_handoff.py')
h98=importlib.util.module_from_spec(spec);spec.loader.exec_module(h98)
class TestHandoff(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
  self.run=pathlib.Path(self.temp.name);self.stage='pickup1';self.relay=self.run/'relay.json'
  def write(rel,obj):
   p=self.run/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(obj));return p
  write('d94_conversion/pickup1_converted_NON_EXECUTABLE.npz',{'dummy':'test'})
  sha=h98.digest(self.run/'d94_conversion/pickup1_converted_NON_EXECUTABLE.npz')
  write('d96c2b_report.json',{'stage':'pickup1','status':'PASS_READ_ONLY_LIVE_REPLAN','converted_controls_sha256':sha,'scenario_sha256':'a'})
  write('d97c/pickup1_d97c_report.json',{'status':'FINISHED_TRAJECTORY_NOT_MISSION','faults':[],'final_stop_confirmed':True,'end_signal_sent':True,'min_measured_separation_m':3.,'final_observed_speed_m_s':{f'summit{i}':0. for i in range(3)}})
  write('relay.json',{'status':'NORMAL_END_STOP_CONFIRMED','faults':[],'normal_end':True})
  write('live/pickup1_live_report.json',{'stages':{'pickup1':{'certified':True,'hybrid_v2':{'certified':True},'assigned_names':['a','b','c']}}})
 def test_pass_is_readonly(self):
  x=h98.evaluate(self.run,self.stage,self.relay)
  self.assertEqual(x['status'],'READY_FOR_PHYSICAL_CONFIRMATION_ONLY')
  self.assertFalse(x['physical_grasp_confirmed'])
 def test_missing_relay_blocks(self):
  self.assertEqual(h98.evaluate(self.run,self.stage,self.run/'missing.json')['status'],'BLOCKED_HANDOFF')
 def test_bad_stop_blocks(self):
  p=self.run/'d97c/pickup1_d97c_report.json';a=json.loads(p.read_text());a['final_stop_confirmed']=False;p.write_text(json.dumps(a))
  self.assertIn('FOLLOWER_STOP_NOT_CONFIRMED',h98.evaluate(self.run,self.stage,self.relay)['reasons'])
 def test_duplicate_objects_blocks(self):
  p=self.run/'live/pickup1_live_report.json';a=json.loads(p.read_text());a['stages']['pickup1']['assigned_names']=['a','a','b'];p.write_text(json.dumps(a))
  self.assertIn('D8_OBJECT_IDENTIFIERS_INVALID',h98.evaluate(self.run,self.stage,self.relay)['reasons'])
 def test_hash_change_blocks(self):
  (self.run/'d94_conversion/pickup1_converted_NON_EXECUTABLE.npz').write_text('other')
  self.assertIn('CONVERTED_SHA_MISMATCH',h98.evaluate(self.run,self.stage,self.relay)['reasons'])
 def test_later_stage_requires_ledger(self):
  self.assertIn('D6_LEDGER_REQUIRED_FOR_LATER_STAGE',h98.evaluate(self.run,'delivery1',self.relay)['reasons'])
 def test_wrong_stage_ledger_blocks(self):
  p=self.run/'ledger.json';p.write_text(json.dumps({'version':'swarmbots.d6.v1','completed':['pickup1'],'scenario_sha256':'a'}))
  self.assertIn('D6_STAGE_ORDER',h98.evaluate(self.run,'pickup1',self.relay,p)['reasons'])
if __name__=='__main__':unittest.main()
