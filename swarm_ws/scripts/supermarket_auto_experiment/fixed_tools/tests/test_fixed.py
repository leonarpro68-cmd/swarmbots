import importlib.util, pathlib, tempfile, unittest, shutil, json
ROOT=pathlib.Path(__file__).resolve().parents[1]

def load(name,p):
 spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

class FixedMission(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.prep=load('prepare_fixed',ROOT/'prepare_fixed.py')
  cls.auto=load('run_auto_experiment',(ROOT/'run_auto_experiment.py' if (ROOT/'run_auto_experiment.py').is_file() else ROOT.parent/'run_auto_experiment.py'))
  cls.tmp=tempfile.TemporaryDirectory();cls.ws=pathlib.Path(cls.tmp.name)/'ws';cls.out=pathlib.Path(cls.tmp.name)/'out'
  d=cls.ws/'scripts/supermarket_v3/data';d.mkdir(parents=True);(cls.ws/'config/supermarket').mkdir(parents=True)
  shutil.copy(ROOT/'fixtures/seed_100.json',cls.ws/'config/supermarket/seed_100.json')
  for f in (ROOT/'fixtures').glob('*supermarket_seed100*'):shutil.copy(f,d/f.name)
  cls.rep=cls.prep.run(cls.ws,cls.out,ROOT/'supermarket_obstaculos.json')
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def test_all_gates(self):
  self.assertEqual(self.rep['status'],'PASS_STATIC_PRECHECK_NOT_EXECUTABLE');self.assertEqual(len(self.rep['d8_tasks']),12)
 def test_contract(self):
  r,t,s=self.auto.check_contract(self.out,'fixed');self.assertEqual(len(r['stages']),4)
 def test_assignment(self):
  a=json.loads((self.out/'assignment_delivery1_FIXED.json').read_text());self.assertEqual(a['robots']['summit0']['trash'],'trash_A2_azul');self.assertEqual(a['robots']['summit0']['target'],'deposit_azul')
 def test_delivery_args_never_deposit_as_trash(self):
  _,task,sc=self.auto.check_contract(self.out,'fixed');c=self.auto.physical('delivery1','summit0',task['delivery1','summit0'],sc,True,self.out,'fixed');self.assertEqual(c[c.index('--trash')+1],'trash_A2_azul');self.assertEqual(c[c.index('--deposit')+1],'deposit_azul')
 def test_six_distinct_pickups(self):
  vals=[json.loads((self.out/f'assignment_{s}_FIXED.json').read_text()) for s in ['pickup1','pickup2']]
  names=[j['trash'] for a in vals for j in a['robots'].values()];self.assertEqual(len(set(names)),6)
 def test_milp_backward_compatibility(self):
  self.assertIn('--assignment-mode',pathlib.Path(self.auto.__file__).read_text())
if __name__=='__main__':unittest.main()
