import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import d98c4_wave_cbs_preview as mod

class SimpleGraph:
    vertices={(0,0),(1,0),(2,0)}
    def pos(self,v):return (float(v[0]),float(v[1]))

class TestD98C4(unittest.TestCase):
    def test_nearest(self):
        self.assertEqual(mod.nearest(SimpleGraph(), (1.2,0))[0],(1,0))
    def test_stationary_exclusion(self):
        self.assertEqual(mod.nearest(SimpleGraph(), (1.2,0), forbidden=[(1,0)],clearance=1.0)[0],(2,0))
    def test_wave_reject_same_deposit(self):
        s={'status':'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED','robots':{r:{'object':r} for r in mod.ROBOTS}}
        w={'status':'PROPOSED_WAVES_ONLY_BLOCKED_FOR_EXECUTION','waves':[{'number':1,'robots':[{'robot':'summit0','object':'summit0','deposit_color':'verde'},{'robot':'summit2','object':'summit2','deposit_color':'verde'}]},{'number':2,'robots':[{'robot':'summit1','object':'summit1','deposit_color':'azul'}]}]}
        with self.assertRaisesRegex(ValueError,'SAME_DEPOSIT'):
            mod.validate_inputs(s,w,{'schema':'swarm.d98c3.geometry_screen.v1'})
    def test_bad_wave_order(self):
        s={'status':'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED','robots':{r:{'object':r} for r in mod.ROBOTS}}
        w={'status':'PROPOSED_WAVES_ONLY_BLOCKED_FOR_EXECUTION','waves':[{'number':2,'robots':[{'robot':r,'object':r,'deposit_color':str(i)} for i,r in enumerate(mod.ROBOTS)]}]}
        with self.assertRaisesRegex(ValueError,'BAD_WAVE_ORDER'):
            mod.validate_inputs(s,w,{'schema':'swarm.d98c3.geometry_screen.v1'})

if __name__=='__main__':unittest.main()
