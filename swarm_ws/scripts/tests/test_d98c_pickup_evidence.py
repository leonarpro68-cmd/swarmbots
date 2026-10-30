import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98c_pickup_evidence import audit,ROBOTS
H={'status':'READY_FOR_PHYSICAL_CONFIRMATION_ONLY','stage':'pickup1','stage_object_names_from_D8_plan':['trash_V2_verde','trash_V1_verde','trash_A2_azul']}
def reports():
 return {r:{'status':'GRASP_ACK_RECEIVED_NO_LEDGER_COMMIT','robot_results':[{'robot':r,'object':H['stage_object_names_from_D8_plan'][i],'status':'GRASP_MANAGER_ACK_NO_LEDGER_COMMIT','steps':[{'phase':'GRASP_ACK_ATTACHED_BY_D8_MANAGER'}]}]} for i,r in enumerate(ROBOTS)}
class TestEvidence(unittest.TestCase):
 def test_all_ack(self):self.assertEqual(audit(H,reports())['status'],'ACK_AUDIT_PASSED_NOT_ATTACHMENT_CERTIFIED')
 def test_missing(self):
  a=reports();a.pop('summit1');self.assertEqual(audit(H,a)['status'],'BLOCKED')
 def test_wrong_object(self):
  a=reports();a['summit1']['robot_results'][0]['object']='trash_V2_verde';self.assertEqual(audit(H,a)['status'],'BLOCKED')
 def test_invalid_handoff(self):
  h=dict(H,status='BLOCKED');self.assertEqual(audit(h,reports())['status'],'BLOCKED')
if __name__=='__main__':unittest.main()
