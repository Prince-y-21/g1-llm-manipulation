from pathlib import Path
import tempfile,unittest,json,hashlib
from compile_thermal_contract_v4 import compile_contract
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.r=Path(self.tmp.name);self.tr=self.r/'trajectory_1.txt';self.pr=self.r/'proof.json';self.out=self.r/'thermal_1.txt'
  self.tr.write_text('CAPSEG1 run 1 3\n'+'\n'.join(str(t)+' '+' '.join(['0']*28) for t in [0,1,2])+'\n')
  self.d=dict(run_id='run',sequence=1,trajectory_sha256=hashlib.sha256(self.tr.read_bytes()).hexdigest(),full_geometry_checked=True,payload_envelope_checked=True,model_uncertainty_envelope_checked=True,interval_load_monotonicity_checked=True,start_kind='PAYLOAD_SUPPORT_TARGET_MODEL',end_kind='PAYLOAD_SUPPORT_TARGET_MODEL',reverse_prefixes_geometry_checked=True,forward_suffixes_geometry_checked=True,start_endpoint_model_checked=True,end_endpoint_model_checked=True,per_arm_joint_model_load_Nm=[[v]+[0]*13 for v in [1,3,1]],protected_body_joints=[15],gripper_binding=dict(left_moving=False,right_moving=False,left_seq=9,right_seq=0,left_sent_target=0,right_sent_target=5.1),contract_id='c',scene_id='s',payload_id='p',role='CARRY')
 def tearDown(self):self.tmp.cleanup()
 def runcompile(self):self.pr.write_text(json.dumps(self.d));return compile_contract(self.tr,self.pr,self.out)
 def test_held_low_load_not_external_support(self):
  self.d['start_kind']='PAYLOAD_HELD_LOW_LOAD_MODEL'
  self.assertEqual(self.runcompile()['protected_mask'],1)
 def test_valid(self):self.assertEqual(self.runcompile()['protected_mask'],1)
 def test_reject_missing_geometry(self):
  self.d['full_geometry_checked']=False
  with self.assertRaises(AssertionError):self.runcompile()
 def test_no_actual_support_claim(self):
  self.d['end_kind']='PHYSICALLY_VERIFIED_SUPPORT'
  with self.assertRaises(AssertionError):self.runcompile()
 def test_hash_changed(self):
  self.tr.write_text(self.tr.read_text()+' ')
  with self.assertRaises(AssertionError):self.runcompile()
 def test_valley(self):
  self.d['per_arm_joint_model_load_Nm']=[[v]+[0]*13 for v in [3,1,3]]
  with self.assertRaises(AssertionError):self.runcompile()
 def test_protected_conflict(self):
  self.d['protected_body_joints']=[15,16];self.d['per_arm_joint_model_load_Nm']=[[1,3]+[0]*12,[2,2]+[0]*12,[3,1]+[0]*12]
  with self.assertRaises(AssertionError):self.runcompile()
 def test_moving_gripper(self):
  self.d['gripper_binding']['left_moving']=True
  with self.assertRaises(AssertionError):self.runcompile()
 def test_no_auto_explicit(self):
  self.d.update(automatic_relief=False,role='NO_AUTOMATIC_RELIEF',uncovered_reason='nonmonotonic actual model',start_kind='NONE',end_kind='NONE',protected_body_joints=[])
  self.assertEqual(self.runcompile()['protected_mask'],0)
 def test_no_false_implicit_fallback(self):
  self.d.update(start_kind='NONE',end_kind='NONE')
  with self.assertRaises(AssertionError):self.runcompile()
if __name__=='__main__':unittest.main()
