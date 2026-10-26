from pathlib import Path
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
s=(old/'task_step.py').read_text().replace(str(old),str(r))
s=s.replace('import argparse',"import os\nos.environ['OPENBLAS_NUM_THREADS']='1'\nos.environ['OMP_NUM_THREADS']='1'\nimport argparse")
s=s.replace('assert error < .0001',"print('OFFLINE_IK_ERROR_M',float(error),'Q',solution.x.tolist(),'DELTA',delta.tolist(),flush=True)\nassert error < .0001")
a="model={}\nexec((work/'task_plan_final.py').read_text().split('\\nq0=np.array')[0],model)"
assert a in s
s=s.replace(a,"import pour_safety as safety\nmodel={'segment':lambda a,b,carried: safety.segment(a,b,carried,True)}")
s+='''
if args.send:
 import pour_action as audit
 audit.log('CARTESIAN_SENT',plan=plan)
 deadline=time.monotonic()+args.duration+6;accepted=False
 while time.monotonic()<deadline:
  live=audit.gs()
  if live['accepted_seq']==plan['seq']:accepted=True
  if accepted and not live['moving']:break
  time.sleep(.1)
 assert accepted and not live['moving'],'Step not complete'
 req=live['recenter_seq']+1;audit.atomic('recenter_request.txt',str(req)+'\\n');deadline=time.monotonic()+4
 while time.monotonic()<deadline:
  live=audit.gs()
  if live['recenter_seq']>=req:break
  time.sleep(.1)
 assert live['recenter_seq']>=req
 audit.log('CARTESIAN_COMPLETE',plan=plan,state=live)
 print('CARTESIAN_VERIFIED',plan['seq'],flush=True)
'''
(r/'pour_step.py').write_text(s);print('REUSED_CARTESIAN_STEP_READY')
