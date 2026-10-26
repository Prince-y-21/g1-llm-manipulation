from pathlib import Path
import sys,json,time,subprocess,numpy as np,traceback
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_action as a;import pour_payload_model as model
plan=json.loads((r/'thermal_recovery_route.json').read_text());oldpid=int((r/'guard.pid').read_text());g=a.fresh('gripper_status.json');assert g['q']>5 and not g['contact']
p=subprocess.Popen([str(r/'pour_guard_thermal'),'--thermal-unload-only'],stdout=(r/'guard_thermal.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(4);print('THERMAL_HANDOFF',p.pid,'exit',p.poll(),flush=True);assert p.poll() is None,(r/'guard_thermal.log').read_text()
op=Path('/proc')/str(oldpid);assert not op.exists() or not (op/'cmdline').read_bytes();(r/'guard_pre_thermal.pid').write_text(str(oldpid));(r/'guard.pid').write_text(str(p.pid));a.log('THERMAL_UNLOAD_HANDOFF',old_pid=oldpid,new_pid=p.pid)
started=time.monotonic()
try:
 for target in plan['targets']:
  assert time.monotonic()-started<220,'Recovery timeout';s=a.gs();assert not s['moving'];assert s['seq']+1==target['seq']
  q=np.array(target['q'])
  for origin in [np.array(s['q'][22:29]),np.array(s['desired'][22:29])]:
   flags,c=model.segment(origin,q,False,True);assert not flags,('Recovery actual-geometry reject',flags)
  a.atomic('arm_target.txt',str(target['seq'])+' '+str(target['duration'])+' '+' '.join(map(str,q))+'\n');a.log('THERMAL_UNLOAD_SENT',target=target,temperatures=s['temperatures'][22])
  deadline=time.monotonic()+target['duration']+6;accepted=False
  while time.monotonic()<deadline:
   s=a.gs()
   if s['accepted_seq']==target['seq']:accepted=True
   if accepted and not s['moving']:break
   time.sleep(.1)
  assert accepted and not s['moving']
  req=s['recenter_seq']+1;a.atomic('recenter_request.txt',str(req)+'\n');deadline=time.monotonic()+4
  while time.monotonic()<deadline:
   s=a.gs()
   if s['recenter_seq']>=req:break
   time.sleep(.1)
  assert s['recenter_seq']>=req
  a.log('THERMAL_UNLOAD_VERIFIED',target=target,state=s);print('UNLOAD_VERIFIED',target['seq'],target['label'],'T',s['temperatures'][22],flush=True)
 a.log('THERMAL_UNLOAD_LOW_LOAD_REACHED',state=a.gs());print('LOW_LOAD_REACHED',flush=True)
except Exception:
 a.log('THERMAL_UNLOAD_STOPPED',error=traceback.format_exc());raise
