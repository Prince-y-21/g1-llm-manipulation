"""Bounded pick/pour supervisor. Idle checkpoints return payload to support."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,time,json,subprocess,importlib,traceback,signal,shutil,fcntl
import numpy as np
ROOT=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(ROOT));import pour_action as a;import pour_payload_model as m
mission_lock=(ROOT/'mission.lock').open('w');fcntl.flock(mission_lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
runid=str(int(time.time()));last_request=0;holding=False;lifted=False;watch=None;success=False;pouring=False;angle=0
approach=json.loads((ROOT/'retry_approach_ready.json').read_text());restq=json.loads((ROOT/'route_rest_ready.json').read_text())[-1]['q'];outboardq=next(p['q'] for p in reversed(approach) if p.get('empty_closed'))
start_rest=np.array(a.gs()['q'][22:29]);source_q=None
def state(phase,**kw):
 s=a.gs();a.atomic('mission_state.json',json.dumps(dict(run_id=runid,phase=phase,unix_ms=int(time.time()*1000),temperature=s['temperatures'][22:24],angle=angle,**kw)))
def request():
 global last_request
 p=ROOT/'mission_request.json'
 if not p.exists():return None
 try:x=json.loads(p.read_text())
 except (ValueError,OSError):return None
 if x['request_id']<=last_request:return None
 last_request=x['request_id'];a.log('MISSION_REQUEST',request=x);return x
def thermal_check():
 s=a.gs();assert max(t[0] for t in s['temperatures'][22:24])<79 and max(t[1] for t in s['temperatures'][22:24])<110,'Reserve thermal margin reached'
def checkpoint(name,seconds=30):
 a.snapshot('mission_'+name);deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  thermal_check();state('WAIT_'+name.upper(),deadline_unix_ms=int((time.time()+deadline-time.monotonic())*1000))
  x=request()
  if x and x.get('action')=='continue' and x.get('checkpoint')==name:return
  if x and x.get('action')=='abort':raise RuntimeError('Requested safe return')
  time.sleep(.2)
 raise RuntimeError('Perception checkpoint timed out: '+name)
def stop_watch():
 global watch
 if watch is None:return
 (ROOT/'watch_stop').write_text('stop')
 try:watch.wait(timeout=5)
 except subprocess.TimeoutExpired:watch.terminate();watch.wait(timeout=3)
 watch=None
def merge_route(payload):
 route=approach+payload
 # Empty escape uses open-hand geometry; rest is widened for leg clearance.
 rest=np.array(restq);rest[1]=min(rest[1],-.25)
 wide=np.array(json.loads((ROOT/'route_safe_ready.json').read_text())[0]['q'])
 fold=next(p['q'] for p in json.loads((ROOT/'route_safe_ready.json').read_text()) if p['label']=='return_fold')
 supported=payload[0]['q'] if payload else approach[-1]['q'];m.EMPTY_CLOSED=False
 flags,c=m.segment(np.array(supported),np.array(outboardq),False,True)
 if flags:
  pre=next(p for p in approach if p['label']=='pick_pregrasp');ff,cc=m.segment(np.array(supported),np.array(pre['q']),False,True);assert not ff,ff
  ff,cc=m.segment(np.array(pre['q']),np.array(outboardq),False,True);assert not ff,ff
  route.append(dict(label='empty_pre_retract',q=pre['q'],carried=False,grasp=True,flags=[],endpoint_flag='',empty_closed=False))
 for label,q in [('empty_outboard',outboardq),('empty_wide_fold',fold),('empty_rest',rest.tolist())]:
  f,c=m.collision(q,False,True);assert not f,(label,f)
  route.append(dict(label=label,q=q,carried=False,grasp=True,flags=[],endpoint_flag='',empty_closed=False,reference_only=False,endpoint_reference=True))
 # Validate the actual post-release escape; normal execution repeats checks.
 ff,cc=m.segment(np.array(outboardq),np.array(fold),False,True);assert not ff,ff
 ff,cc=m.segment(np.array(fold),rest,False,True);assert not ff,ff
 a.atomic('mission_route_ready.json',json.dumps(route,indent=2));a.atomic('active_control.json',json.dumps(dict(side='right',indices=[22,29],route='mission_route_ready.json',model='pour_payload_model')))
def move(label):
 thermal_check();state('MOVING',label=label);a.move(label);thermal_check()
def return_payload():
 global holding,lifted,angle
 if not holding:return
 route=json.loads((ROOT/'mission_route_ready.json').read_text());by={p['label']:p for p in route};m.EMPTY_CLOSED=False
 # Always find a checked reverse path from ACTUAL pose; no direct tilt->table jump.
 desired=['pour_approach','transfer_4','transfer_3','transfer_2','transfer_1','lift_high','lift_mid','test_lift']
 if not lifted:desired=[]
 else:
  if angle>0:
   tilted=[p['label'] for p in route if p['label'].startswith('tilt_') and int(p['label'].split('_')[1])<=angle]
   desired=list(reversed(tilted))+desired
  # If recovery begins before transfer, skip unnecessary farther waypoints.
  s=a.gs();pnow,anow=m.payload(s['q'][22:29]);source=np.array(m.cal['mouth_body'])
  if np.linalg.norm((pnow-source)[:2])<.040:desired=['lift_mid','test_lift']
  while desired:
   s=a.gs();current=np.array(s['q'][22:29]);target_index=None
   for i in range(len(desired)-1,-1,-1):
    q=np.array(by[desired[i]]['q']);flags,c=m.segment(current,q,True,True)
    if not flags:target_index=i;break
   assert target_index is not None,'No checked reverse-carry path'
   name=desired[target_index];a.move(name);desired=desired[target_index+1:]
  a.move('putdown_reference')
 stop_watch();a.snapshot('mission_supported_before_open');a.grip(5.22,.25);holding=False;lifted=False;angle=0
 a.snapshot('mission_bottle_independent')
def empty_return():
 route=json.loads((ROOT/'mission_route_ready.json').read_text());by={p['label']:p for p in route};m.EMPTY_CLOSED=False
 if a.fresh('gripper_status.json')['q']<3.2:
  p=next(p for p in route if p['label']=='empty_outboard');p['empty_closed']=True;a.atomic('mission_route_ready.json',json.dumps(route,indent=2))
 s=a.gs();flags,c=m.segment(np.array(s['q'][22:29]),np.array(by['empty_outboard']['q']),False,True)
 if flags and 'empty_pre_retract' in by:a.move('empty_pre_retract')
 a.move('empty_outboard')
 for name in ['empty_rest','empty_wide_fold']:
  s=a.gs();flags,c=m.segment(np.array(s['q'][22:29]),np.array(by[name]['q']),False,True)
  if not flags:
   a.move(name)
   if name=='empty_wide_fold':a.move('empty_rest')
   break
 else:raise RuntimeError('No checked empty rest path')
 a.snapshot('mission_empty_rest')
def shutdown():
 stop_watch();pid=int((ROOT/'gripper.pid').read_text());proc=Path('/proc')/str(pid)
 if proc.exists() and 'pour_right_gripper' in (proc/'cmdline').read_text().replace('\0',' '):os.kill(pid,signal.SIGTERM)
 a.atomic('release_request','mission complete\n');deadline=time.monotonic()+9;pid=int((ROOT/'guard.pid').read_text());proc=Path('/proc')/str(pid)
 while time.monotonic()<deadline:
  if not proc.exists() or not (proc/'cmdline').read_bytes():break
  time.sleep(.2)
 assert not proc.exists() or not (proc/'cmdline').read_bytes(),'Body publisher did not exit'
 a.log('MISSION_CONTROLLERS_EXITED',success=success)
def start_watch():
 global watch
 if (ROOT/'watch_stop').exists():(ROOT/'watch_stop').rename(ROOT/('watch_stop_'+runid))
 watch=subprocess.Popen([str(ROOT/'pour_watch'),'mission_'+runid],stdout=(ROOT/('watch_'+runid+'.log')).open('w'),stderr=subprocess.STDOUT,start_new_session=True)
 (ROOT/'watch.pid').write_text(str(watch.pid));time.sleep(2);assert watch.poll() is None
def publish_review(req):
 # Save a compact diagnostic contact sheet around the actual maximum tilt.
 import cv2
 ws=a.fresh('watch_status.json');folder=Path(ws['folder']);rows=[]
 for line in (folder/'states.jsonl').read_text().splitlines():
  try:x=json.loads(line)
  except ValueError:continue
  if x['unix_ms']<req['started_ms']-250:continue
  _,axis=m.payload(x['guard']['q'][22:29]);x['angle_deg']=float(np.rad2deg(np.arccos(np.clip(axis.dot(m.normal),-1,1))));rows.append(x)
 if not rows:return
 top=max(rows,key=lambda x:x['angle_deg']);ids=np.linspace(max(0,rows.index(top)-4),min(len(rows)-1,rows.index(top)+4),4).astype(int);images=[]
 for i in ids:
  x=rows[i];im=cv2.imread(str(folder/('head_'+str(x['frame'])+'.jpg')))
  if im is None:continue
  cv2.putText(im,f"{x['angle_deg']:.1f} deg frame {x['frame']}",(8,22),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2);images.append(im)
 if images:
  sheet=np.concatenate([np.concatenate(images[:2],axis=1),np.concatenate(images[2:4],axis=1)],axis=0) if len(images)==4 else np.concatenate(images,axis=1)
  output=ROOT/('review_'+str(req['request_id'])+'.jpg');cv2.imwrite(str(output),sheet)
  a.atomic('mission_review.json',json.dumps(dict(request=req,path=str(output),actual_peak_angle_deg=top['angle_deg'],frame=top['frame'],watch_folder=str(folder))))
try:
 # Complete initial approach without stopping in a high-load posture to plan.
 assert max(t[0] for t in a.gs()['temperatures'][22:24])<=70
 for p in approach:
  if not p.get('empty_closed') and a.fresh('gripper_status.json')['q']<3.2:
   m.EMPTY_CLOSED=False;f,c=m.collision(a.gs()['q'][22:29],False,True);assert not f,('Open-hand clearance',f);a.grip(5.22,.25)
  move(p['label'])
 a.snapshot('mission_pregrasp_check')
 subprocess.check_call(['python3',str(ROOT/'pour_calibrate_payload.py'),'--scene-only'],timeout=20)
 m=importlib.reload(m);pred,axis=m.payload(a.gs()['q'][22:29])
 assert np.linalg.norm(pred-np.array(m.cal['mouth_body']))<.020,'Pregrasp neck/FK alignment uncertain'
 assert axis.dot(m.normal)>np.cos(np.deg2rad(6)),'Pregrasp bottle-axis alignment uncertain'
 a.grip(3.,.18);g=a.fresh('gripper_status.json');assert g['contact'],'Grasp did not contact';holding=True
 a.snapshot('mission_grip_contact')
 subprocess.check_call(['python3',str(ROOT/'pour_calibrate_payload.py')],timeout=20)
 subprocess.check_call(['python3',str(ROOT/'pour_leg_model.py')],timeout=10)
 subprocess.check_call(['python3',str(ROOT/'pour_plan_payload.py')],timeout=60)
 m=importlib.reload(m);payload=json.loads((ROOT/'route_payload_ready.json').read_text());merge_route(payload);source_q=payload[0]['q']
 move('test_lift');lifted=True;a.snapshot('mission_lift_verify')
 subprocess.check_call(['python3',str(ROOT/'pour_verify_lift.py')],timeout=20)
 verification=json.loads((ROOT/'lift_verification.json').read_text());rigid=verification['wrist_rigidity']
 wrist_ok=rigid.get('inliers',0)>=25 and .95<rigid.get('scale',0)<1.05 and abs(rigid.get('rotation_deg',100))<3 and np.linalg.norm(rigid.get('translation_px',[100,100]))<20
 assert verification['external_lift_supported'] and wrist_ok,('Lift not corroborated',verification)
 a.log('AUTOMATIC_LIFT_VERIFIED',verification=verification)
 for label in ['lift_mid','lift_high','transfer_1','transfer_2','transfer_3','transfer_4','pour_approach']:move(label)
 start_watch();checkpoint('pour',30);pouring=True;deadline=time.monotonic()+30
 while time.monotonic()<deadline:
  thermal_check();state('POUR_WAIT',deadline_unix_ms=int((time.time()+deadline-time.monotonic())*1000));req=request()
  if not req:time.sleep(.1);continue
  if req.get('action')=='finish':success=True;break
  if req.get('action')=='abort':raise RuntimeError('Requested return')
  req['started_ms']=int(time.time()*1000)
  labels=req.get('labels',[req.get('label')]);assert 0<len(labels)<=4
  for label in labels:
   assert label.startswith('tilt_');target_angle=int(label.split('_')[1]);assert target_angle<=angle+10 and target_angle>=0
   if req['action']=='pulse':a.pulse(label)
   elif req['action']=='tilt':move(label);angle=target_angle
   else:raise RuntimeError('Invalid pour request')
  publish_review(req);deadline=time.monotonic()+30
 else:raise RuntimeError('Pour perception wait timed out')
except Exception:
 error=traceback.format_exc();a.log('MISSION_ABORT',error=error);(ROOT/'mission_abort.txt').write_text(error);print(error,flush=True)
finally:
 try:
  state('RETURNING',success=success)
  # If failure occurred before payload routes were activated, restore an empty
  # retreat plan from the already checked approach rather than hold indefinitely.
  if not (ROOT/'mission_route_ready.json').exists():merge_route([])
  if holding:return_payload()
  empty_return();shutdown()
  a.atomic('mission_state.json',json.dumps(dict(run_id=runid,phase='DONE',success=success,unix_ms=int(time.time()*1000))))
 except Exception:
  error=traceback.format_exc();a.log('MISSION_RECOVERY_NEEDS_ATTENTION',error=error);(ROOT/'mission_attention.txt').write_text(error);a.atomic('mission_state.json',json.dumps(dict(run_id=runid,phase='RECOVERY_ATTENTION',success=False,error=error)));print(error,flush=True)
