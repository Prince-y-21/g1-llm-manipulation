"""Explicit single actions for this experiment; append execution evidence."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import sys,time,json,hashlib,shutil,subprocess
from pathlib import Path
import numpy as np
ROOT=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
def log(kind,**kw):
 record=dict(utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),unix_ms=int(time.time()*1000),event=kind,**kw)
 with (ROOT/'actions.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
def atomic(name,value):
 p=ROOT/(name+'.tmp');p.write_text(value);os.replace(str(p),str(ROOT/name))
def fresh(name):
 p=ROOT/name;assert time.time()-p.stat().st_mtime<1.2,'Stale '+name
 return json.loads(p.read_text())
def gs():
 s=fresh('guard_status.json');assert not s['fault'],s['reason'];assert not s['releasing'];return s
def grip(q,speed):
 s=fresh('gripper_status.json');seq=s['seq']+1
 assert not s['error'] and .01<=speed<=.25 and 0<=q<=5.4
 atomic('gripper_target.txt',f'{seq} {q} {speed}\n');log('GRIPPER_SENT',seq=seq,target=q,speed=speed,before=s)
 deadline=time.monotonic()+abs(q-s['q'])/speed+8
 while time.monotonic()<deadline:
  s=fresh('gripper_status.json');assert not s['error']
  if s['seq']==seq and (s['contact'] or (abs(s['q']-q)<.035 and abs(s['dq'])<.03)):
   log('GRIPPER_COMPLETE',state=s);print(json.dumps(s),flush=True);return
  time.sleep(.1)
 log('GRIPPER_NOT_AT_TARGET',state=s);print(json.dumps(s),flush=True);raise RuntimeError('Gripper target not reached; inspect actual resistance')
def snapshot(label):
 assert label.replace('_','').isalnum()
 stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());out=ROOT/'snapshots'/(stamp+'_'+label);out.mkdir(parents=True,exist_ok=False)
 meta=dict(utc=stamp,label=label)
 for name in ['guard_status.json','gripper_status.json']:
  if (ROOT/name).exists():meta[name+'_before']=json.loads((ROOT/name).read_text())
 watch=ROOT/'watch_status.json'
 watch_live=False
 if watch.exists():
  try:watch_live=time.time()*1000-json.loads(watch.read_text())['unix_ms']<1200
  except (ValueError,OSError):pass
 if watch_live:
  meta['capture_source']='continuous_watch'
  for src,dst in [('latest_head.jpg','head_color.jpg'),('latest_depth.png','head_depth.png'),('latest_wrist0.jpg','wrist0.jpg'),('latest_wrist8.jpg','wrist8.jpg')]:
   if (ROOT/src).exists():shutil.copy2(str(ROOT/src),str(ROOT/dst))
 else:subprocess.check_call([str(ROOT/'camera')],timeout=25)
 for name in ['head_color.jpg','head_depth.png','head_intrinsics.txt','wrist0.jpg','wrist8.jpg','guard_status.json','gripper_status.json']:
  if (ROOT/name).exists():shutil.copy2(str(ROOT/name),str(out/name))
 (out/'capture.json').write_text(json.dumps(meta,indent=2));log('SNAPSHOT',path=str(out));print('SNAPSHOT',out,flush=True)
def move(label,reverse=False):
 import importlib
 config=json.loads((ROOT/'active_control.json').read_text());i0,i1=config['indices']
 model=importlib.import_module(config['model'])
 route_path=ROOT/config['route'];route=json.loads(route_path.read_text());assert all(not p['flags'] for p in route)
 found=[p for p in route if p['label']==label];assert len(found)==1
 p=found[0];carried=p['carried'] and not reverse;grasp=p['grasp'] or reverse
 if hasattr(model,'EMPTY_CLOSED'):
  model.EMPTY_CLOSED=bool(p.get('empty_closed',config.get('empty_closed',False)))
  if model.EMPTY_CLOSED:
   g=fresh('gripper_status.json');assert g['q']<3.2 and not g['contact'],'Empty closed-hand geometry requires verified closed empty gripper'
 s=gs();assert not s['moving']
 if p.get('reference_only'):
  pre=next(v for v in route if v['label']==p['required_predecessor'])
  assert np.max(abs(np.array(s['q'][i0:i1])-np.array(pre['q'])))<.06,'Reference endpoint requires the checked preceding phase'
 if carried:
  g=fresh('gripper_status.json');assert g['contact'] and not g['error'],'No held-bottle feedback'
 start=np.array(s['desired'][i0:i1]);goal=np.array(p['q']);step_limit=.10 if label.startswith('deeper_grasp') else .175;count=max(1,int(np.ceil(np.max(abs(goal-start))/step_limit)))
 log('PHASE_START',label=label,reverse=reverse,route_sha256=hashlib.sha256(route_path.read_bytes()).hexdigest(),before=s)
 for k in range(1,count+1):
  s=gs();assert not s['moving'];current=np.array(s['desired'][i0:i1]);q=start+(goal-start)*(k/count)
  assert np.max(abs(q-current))<.24
  flags=set()
  for origin in [current,np.array(s['q'][i0:i1])]:
   for w in np.linspace(0,1,25):
    f,_=model.collision(origin*(1-w)+q*w,carried,grasp)
    if f:flags.add(f)
  assert not flags,('Actual segment rejected',flags)
  speed=.025 if label in ['pregrasp','grasp','putdown','pick_grasp','pick_pregrasp'] or label.startswith('deeper_grasp') or label.startswith('putdown') else .06
  seq=s['seq']+1;duration=max(3.3,float(np.max(abs(q-current)))*1.5/speed+.15)
  assert duration<30
  atomic('arm_target.txt',f'{seq} {duration} '+' '.join(map(str,q))+'\n');log('BODY_SENT',seq=seq,label=label,duration=duration,q=q.tolist())
  deadline=time.monotonic()+duration+5;accepted=False;last=0
  while time.monotonic()<deadline:
   s=gs()
   if s['accepted_seq']==seq:accepted=True
   if time.monotonic()-last>.5:log('BODY_FEEDBACK',label=label,state=s);last=time.monotonic()
   if accepted and not s['moving']:break
   time.sleep(.1)
  assert accepted and not s['moving'],'Motion timeout'
  assert np.max(abs(np.array(s['q'][i0:i1])-q))<=.04
  # Sampled static effort transfer; never accumulate integral correction.
  req=s['recenter_seq']+1;atomic('recenter_request.txt',str(req)+'\n');deadline=time.monotonic()+4
  while time.monotonic()<deadline:
   s=gs()
   if s['recenter_seq']>=req:break
   time.sleep(.1)
  assert s['recenter_seq']>=req,'Effort transfer not accepted'
  log('BODY_VERIFIED',seq=seq,label=label,state=s)
  print('VERIFIED',label,seq,'temperature',s['temperatures'][i0],'max_error',float(np.max(abs(np.array(s['q'][i0:i1])-q))),flush=True)
 log('PHASE_COMPLETE',label=label,state=s)
def pulse(label):
 import importlib
 config=json.loads((ROOT/'active_control.json').read_text());i0,i1=config['indices'];assert [i0,i1]==[22,29]
 model=importlib.import_module(config['model']);route=json.loads((ROOT/config['route']).read_text());assert all(not p['flags'] for p in route)
 p=next(p for p in route if p['label']==label);assert p['carried']
 s=gs();assert not s['moving'];g=fresh('gripper_status.json');assert g['contact'] and not g['error']
 watch=fresh('watch_status.json');assert time.time()*1000-watch['unix_ms']<1200 and watch['frame']>10,'Camera not ready'
 current=np.array(s['desired'][i0:i1]);q=np.array(p['q']);delta=float(np.max(abs(q-current)));assert delta<.24
 for origin in [current,np.array(s['q'][i0:i1])]:
  flags,_=model.segment(origin,q,True,True);assert not flags,flags
 duration=max(1.2,delta*1.5/.07+.1);assert duration<=10
 seq=s['seq']+1;atomic('pulse_target.txt',f'{seq} {duration} .25 '+' '.join(map(str,q))+'\n')
 log('PULSE_SENT',seq=seq,label=label,from_q=current.tolist(),target=q.tolist(),duration=duration,hold=.25)
 deadline=time.monotonic()+2*duration+.25+5;accepted=False;last=0
 while time.monotonic()<deadline:
  s=gs()
  if s['accepted_seq']==seq:accepted=True
  if time.monotonic()-last>.2:log('PULSE_FEEDBACK',seq=seq,state=s);last=time.monotonic()
  if accepted and not s['moving']:break
  time.sleep(.05)
 assert accepted and not s['moving'],'Pulse timeout'
 assert np.max(abs(np.array(s['q'][i0:i1])-current))<.04,'Pulse did not return'
 log('PULSE_RETURNED',seq=seq,label=label,state=s);print('PULSE_RETURNED',label,seq,'temperature',s['temperatures'][i0],flush=True)
if __name__=='__main__':
 if sys.argv[1]=='snapshot':snapshot(sys.argv[2])
 elif sys.argv[1]=='grip':grip(float(sys.argv[2]),float(sys.argv[3]))
 elif sys.argv[1]=='move':move(sys.argv[2],len(sys.argv)>3 and sys.argv[3]=='empty')
 elif sys.argv[1]=='pulse':pulse(sys.argv[2])
 else:raise ValueError('Unknown action')
