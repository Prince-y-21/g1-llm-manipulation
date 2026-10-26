from pathlib import Path
import sys,json,time,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_action as a;import pour_safety as m
s=a.gs();assert not s['moving'];target=np.array(json.loads((r/'escape_plan.json').read_text())['target'])
for k in range(12):
 s=a.gs();assert not s['moving'];q=np.array(s['desired'][22:29]);actual=np.array(s['q'][22:29])
 if actual[1]<=-.625:break
 goal=q.copy();goal[1]=max(-.65,actual[1]-.035)
 oldclear=m.leg_clearance(actual)
 for origin in [q,actual]:
  clears=[]
  for w in np.linspace(0,1,31):
   x=origin*(1-w)+goal*w;f,_=m.base.collision(x);assert not f,f;clears.append(m.leg_clearance(x))
  assert min(clears)>oldclear-.002,'Escape worsens proximity'
 seq=s['seq']+1;duration=3.5;a.atomic('arm_target.txt',f'{seq} {duration} '+' '.join(map(str,goal))+'\n');a.log('EGRESS_SENT',seq=seq,q=goal.tolist(),clearance=oldclear)
 deadline=time.monotonic()+9;accepted=False
 while time.monotonic()<deadline:
  s=a.gs()
  if s['accepted_seq']==seq:accepted=True
  if accepted and not s['moving']:break
  time.sleep(.1)
 assert accepted and not s['moving']
 new=np.array(s['q'][22:29]);newclear=m.leg_clearance(new)
 assert new[1]<actual[1]-.003 and newclear>=oldclear-.001,'No measured outward egress'
 req=s['recenter_seq']+1;a.atomic('recenter_request.txt',str(req)+'\n');deadline=time.monotonic()+4
 while time.monotonic()<deadline:
  s=a.gs()
  if s['recenter_seq']>=req:break
  time.sleep(.1)
 assert s['recenter_seq']>=req
 a.log('EGRESS_VERIFIED',seq=seq,clearance=newclear,state=s);print('EGRESS',seq,'roll',new[1],'clearance',newclear,'temperature',s['temperatures'][22],flush=True)
s=a.gs();assert m.leg_clearance(np.array(s['q'][22:29]))>.03,'Not clear of leg'
a.log('EGRESS_COMPLETE',state=s);print('LEG_CLEARANCE_CONFIRMED',flush=True)
