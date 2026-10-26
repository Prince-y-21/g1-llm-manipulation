from pathlib import Path
import sys,json,time,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_action as a;import pour_payload_model as m
m.EMPTY_CLOSED=True;g=a.fresh('gripper_status.json');assert g['q']<3.2 and not g['contact']
for k in range(6):
 s=a.gs();actual=np.array(s['q'][22:29]);f,c=m.collision(actual,False,True)
 if not f:break
 assert f=='LEG' and not s['moving']
 goal=np.array(s['desired'][22:29]);goal[1]=actual[1]-.035
 for origin in [actual,np.array(s['desired'][22:29])]:
  vals=[]
  for w in np.linspace(0,1,61):
   ff,cc=m.collision(origin*(1-w)+goal*w,False,True);assert ff in ['', 'LEG'],ff;vals.append(cc)
  assert min(vals)>=c-.001 and vals[-1]>c+.003,'Egress does not improve proximity'
 seq=s['seq']+1;a.atomic('arm_target.txt',f'{seq} 3.5 '+' '.join(map(str,goal))+'\n');a.log('REST_EGRESS_SENT',seq=seq,target=goal.tolist(),before=s)
 deadline=time.monotonic()+10;accepted=False
 while time.monotonic()<deadline:
  s=a.gs();accepted=accepted or s['accepted_seq']==seq
  if accepted and not s['moving']:break
  time.sleep(.1)
 assert accepted and not s['moving']
 ff,cc=m.collision(s['q'][22:29],False,True);assert cc>=c-.001 and s['q'][23]<actual[1]-.003
 req=s['recenter_seq']+1;a.atomic('recenter_request.txt',str(req)+'\n');deadline=time.monotonic()+4
 while time.monotonic()<deadline:
  s=a.gs()
  if s['recenter_seq']>=req:break
  time.sleep(.1)
 assert s['recenter_seq']>=req;a.log('REST_EGRESS_VERIFIED',state=s,clearance=cc);print('EGRESS',seq,ff,cc,flush=True)
assert not m.collision(a.gs()['q'][22:29],False,True)[0]
print('REST_EGRESS_CLEAR',flush=True)
