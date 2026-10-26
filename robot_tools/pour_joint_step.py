from pathlib import Path
import sys,json,time,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_action as a;import pour_safety as m
assert len(sys.argv)==3;j=int(sys.argv[1]);delta=float(sys.argv[2]);assert 0<=j<7 and abs(delta)<=.10
s=a.gs();assert not s['moving'];q=np.array(s['desired'][22:29]);goal=q.copy();goal[j]+=delta
for origin in [q,np.array(s['q'][22:29])]:
 flags,c=m.segment(origin,goal,False,True);assert not flags,flags
p0,r0=m.base.tcp_pose(q);p1,r1=m.base.tcp_pose(goal);seq=s['seq']+1;duration=max(5.,abs(delta)*1.5/.025+.1)
assert .8<duration<30
a.log('JOINT_ALIGN_SENT',seq=seq,joint=j+22,delta=delta,tcp_delta=(p1-p0).tolist(),q=goal.tolist())
a.atomic('arm_target.txt',f'{seq} {duration} '+' '.join(map(str,goal))+'\n')
accepted=False;deadline=time.monotonic()+duration+6
while time.monotonic()<deadline:
 s=a.gs()
 if s['accepted_seq']==seq:accepted=True
 if accepted and not s['moving']:break
 time.sleep(.1)
assert accepted and not s['moving']
req=s['recenter_seq']+1;a.atomic('recenter_request.txt',str(req)+'\n');deadline=time.monotonic()+4
while time.monotonic()<deadline:
 s=a.gs()
 if s['recenter_seq']>=req:break
 time.sleep(.1)
assert s['recenter_seq']>=req
a.log('JOINT_ALIGN_COMPLETE',seq=seq,state=s);print('JOINT_ALIGN_COMPLETE',seq,'tcp_delta',p1-p0,'temperature',s['temperatures'][22],flush=True)
