from pathlib import Path
import sys,json,numpy as np,subprocess,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_safety as m
s=json.loads((r/'guard_status.json').read_text());assert not s['fault'] and not s['moving']
q0=np.array(s['q'][22:29]);assert m.leg_clearance(q0)>.03
previous=json.loads((r/'route_right_extended.json').read_text());by={p['label']:p for p in previous}
out=[]
def add(label,q,carried=False,grasp=False,base_validated=False):
 if base_validated:
  a=np.array(out[-1]['q']);b=np.array(q);c=min(m.leg_clearance(a*(1-w)+b*w,carried) for w in np.linspace(0,1,81));flags=['LEG'] if c<.012 else []
 else:flags,c=m.segment(out[-1]['q'],q,carried,grasp) if out else ([],10)
 f,_=m.collision(q,carried,grasp);pos,rr=m.base.tcp_pose(q)
 out.append(dict(label=label,q=np.array(q).tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=carried,grasp=grasp,flags=flags,endpoint_flag=f,clearance=c))
 print(label,flags,'endpoint',f,'clearance',c,flush=True)
add('resume_initial',q0)
q=q0.copy();q[3]=0.;add('safe_unfold',q)
q[5]=-.4;add('safe_level',q)
safe_level=q.copy()
for p in previous[next(i for i,p in enumerate(previous) if p['label']=='outboard'):]:add(p['label'],p['q'],p['carried'],p['grasp'],p['label']!='outboard')
# Return is checked independently from the outbound low-arm path.
return_path=[]
fold=np.array(by['initial']['q']);fold[0]=q0[0];fold[1]=q0[1]
pitchback=fold.copy();pitchback[0]=by['initial']['q'][0]
return_defs=[('return_safe_level',safe_level),('return_fold',fold),('return_pitch',pitchback),('return_initial',np.array(by['initial']['q']))]
prev=np.array(by['outboard']['q'])
for label,q in return_defs:
 flags,c=m.segment(prev,q,False,False);f,_=m.collision(q,False,False)
 pos,rr=m.base.tcp_pose(q);return_path.append(dict(label=label,q=q.tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=False,grasp=False,flags=flags,endpoint_flag=f,clearance=c));prev=q
 print(label,flags,'endpoint',f,flush=True)
(r/'route_safe_candidate.json').write_text(json.dumps(out+return_path,indent=2))
assert all(not p['flags'] and not p['endpoint_flag'] for p in out),'Unsafe working route'
# Do not block a safe working route on a rejected optional exact-posture return.
validreturn=[]
for p in return_path:
 if p['flags'] or p['endpoint_flag']:break
 validreturn.append(p)
(r/'route_safe_ready.json').write_text(json.dumps(out+validreturn,indent=2))
(r/'active_control.json').write_text(json.dumps(dict(side='right',indices=[22,29],route='route_safe_ready.json',model='pour_safety')))
print('SAFE_READY',len(out),'return_nodes',len(validreturn),flush=True)
