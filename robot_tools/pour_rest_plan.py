from pathlib import Path
import sys,json,numpy as np,itertools
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_payload_model as m
s=json.loads((r/'guard_status.json').read_text());assert not s['fault'] and not s['moving'];g=json.loads((r/'gripper_status.json').read_text());assert g['q']<3.2 and not g['contact'];m.EMPTY_CLOSED=True
start=np.array(s['q'][22:29]);candidates=[]
for pitch,roll,elbow,wroll,wpitch in itertools.product([-.05,0,.05],[-.10,-.15,-.20,-.25,-.30],[1.2,1.4,1.57,1.75],[-1.5,-.75,0,.75,1.5],[-.2,0,.2]):
 q=np.array([pitch,roll,.334,elbow,wroll,wpitch,0.]);f,c=m.collision(q,False,False)
 if f:continue
 t,p=m.fk(q);shoulder=p[1];com=1.5*((p[1]+p[3])/2-shoulder)+.9*((p[3]+p[6])/2-shoulder)+.9*(t[:3,3]+t[:3,:3].dot([.10,0,0])-shoulder)
 cost=float(np.linalg.norm(com[:2]));candidates.append((cost,q))
assert candidates,'No resting candidates'
chosen=None
for cost,q in sorted(candidates,key=lambda x:x[0])[:60]:
 flags,c=m.segment(start,q,False,False)
 if not flags:chosen=[q];break
 wide=q.copy();wide[1]=start[1]
 flags,c=m.segment(start,wide,False,False)
 if flags:continue
 flags,c=m.segment(wide,q,False,False)
 if not flags:chosen=[wide,q];break
assert chosen is not None,'No checked rest path'
out=[];prev=start
for i,q in enumerate(chosen):
 flags,c=m.segment(prev,q,False,False);assert not flags
 out.append(dict(label='rest_'+str(i+1),q=q.tolist(),carried=False,grasp=False,flags=[],endpoint_flag='',clearance=c));prev=q
(r/'route_rest_ready.json').write_text(json.dumps(out,indent=2));(r/'active_control.json').write_text(json.dumps(dict(side='right',indices=[22,29],route='route_rest_ready.json',model='pour_payload_model',empty_closed=True)))
print('REST_PLAN_READY','candidates',len(candidates),'moment_proxy',cost,'q',[p['q'] for p in out],flush=True)
