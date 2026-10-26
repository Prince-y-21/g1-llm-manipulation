import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_payload_model as m
ns={};source=(r/'pour_plan_payload.py').read_text().split("add('supported_grasp'")[0].replace("assert g['contact']","assert not g['contact']");exec(source,ns);ik=ns['ik']
s=json.loads((r/'guard_status.json').read_text());assert not s['fault'] and not s['moving'];g=json.loads((r/'gripper_status.json').read_text());assert g['q']<3.2 and not g['contact']
old=json.loads((r/'route_safe_ready.json').read_text());by={p['label']:p for p in old};out=[];current=np.array(s['q'][22:29]);m.EMPTY_CLOSED=True
def add(label,q,closed=False):
 global current
 m.EMPTY_CLOSED=closed;flags,c=m.segment(current,q,False,True);f,_=m.collision(q,False,True);assert not flags and not f,(label,flags,f)
 out.append(dict(label=label,q=np.array(q).tolist(),carried=False,grasp=True,flags=[],endpoint_flag='',clearance=c,empty_closed=closed));current=np.array(q);print('PICK_PLAN',label,flush=True)
# Use the farthest checked initial target, with a closed empty hand near legs.
chain=['return_fold','return_safe_level','outboard'];idx=0
while idx<len(chain):
 for j in range(len(chain)-1,idx-1,-1):
  q=np.array(by[chain[j]]['q']);flags,c=m.segment(current,q,False,True)
  if not flags:break
 else:raise RuntimeError('No checked approach from rest')
 add('pick_closed_'+str(len(out)+1),q,True);idx=j+1
m.EMPTY_CLOSED=False;f,c=m.collision(current,False,True);assert not f,('Cannot safely open here',f)
mouth=np.array(m.cal['mouth_body']);N=m.normal
pre=mouth+[-.060,0,0]+N*.012
qpre=ik(pre,N,current)
flags,c=m.segment(current,qpre,False,True)
if flags:add('pick_bridge',np.array(by['bridge']['q']),False);qpre=ik(pre,N,current)
add('pick_pregrasp',qpre,False)
goal=ik(mouth,N,current);add('pick_grasp',goal,False)
(r/'retry_approach_ready.json').write_text(json.dumps(out,indent=2));(r/'active_control.json').write_text(json.dumps(dict(side='right',indices=[22,29],route='retry_approach_ready.json',model='pour_payload_model')))
print('RETRY_APPROACH_READY',len(out),flush=True)
