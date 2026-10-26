from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_safety as safety
sys.argv=['pour_step.py','.02','0','.02','--duration','8']
src=(r/'pour_step.py').read_text().split('assert np.max(np.abs(delta)) <= .12')[0];ns={};exec(src,ns)
q=ns['q_target'];s=ns['status'];assert np.max(abs(q-np.array(s['desired'][22:29])))<.60
for origin in [np.array(s['q'][22:29]),np.array(s['desired'][22:29])]:
 flags,c=safety.segment(origin,q,False,True);assert not flags,flags
cfg=json.loads((r/'active_control.json').read_text());route=json.loads((r/cfg['route']).read_text());label='deeper_grasp1'
assert all(p['label']!=label for p in route)
p=dict(label=label,q=q.tolist(),carried=False,grasp=True,flags=[],endpoint_flag='',planned_from_seq=s['seq'],requested_delta=[.02,0,.02],planned_from_q=s['q'][22:29])
route.append(p);(r/'route_deeper_ready.json').write_text(json.dumps(route,indent=2));cfg['route']='route_deeper_ready.json';(r/'active_control.json').write_text(json.dumps(cfg));print('DEEPER_READY',p,flush=True)
