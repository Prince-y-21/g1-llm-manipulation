from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_payload_model as m
s=json.loads((r/'guard_status.json').read_text());q=np.array(s['q'][22:29]);m.EMPTY_CLOSED=True
print('CURRENT',q.tolist(),m.collision(q,False,True))
for p in json.loads((r/'route_safe_ready.json').read_text()):
 if p['label'] in ['return_fold','return_safe_level','outboard']:
  print(p['label'],m.segment(q,np.array(p['q']),False,True))
for roll in [-.1,-.2,-.3,-.4,-.5,-.6]:
 target=q.copy();target[1]=roll;print('ROLL',roll,m.segment(q,target,False,True),m.collision(target,False,True))
