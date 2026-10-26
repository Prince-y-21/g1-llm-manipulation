from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_safety as m
s=json.loads((r/'guard_status.json').read_text());assert s['fault'] and s['reason'].startswith('TRACKING_JOINT_25')
q=np.array(s['q'][22:29]);goal=q.copy();goal[1]=-.65
values=[]
for w in np.linspace(0,1,101):
 x=q*(1-w)+goal*w;f,c=m.base.collision(x);assert not f,('Other escape geometry',f)
 values.append(m.leg_clearance(x))
assert min(np.diff(values))>-.0003,'Escape clearance decreases'
assert values[-1]>.030
out=dict(start=q.tolist(),target=goal.tolist(),start_clearance=values[0],final_clearance=values[-1],minimum_monotonic_increment=float(min(np.diff(values))),reason='Visual proximity to right thigh; model overlap; outward egress, not repetition of elbow extension')
(r/'escape_plan.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
