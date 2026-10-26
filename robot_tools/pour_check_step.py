from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_current_model as m
s=json.loads((r/'guard_status.json').read_text());p=max(r.glob('step_51_*.json'),key=lambda p:p.stat().st_mtime);a=json.loads(p.read_text());b=m.fk(a['observed_q'])[0];t=m.fk(s['q'][22:29])[0]
print(json.dumps(dict(actual_wrist_delta=(t[:3,3]-b[:3,3]).tolist(),desired=s['desired'][22:29],actual=s['q'][22:29],dq=s['dq'][22:29],tau=s['measured_tau'][22:29],ff=s['feedforward'][22:29],temperatures=s['temperatures'][22:29],current_tcp=m.tcp_pose(s['q'][22:29])[0].tolist()),indent=2))
