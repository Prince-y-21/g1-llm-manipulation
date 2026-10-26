from pathlib import Path
import json,sys,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_current_model as m
s=json.loads((r/'guard_status.json').read_text());q=np.array(s['q'][22:29]);t,p=m.fk(q);g,rot=m.tcp_pose(q)
print('CURRENT',json.dumps(dict(q=q.tolist(),points=p.tolist(),tcp=g.tolist(),model_flag=m.collision(q),measured_tau=s.get('measured_tau',[0]*29)[22:29],desired=s['desired'][22:29],ff=s['feedforward'][22:29],temperature=s['temperatures'][22:29])))
ev=[json.loads(x) for x in (r/'actions.jsonl').read_text().splitlines()]
for e in ev:
 if e['event']=='BODY_FEEDBACK' and e['state']['accepted_seq']==10:
  a=e['state'];print('SEQ10',e['utc'],'moving',a['moving'],'fault',a['fault'],'q25',a['q'][25],'target25',a['desired'][25],'dq25',a['dq'][25],'tau25',a.get('measured_tau',[0]*29)[25])
