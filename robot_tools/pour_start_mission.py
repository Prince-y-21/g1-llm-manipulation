from pathlib import Path
import subprocess,os,json,time,shutil
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');assert (r/'route_payload_predicted_ready.json').exists()
if (r/'mission.pid').exists():
 proc=Path('/proc')/(r/'mission.pid').read_text().strip();assert not proc.exists() or 'pour_mission.py' not in (proc/'cmdline').read_text().replace('\0',' '),'Mission already running'
s=json.loads((r/'guard_status.json').read_text());assert not s['fault'] and not s['moving'];assert max(t[0] for t in s['temperatures'][22:24])<=70
for name in ['mission_route_ready.json','mission_attention.txt','mission_abort.txt','mission_request.json','mission_state.json']:
 if (r/name).exists():(r/name).rename(r/(name+'.before_'+str(time.time_ns())))
p=subprocess.Popen(['python3','-u',str(r/'pour_mission.py')],stdout=(r/'mission.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True);(r/'mission.pid').write_text(str(p.pid));print('MISSION_PID',p.pid)
