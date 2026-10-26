from pathlib import Path
import subprocess,time,json
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
route=json.loads((r/'route_right_ready.json').read_text());assert all(not p['flags'] and not p['endpoint_flag'] for p in route)
assert json.loads((r/'pulse_trajectory_check.json').read_text())['passed']
for f in ['arm_target.txt','pulse_target.txt','release_request','stop_guard','pause_request','guard.pid']:
 assert not (r/f).exists(),'Stale '+f
subprocess.check_call([str(old/'release_ai')],timeout=10)
p=subprocess.Popen([str(r/'pour_guard_pulse'),'--execute-suspended'],stdout=(r/'guard_right.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'guard.pid').write_text(str(p.pid));time.sleep(4)
print('BODY_PID',p.pid,'exit',p.poll());print((r/'guard_right.log').read_text());assert p.poll() is None
s=json.loads((r/'guard_status.json').read_text());assert not s['fault']
print(json.dumps(dict(age=s['age_s'],q=s['q'],initial=s['initial'],temperatures=s['temperatures'])))
