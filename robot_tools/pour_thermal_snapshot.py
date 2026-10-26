from pathlib import Path
import subprocess,json,concurrent.futures,time
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as e:
 futures=[e.submit(subprocess.check_call,[str(old/'observe'),str(r/'thermal_passive.json')]),e.submit(subprocess.check_call,['python3',str(r/'pour_action.py'),'snapshot','thermal_unloaded'])]
 for f in futures:f.result()
s=json.loads((r/'guard_status.json').read_text());p=json.loads((r/'thermal_passive.json').read_text())
print(json.dumps(dict(utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),fault=s['fault'],reason=s['reason'],q=s['q'][22:29],desired=s['desired'][22:29],tau=s['measured_tau'][22:29],temperatures=s['temperatures'][22:29],motorstates=[j['motorstate'] for j in p['joints']],dq=s['dq'][22:29]),indent=2))
