from pathlib import Path
import json,time
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');ready=0
for _ in range(180):
 p=r/'guard_status.json';assert time.time()-p.stat().st_mtime<1.2
 s=json.loads(p.read_text());assert not s['fault'],s['reason']
 ts=s['temperatures'][22:24];taus=s['measured_tau'][22:24]
 rec=dict(utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),temperatures=ts,tau=taus,seq=s['seq'])
 with (r/'cooling.jsonl').open('a') as f:f.write(json.dumps(rec)+'\n')
 print(json.dumps(rec),flush=True)
 ready=ready+1 if max(t[0] for t in ts)<=70 and max(t[1] for t in ts)<=75 else 0
 if ready>=3:
  (r/'cooled_ready.json').write_text(json.dumps(rec,indent=2));print('COOLED_READY',flush=True);break
 time.sleep(10)
else:print('COOLING_NOT_READY_WITHIN_MONITOR_WINDOW',flush=True)
