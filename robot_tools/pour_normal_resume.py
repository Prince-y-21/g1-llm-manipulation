from pathlib import Path
import subprocess,time,json
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');s=json.loads((r/'guard_status.json').read_text());assert not s['moving']
assert all(t[0]<85 and t[1]<120 for t in s['temperatures'])
stamp=str(time.time_ns());(r/('before_normal_resume_'+stamp+'.json')).write_text(json.dumps(s,indent=2));oldpid=int((r/'guard.pid').read_text())
log=r/('guard_normal_'+stamp+'.log');p=subprocess.Popen([str(r/'pour_guard_waist'),'--handoff-suspended-recenter'],stdout=log.open('w'),stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(4);print('NORMAL_HANDOFF',p.pid,'exit',p.poll(),'log',str(log),flush=True);assert p.poll() is None,log.read_text()
op=Path('/proc')/str(oldpid);assert not op.exists() or not (op/'cmdline').read_bytes();(r/'guard.pid').write_text(str(p.pid));s=json.loads((r/'guard_status.json').read_text());assert not s['fault'];print('ORDINARY_THERMAL_GUARDS_ACTIVE',s['temperatures'][22:24],flush=True)
