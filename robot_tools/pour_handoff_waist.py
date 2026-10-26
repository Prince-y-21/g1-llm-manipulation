from pathlib import Path
import subprocess,json,time
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');s=json.loads((r/'guard_status.json').read_text());assert not s['moving'] and not s['fault']
oldpid=int((r/'guard.pid').read_text());(r/'guard_before_waist.json').write_text(json.dumps(s,indent=2))
p=subprocess.Popen([str(r/'pour_guard_waist'),'--handoff-suspended-recenter'],stdout=(r/'guard_waist.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(4);print('WAIST_HANDOFF_PID',p.pid,'exit',p.poll());print((r/'guard_waist.log').read_text());assert p.poll() is None
oldproc=Path('/proc')/str(oldpid);assert not oldproc.exists() or not (oldproc/'cmdline').read_bytes()
(r/'guard_pre_waist.pid').write_text(str(oldpid));(r/'guard.pid').write_text(str(p.pid));s=json.loads((r/'guard_status.json').read_text());assert not s['fault'];print('WAIST_RAMP',s['waist_gains'],'temperature',s['temperatures'][22])
