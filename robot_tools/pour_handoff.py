from pathlib import Path
import subprocess,json,time,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
plan=json.loads((r/'escape_plan.json').read_text());assert plan['final_clearance']>.03
oldpid=int((r/'guard.pid').read_text())
(r/'guard_status_before_recovery.json').write_text((r/'guard_status.json').read_text())
p=subprocess.Popen([str(r/'pour_guard_pulse'),'--handoff-suspended-recenter'],stdout=(r/'guard_recovery.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(4)
print('RECOVERY_PID',p.pid,'exit',p.poll());print((r/'guard_recovery.log').read_text());assert p.poll() is None
s=json.loads((r/'guard_status.json').read_text());assert not s['fault']
oldproc=Path('/proc')/str(oldpid);assert not oldproc.exists() or not (oldproc/'cmdline').read_bytes(),'Old publisher still active'
(r/'guard_original.pid').write_text(str(oldpid));(r/'guard.pid').write_text(str(p.pid))
print('HANDOFF_EXIT_VERIFIED',oldpid,'new_seq',s['seq'],'qright',s['q'][22:29])
