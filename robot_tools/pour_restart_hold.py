"""Fresh takeover from passive feedback; archive all previous command files."""
from pathlib import Path
import json,time,subprocess,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
for token in ['pour_guard','pour_right_gripper','pour_mission.py']:
 for proc in Path('/proc').glob('[0-9]*'):
  try: cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
  except (OSError,UnicodeError): continue
  assert token not in cmd,('Existing controller',proc,cmd)
s=json.loads((r/'restart_passive.json').read_text());assert s['command_count']==0 and s['mode_machine']==5 and s['mode_pr']==0
assert s['soc']>30 and all(j['motorstate']==0 for j in s['joints'])
archive=r/('before_fresh_hold_'+str(time.time_ns()));archive.mkdir()
for name in ['arm_target.txt','pulse_target.txt','release_request','stop_guard','pause_request','guard.pid','guard_status.json','recenter_request.txt','handoff_unwind.txt','gripper_target.txt','gripper.pid','gripper_status.json']:
 if (r/name).exists():(r/name).rename(archive/name)
assert os.access('/dev/ttyUSB1',os.R_OK|os.W_OK)
p=subprocess.Popen([str(r/'pour_right_gripper')],stdout=(r/('gripper_'+str(time.time_ns())+'.log')).open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'gripper.pid').write_text(str(p.pid));time.sleep(1);assert p.poll() is None
p=subprocess.Popen([str(r/'pour_guard_waist'),'--execute-suspended'],stdout=(r/('guard_fresh_'+str(time.time_ns())+'.log')).open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'guard.pid').write_text(str(p.pid));time.sleep(12);assert p.poll() is None
s=json.loads((r/'guard_status.json').read_text());assert not s['fault'];print('HOLD_READY',s['q'][22:29],s['temperatures'][22:24],flush=True)
