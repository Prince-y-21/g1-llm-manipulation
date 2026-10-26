from pathlib import Path
import subprocess,time,json,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
assert os.access('/dev/ttyUSB2',os.R_OK|os.W_OK)
assert not (r/'gripper_target.txt').exists()
if (r/'gripper.pid').exists():
 oldpid=int((r/'gripper.pid').read_text());pdir=Path('/proc')/str(oldpid)
 assert not pdir.exists() or 'pour_left_gripper' not in (pdir/'cmdline').read_text().replace('\0',' '),'Existing gripper process'
 (r/'gripper.pid').rename(r/'gripper_first_attempt.pid')
 (r/'gripper_left.log').rename(r/'gripper_left_first_attempt.log')
p=subprocess.Popen([str(r/'pour_left_gripper')],stdout=(r/'gripper_left.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'gripper.pid').write_text(str(p.pid))
time.sleep(1)
print('GRIPPER_PID',p.pid,'exit',p.poll())
print((r/'gripper_left.log').read_text())
if (r/'gripper_status.json').exists():print((r/'gripper_status.json').read_text())
