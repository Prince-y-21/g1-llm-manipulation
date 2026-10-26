from pathlib import Path
import subprocess,time,os,json
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
assert os.access('/dev/ttyUSB1',os.R_OK|os.W_OK)
assert not (r/'gripper_target.txt').exists() and not (r/'gripper.pid').exists()
p=subprocess.Popen([str(r/'pour_right_gripper')],stdout=(r/'gripper_right.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(r/'gripper.pid').write_text(str(p.pid));time.sleep(1)
print('RIGHT_GRIPPER_PID',p.pid,'exit',p.poll());print((r/'gripper_right.log').read_text())
assert p.poll() is None,'Gripper failed init'
print((r/'gripper_status.json').read_text())
