"""Final observation and ownership evidence. Never starts any motor controller."""
from pathlib import Path
import subprocess,concurrent.futures,shlex,json,time
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z');p=r/'camera.cpp';src=p.read_text()
src=src.replace('meta<<"{', 'meta.precision(17);meta<<"{')
p.write_text(src)
args=['g++','-std=c++17','-O2','-I/opt/ros/noetic/include',str(p),'-L/opt/ros/noetic/lib/aarch64-linux-gnu','-Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu','-lrealsense2']+shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],universal_newlines=True))+['-o',str(r/'camera')]
subprocess.check_call(args,timeout=25)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 fs=[pool.submit(subprocess.check_output,[str(r/'camera')],timeout=25,universal_newlines=True),pool.submit(subprocess.check_output,['/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/observe',str(r/'final_state.json')],timeout=12,universal_newlines=True)]
 for f in fs:print(f.result(),flush=True)
processes=[]
for d in Path('/proc').glob('[0-9]*'):
 try:cmd=(d/'cmdline').read_bytes().replace(b'\0',b' ').decode()
 except (OSError,UnicodeError):continue
 if any(x in cmd for x in ['pour_guard','pour_right_gripper','pour_left_gripper','pour_mission.py']):processes.append(dict(pid=int(d.name),cmd=cmd))
service=subprocess.check_output(['systemctl','show','dex1_gripper.service','-p','ActiveState','-p','SubState','-p','MainPID','-p','NRestarts','-p','ExecMainStatus'],universal_newlines=True)
state=json.loads((r/'final_state.json').read_text());out=dict(server_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),owned_motor_controllers_started=False,matching_legacy_controller_processes=processes,preexisting_gripper_service=service,body_lowcmd_count=state['command_count'],soc=state['soc'],mode_machine=state['mode_machine'],mode_pr=state['mode_pr'],cap_opening_executed=False,reason='Bimanual coordination, local gripper geometry/calibration and torsional/axial contact verification unavailable; no actuation attempted')
(r/'final_check.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2),flush=True)
