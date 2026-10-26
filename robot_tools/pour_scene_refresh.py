from pathlib import Path
import subprocess,os,signal,json,time
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
pid=int((r/'gripper.pid').read_text());proc=Path('/proc')/str(pid)
if proc.exists() and 'pour_left_gripper' in (proc/'cmdline').read_text():
 os.kill(pid,signal.SIGTERM)
 for _ in range(30):
  if not (proc/'cmdline').exists() or not (proc/'cmdline').read_bytes():break
  time.sleep(.1)
 assert not proc.exists() or not (proc/'cmdline').read_bytes(),'Left gripper did not exit'
print('LEFT_GRIPPER_EXIT_CONFIRMED')
for name in ['gripper.pid','gripper_target.txt','gripper_status.json']:
 p=r/name
 if p.exists():p.rename(r/(name+'.left_session'))
s=(r/'pour_scene.py').read_text()
for a,b in [("364,223,4","480,207,4"),("363,242,4","480,226,4"),("364,303,8","480,285,8"),("363,374,4","476,350,4"),("143,254,3","270,250,3"),("145,336,3","273,325,3"),("89,289,3","219,281,3"),("196,291,3","323,282,3"),("145,293,10","270,285,10"),("intersect(363,374)","intersect(476,350)"),("intersect(150,403)","intersect(275,390)"),("[(80,477),(300,456),(600,400)]","[(80,477),(300,430),(600,365)]")]:s=s.replace(a,b)
(r/'pour_scene_current.py').write_text(s)
subprocess.check_call(['python3',str(r/'pour_scene_current.py')])
