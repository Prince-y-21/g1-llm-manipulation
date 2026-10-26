from pathlib import Path
import subprocess, shlex, concurrent.futures, json, os
old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
r.mkdir(exist_ok=False)
(r/'camera.cpp').write_text((old/'camera.cpp').read_text().replace(str(old),str(r)))
def camera():
 args=['g++','-std=c++17','-O2','-I/opt/ros/noetic/include',str(r/'camera.cpp'),'-L/opt/ros/noetic/lib/aarch64-linux-gnu','-Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu','-lrealsense2']+shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],universal_newlines=True))+['-o',str(r/'camera')]
 subprocess.check_call(args)
 return subprocess.check_output([str(r/'camera')],timeout=25,universal_newlines=True)
def observe():
 return subprocess.check_output([str(old/'observe'),str(r/'initial_state.json')],timeout=12,universal_newlines=True)
def permissions():
 p=subprocess.run(['sudo','-n','setfacl','-m','u:unitree:rw','/dev/ttyUSB1','/dev/ttyUSB2'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
 return dict(exit=p.returncode,error=p.stderr,rw=os.access('/dev/ttyUSB1',os.R_OK|os.W_OK))
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
 futures={k:pool.submit(f) for k,f in [('camera',camera),('observe',observe),('permissions',permissions)]}
 for k,f in futures.items():
  try: print(k,f.result(),flush=True)
  except Exception as e:print(k,'ERROR',str(e),flush=True)
print('STATE', (r/'initial_state.json').read_text(),flush=True)
