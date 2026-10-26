"""New-run inventory and camera/passive DDS capture. No motor commands."""
from pathlib import Path
import time,json,os,subprocess,concurrent.futures,hashlib,glob,shlex
stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
r=Path('/home/unitree/codex_g1_tasks')/('cap_open_'+stamp);r.mkdir(exist_ok=False)
old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
camera_source=(old/'camera.cpp').read_text()
assert str(old) in camera_source
camera_source=camera_source.replace(str(old),str(r))
camera_source=camera_source.replace('rs2::config config;','rs2::config config; config.enable_device("DEVICE_ID_01");')
(r/'camera.cpp').write_text(camera_source)
manifest=dict(run_id=r.name,root=str(r),started_utc=stamp,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),hostname=os.uname().nodename,mode='observation_only_no_motor_publisher',serials={p:dict(real=os.path.realpath(p),rw=os.access(p,os.R_OK|os.W_OK)) for p in glob.glob('/dev/serial/by-id/DEVICE_TO_CONFIGURE')},cameras={p:os.path.realpath(p) for p in glob.glob('/dev/v4l/by-id/*')})
manifest['passive_binary_sha256']=hashlib.sha256((old/'observe').read_bytes()).hexdigest()
(r/'manifest.json').write_text(json.dumps(manifest,indent=2));print('RUN_ROOT',r,flush=True)
def capture():
 args=['g++','-std=c++17','-O2','-I/opt/ros/noetic/include',str(r/'camera.cpp'),'-L/opt/ros/noetic/lib/aarch64-linux-gnu','-Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu','-lrealsense2']+shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],universal_newlines=True))+['-o',str(r/'camera')]
 subprocess.check_call(args,timeout=25)
 return subprocess.check_output([str(r/'camera')],timeout=25,universal_newlines=True)
def observe():
 return subprocess.check_output([str(old/'observe'),str(r/'initial_state.json')],timeout=12,universal_newlines=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 futures={k:pool.submit(f) for k,f in [('camera',capture),('observe',observe)]}
 for k,f in futures.items():
  try:print(k,f.result(),flush=True)
  except Exception as e:manifest[k+'_error']=str(e);print(k,'FAILED',str(e),flush=True)
manifest['finished_utc']=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());(r/'manifest.json').write_text(json.dumps(manifest,indent=2))
print('STATE', (r/'initial_state.json').read_text(),flush=True)
