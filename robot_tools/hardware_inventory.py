import os,json,subprocess,time
from pathlib import Path
out={'captured_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'scope':'Read-only inventory after user-reported battery depletion and reboot; no motor commands, mode changes or camera streaming.'}
def run(name,args):
 try:
  p=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=15)
  out[name]={'exit_code':p.returncode,'output':p.stdout.strip()}
 except Exception as e:out[name]={'error':str(e)}
for name,path in [('device_tree_model','/proc/device-tree/model'),('os_release','/etc/os-release'),('l4t_release','/etc/nv_tegra_release'),('boot_id','/proc/sys/kernel/random/boot_id')]:
 try:out[name]=Path(path).read_text().strip('\x00\n')
 except Exception as e:out[name]={'error':str(e)}
for name,args in [('kernel',['uname','-a']),('cpu',['lscpu']),('memory',['free','-h']),('storage',['df','-h','/home/unitree','/tmp']),('usb',['lsusb']),('network',['ip','-br','addr']),('video_by_id',['ls','-l','/dev/v4l/by-id/']),('serial_by_id',['ls','-l','/dev/serial/by-id/']),('gripper_service',['systemctl','cat','dex1_gripper.service']),('gripper_service_active',['systemctl','is-active','dex1_gripper.service']),('gripper_service_enabled',['systemctl','is-enabled','dex1_gripper.service']),('temporary_experiment_dir',['ls','-ld','/tmp/codex-g1-observe'])]:run(name,args)
out['video_nodes']=[]
for p in sorted(Path('/sys/class/video4linux').glob('video*')):
 try:out['video_nodes'].append({'node':'/dev/'+p.name,'name':(p/'name').read_text().strip(),'sysfs_device':str((p/'device').resolve())})
 except Exception as e:out['video_nodes'].append({'node':p.name,'error':str(e)})
for dev in ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/video0','/dev/video8']:
 run('udev_'+Path(dev).name,['udevadm','info','--query=property','--name='+dev])
run('serial_acl',['getfacl','-p','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3'])
run('python_dependencies',['python3','-c',"import sys,cv2,numpy,scipy,importlib.util,json;print(json.dumps({'python':sys.version,'opencv':cv2.__version__,'numpy':numpy.__version__,'scipy':scipy.__version__,'pyrealsense2_available':importlib.util.find_spec('pyrealsense2') is not None,'pinocchio_available':importlib.util.find_spec('pinocchio') is not None,'unitree_sdk2py_available':importlib.util.find_spec('unitree_sdk2py') is not None}))"])
run('toolchain',['g++','--version'])
run('camera_binary_dependencies',['ldd','/tmp/codex-g1-observe/camera'])
run('gripper_binary_dependencies',['ldd','/tmp/codex-g1-observe/gripper_guard_v2'])
out['serial_access']={dev:{'read':os.access(dev,os.R_OK),'write':os.access(dev,os.W_OK)} for dev in ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']}
out['our_controllers']=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  exe=Path(os.readlink(str(p/'exe')))
  if exe.parent==Path('/tmp/codex-g1-observe') and (exe.name.startswith('lowlevel_guard') or exe.name.startswith('gripper_guard')):out['our_controllers'].append({'pid':int(p.name),'exe':str(exe)})
 except (OSError,PermissionError):pass
print(json.dumps(out,ensure_ascii=False,indent=2))
