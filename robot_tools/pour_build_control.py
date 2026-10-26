from pathlib import Path
import subprocess, concurrent.futures
old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016');r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
for src,dst in [('task_guard_fast.cpp','pour_guard.cpp'),('task_gripper_wide.cpp','pour_gripper.cpp')]:
 (r/dst).write_text((old/src).read_text().replace(str(old),str(r)))
def build(name):
 if name=='pour_gripper':
  args=['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/(name+'.cpp')),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/name)]
 else:
  args=['g++','-std=c++17','-O2','-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/(name+'.cpp')),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/name)]
 subprocess.check_call(args);return name+' BUILT_NO_START'
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as e:
 for result in e.map(build,['pour_guard','pour_gripper']):print(result,flush=True)
