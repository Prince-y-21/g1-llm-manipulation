from pathlib import Path
import subprocess
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'pour_probe.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'pour_probe')])
for port,id in [('/dev/ttyUSB2',1),('/dev/ttyUSB2',0),('/dev/ttyUSB1',0),('/dev/ttyUSB1',1)]:
 p=subprocess.run([str(r/'pour_probe'),port,str(id)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=6)
 print(p.stdout,flush=True)
