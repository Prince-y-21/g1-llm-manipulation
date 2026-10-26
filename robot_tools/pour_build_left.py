from pathlib import Path
import subprocess, concurrent.futures
old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016');r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
s=(old/'task_guard_fast.cpp').read_text().replace(str(old),str(r))
for a,b in [('kp[22]','kp[15]'),('kd[22]','kd[15]'),('i==22','i==15'),('j+22','j+15'),('i=22;i<29','i=15;i<22'),('for(int i:{22,25})','for(int i:{15,18})'),('q22 q23 q24 q25 q26 q27 q28','q15 q16 q17 q18 q19 q20 q21'),('-2.31f,-1.91f,-2.40f','-2.31f,-.80f,-2.40f'),('.7f,.8f,2.40f','.7f,1.91f,2.40f')]:s=s.replace(a,b)
(r/'pour_left_guard.cpp').write_text(s)
s=(old/'task_gripper_wide.cpp').read_text().replace(str(old),str(r)).replace('DEVICE_ID_02-if01','DEVICE_ID_02-if02').replace('c.id=0','c.id=1').replace('RIGHT_GRIPPER_HOLD','LEFT_GRIPPER_HOLD')
s=s.replace('SerialPort serial(port,16,6000000);','SerialPort serial(port,16,6000000);\n std::this_thread::sleep_for(std::chrono::milliseconds(100));')
s=s.replace('MotorData d;d.motorType=c.motorType;','MotorData d;d.motorType=c.motorType;d.correct=false;d.merror=0;d.temp=0;')
(r/'pour_left_gripper.cpp').write_text(s)
def build(name):
 if name=='pour_left_gripper':args=['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/(name+'.cpp')),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/name)]
 else:args=['g++','-std=c++17','-O2','-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/(name+'.cpp')),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/name)]
 subprocess.check_call(args);return name+' BUILT_NOT_STARTED'
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as e:
 for result in e.map(build,['pour_left_guard','pour_left_gripper']):print(result,flush=True)
