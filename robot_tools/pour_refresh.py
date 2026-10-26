from pathlib import Path
import subprocess,time,shutil,json
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());archive=r/'scene_versions'/stamp;archive.mkdir(parents=True)
for name in ['initial_state.json','scene_estimate.json','head_color.jpg','head_depth.png','route_left_ready.json','route_candidate.json']:
 if (r/name).exists():shutil.copy2(str(r/name),str(archive/name))
subprocess.check_call([str(old/'observe'),str(r/'initial_state.json')])
s=(r/'pour_scene.py').read_text() if (r/'pour_scene.py').exists() else ''
print('CURRENT_STATE',(r/'initial_state.json').read_text())
subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'pour_probe.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'pour_probe')])
for id in [0,1]:
 p=subprocess.run([str(r/'pour_probe'),'/dev/ttyUSB1',str(id)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=6)
 print('RIGHT_LINK',p.stdout,flush=True)
