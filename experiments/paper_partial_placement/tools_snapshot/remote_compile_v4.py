"""Compile isolated reviewed sources. Dry-run subscribes only; no publisher or serial."""
from pathlib import Path
import subprocess,json,hashlib,re,shlex
R=Path('/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01')
D=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
assert not (R/'control').exists()
names=['paper_body_retry_v4.cpp','paper_gripper_v4.cpp','paper_cameras_v4.cpp','phase_guardian_retry_v4.py','retry_gate.hpp','retry_ack.hpp','retry_thermal_v4.hpp','bimanual_motion.hpp','retry_remote_v4.py','retry_evidence.py']
assert all((R/n).is_file() for n in names)
commands=[['g++','-std=c++17','-O2','-Wall','-Wextra','-I'+str(R),'-I'+str(D),'-I/usr/local/include','-I/usr/local/include/ddscxx',str(R/'paper_body_retry_v4.cpp'),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(R/'paper_body_retry_v4')],['g++','-std=c++17','-O2','-Wall','-Wextra','-I/home/unitree/dex1_1_service/include',str(R/'paper_gripper_v4.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(R/'paper_gripper_v4')]]
reference=Path('/home/unitree/codex_g1_tasks/paper_under_stool_20261021T122400Z/paper_cameras')
ldd=subprocess.check_output(['ldd',str(reference)],text=True);lib=Path(re.search(r'librealsense2\.so\S*\s+=>\s+(\S+)',ldd).group(1));assert lib.exists()
commands.append(['g++','-std=c++17','-O2','-Wall','-Wextra','-I/opt/ros/noetic/include',str(R/'paper_cameras_v4.cpp'),str(lib),'-Wl,-rpath,'+str(lib.parent),'-lpthread']+shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))+['-o',str(R/'paper_cameras_v4')])
for index,cmd in enumerate(commands):
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=60)
    (R/('compile_'+str(index)+'.log')).write_text(p.stdout+p.stderr)
    print(json.dumps(dict(index=index,returncode=p.returncode,output=str(R/('compile_'+str(index)+'.log')))),flush=True)
    if p.returncode:raise RuntimeError((p.stdout+p.stderr)[-4000:])
subprocess.check_call(['g++','-std=c++17','-O2',str(R/'test_thermal_v4.cpp'),'-o',str(R/'test_thermal_v4')],timeout=25)
subprocess.check_call([str(R/'test_thermal_v4')],timeout=5)
subprocess.check_call(['python3','-m','py_compile',str(R/'phase_guardian_retry_v4.py'),str(R/'retry_evidence.py'),str(R/'retry_remote_v4.py')])
(R/'dry_run').mkdir(exist_ok=False)
p=subprocess.run([str(R/'paper_body_retry_v4'),str(R/'dry_run'),'paper_resume_20261025_2041','--dry-run'],capture_output=True,text=True,timeout=12)
(R/'dry_run.json').write_text(json.dumps(dict(returncode=p.returncode,output=p.stdout+p.stderr),indent=2))
assert p.returncode==0,p.stdout+p.stderr
names+=['paper_body_retry_v4','paper_gripper_v4','paper_cameras_v4']
(R/'versions.json').write_text(json.dumps({n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in names},indent=2))
print('V4_COMPILED_AND_PASSIVE_DRY_RUN_OK',p.stdout,flush=True)
