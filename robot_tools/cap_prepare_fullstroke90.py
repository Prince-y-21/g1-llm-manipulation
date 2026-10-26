from pathlib import Path
import subprocess,json
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=root/'left_fullstroke90_080';r.mkdir(exist_ok=False);s=(root/'left_fullstroke_080/cycle.cpp').read_text()
changes=[('elapsed>64.5f','elapsed>104.5f'),('elapsed<30.5f?(elapsed-.5f)/30.f:(elapsed<32.5f?1.f:(elapsed<62.5f?1.f-(elapsed-32.5f)/30.f:0.f))','elapsed<50.5f?(elapsed-.5f)/50.f:(elapsed<52.5f?1.f:(elapsed<102.5f?1.f-(elapsed-52.5f)/50.f:0.f))'),('initial+3.0f*phase','initial+4.6f*phase'),('q>initial+3.04f','q>initial+4.64f'),('maximum-initial<2.9f','maximum-initial<4.45f')]
for a,b in changes:assert a in s,a;s=s.replace(a,b)
(r/'cycle.cpp').write_text(s);subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'cycle')],timeout=25)
runner=(root/'cap_left_force080.py').read_text().replace("r=root/'left_force_step_080'","r=root/'left_fullstroke90_080'").replace('start<17','start<110');(root/'cap_left_fullstroke90.py').write_text(runner)
(r/'preflight.json').write_text(json.dumps(dict(scope='Empty required wide opening and return in outward space; original 3-rad test never started due observer preflight failure',delta_q_rad=4.6,p_cap_nm=.8,kd=3.,speed_guard_rad_s=.25,torque_guard_nm=1.,stall_abort_s=.8,body_hold_s=115),indent=2));print('WIDE_OPENING_PREPARED_NO_COMMANDS')
