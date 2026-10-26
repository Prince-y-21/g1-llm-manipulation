from pathlib import Path
import subprocess,json
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=root/'left_fullstroke_080';r.mkdir(exist_ok=False);s=(root/'left_force_step_080/cycle.cpp').read_text()
changes=[('elapsed>14.5f','elapsed>64.5f'),('elapsed<6.5f?(elapsed-.5f)/6.f:(elapsed<7.5f?1.f:(elapsed<13.5f?1.f-(elapsed-7.5f)/6.f:0.f))','elapsed<30.5f?(elapsed-.5f)/30.f:(elapsed<32.5f?1.f:(elapsed<62.5f?1.f-(elapsed-32.5f)/30.f:0.f))'),('initial+.35f*phase','initial+3.0f*phase'),('q>initial+.39f','q>initial+3.04f'),('maximum-initial<.28f','maximum-initial<2.9f')]
for a,b in changes:assert a in s,a;s=s.replace(a,b)
(r/'cycle.cpp').write_text(s);subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'cycle')],timeout=25)
runner=(root/'cap_left_force080.py').read_text().replace("r=root/'left_force_step_080'","r=root/'left_fullstroke_080'").replace('start<17','start<69');(root/'cap_left_fullstroke.py').write_text(runner)
(r/'preflight.json').write_text(json.dumps(dict(scope='Required task opening identification in outward empty space; return before body leaves outward hold',maximum_target_change_rad=3.,p_cap_nm=.8,kd=3.,speed_guard_rad_s=.25,torque_guard_nm=1.,stall_abort_s=.8,body_hold_s=75,evidence='Same settings passed small reversible opening; no object-contact force permission inferred'),indent=2));print('FULLSTROKE_PREPARED_NO_COMMANDS')
