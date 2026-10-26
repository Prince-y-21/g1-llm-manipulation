from pathlib import Path
import subprocess
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');s=(r/'pour_guard_pulse.cpp').read_text()
a='float shoulder_kp_start=kp[22],shoulder_kd_start=kd[22];';assert a in s;s=s.replace(a,a+'float waist_kp_start=40.f,waist_kd_start=1.f;')
a=':(m.kp()==kp[i]&&m.kd()==kd[i]);';assert a in s;s=s.replace(a,':(i==14?(m.kp()>=40&&m.kp()<=80&&m.kd()>=1&&m.kd()<=2):(m.kp()==kp[i]&&m.kd()==kd[i]));')
a='if(i==22){shoulder_kp_start=m.kp();shoulder_kd_start=m.kd();}';assert a in s;s=s.replace(a,a+'if(i==14){waist_kp_start=m.kp();waist_kd_start=m.kd();}')
a=':kp[i];m.kd()=';assert a in s;s=s.replace(a,':(i==14?waist_kp_start+(80-waist_kp_start)*gain_phase:kp[i]);m.kd()=')
a=':kd[i];m.tau()=';assert a in s;s=s.replace(a,':(i==14?waist_kd_start+(2-waist_kd_start)*gain_phase:kd[i]);m.tau()=')
# Report the changing gain without changing any guard thresholds.
a='   f<<"],\\\"pulsing\\\":"';assert a in s
s=s.replace(a,'   f<<"],\\\"waist_gains\\\":["<<(waist_kp_start+(80-waist_kp_start)*gain_phase)<<","<<(waist_kd_start+(2-waist_kd_start)*gain_phase)<<"],\\\"pulsing\\\":"')
(r/'pour_guard_waist.cpp').write_text(s)
subprocess.check_call(['g++','-std=c++17','-O2','-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/'pour_guard_waist.cpp'),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/'pour_guard_waist')])
print('WAIST_GAIN_VARIANT_BUILT_NOT_STARTED')
