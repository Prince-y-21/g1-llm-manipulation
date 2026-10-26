"""Single bounded empty-space breakaway check, after same-effort location test."""
from pathlib import Path
import subprocess,cv2,time,json,sys
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=root/'left_force_step_050'
if '--prepare' in sys.argv:
    r.mkdir(exist_ok=False);s=(root/'left_bounded_servo/cycle.cpp').read_text()
    changes=[('elapsed>14.1f','elapsed>14.5f'),('elapsed<6.5f?(elapsed-.5f)/6.f:(elapsed<7.1f?1.f:(elapsed<13.1f?1.f-(elapsed-7.1f)/6.f:0.f))','elapsed<6.5f?(elapsed-.5f)/6.f:(elapsed<7.5f?1.f:(elapsed<13.5f?1.f-(elapsed-7.5f)/6.f:0.f))'),('initial+.15f*phase','initial+.35f*phase'),('initial>.25','initial>.4'),('c.kd=.5f/(g*g)','c.kd=1.5f/(g*g)'),('-.08f,.08f','-.10f,.10f'),('std::abs(torque)>.60f','std::abs(torque)>.70f'),('q>initial+.19f','q>initial+.39f'),('maximum-initial<.08f','maximum-initial<.28f')]
    for a,b in changes:assert a in s,a;s=s.replace(a,b)
    s=s.replace('int result=0;float minimum','int result=0;float stall_since=-1;float minimum')
    anchor='  minimum=std::min(minimum,q);'
    s=s.replace(anchor,'''  if(target-q>.08f&&std::abs(velocity)<.015f&&torque>.42f){if(stall_since<0)stall_since=elapsed;else if(elapsed-stall_since>.8f){result=7;break;}}else stall_since=-1;
'''+anchor)
    (r/'cycle.cpp').write_text(s);subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'cycle')],timeout=25)
    (r/'preflight.json').write_text(json.dumps(dict(scope='empty outward space, no objects',evidence='same .4 Nm P cap stalled near .335 rad both at rest and with left arm outward .4 rad; healthy feedback, no motor error, <=35 C',manufacturer_reference='/home/unitree/dex1_1_service/test/test_gripper.cpp: output-side Kp=5 and q range .5..5.5; not used as unrestricted effort permission',p_cap_nm=.5,prior_p_cap_nm=.4,output_kd=1.5,measured_torque_guard_nm=.7,speed_guard_rad_s=.25,max_target_change_rad=.35,stall_abort_s=.8),indent=2));print('EMPTY_FORCE_STEP_PREPARED_NO_MOTOR_COMMANDS');sys.exit(0)
assert (r/'cycle').exists() and not (r/'cycle.log').exists()
p=subprocess.run(['fuser','/dev/ttyUSB2'],stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert not p.stdout.strip();cam=cv2.VideoCapture(0,cv2.CAP_V4L2);assert cam.isOpened();frames=[]
with (r/'cycle.log').open('w') as log:
    p=subprocess.Popen([str(r/'cycle'),str(r)],stdout=log,stderr=subprocess.STDOUT);start=time.monotonic()
    while p.poll() is None and time.monotonic()-start<17:
        ok,im=cam.read();elapsed=time.monotonic()-start
        if ok and (not frames or elapsed-frames[-1]['elapsed_s']>.2):
            name='frame_'+str(len(frames)).zfill(3)+'.jpg';cv2.imwrite(str(r/name),im);frames.append(dict(elapsed_s=elapsed,file=name))
    if p.poll() is None:p.terminate()
    code=p.wait(timeout=3)
cam.release();(r/'frames.json').write_text(json.dumps(frames,indent=2));(r/'result.json').write_text(json.dumps(dict(exit_code=code)));print((r/'cycle.log').read_text(),'EXIT',code,flush=True)
