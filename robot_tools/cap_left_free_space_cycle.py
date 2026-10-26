"""Called only after finite body profile verifies outward empty holding phase."""
from pathlib import Path
import subprocess,cv2,time,json
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=root/'left_free_space_cycle';r.mkdir(exist_ok=False)
s=(root/'left_bounded_servo/cycle.cpp').read_text()
changes=[('elapsed>14.1f','elapsed>10.5f'),
('elapsed<6.5f?(elapsed-.5f)/6.f:(elapsed<7.1f?1.f:(elapsed<13.1f?1.f-(elapsed-7.1f)/6.f:0.f))','elapsed<4.5f?(elapsed-.5f)/4.f:(elapsed<5.5f?1.f:(elapsed<9.5f?1.f-(elapsed-5.5f)/4.f:0.f))'),
('initial+.15f*phase','initial+.5f*phase'),('initial>.25','initial>.4'),('c.kd=.5f/(g*g)','c.kd=1.5f/(g*g)'),
('q>initial+.19f','q>initial+.54f'),('maximum-initial<.08f','maximum-initial<.4f')]
for a,b in changes:assert a in s,a;s=s.replace(a,b)
(r/'cycle.cpp').write_text(s);p=subprocess.run(['fuser','/dev/ttyUSB2'],stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert not p.stdout.strip()
subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'cycle')],timeout=15)
cam=cv2.VideoCapture(0,cv2.CAP_V4L2);assert cam.isOpened();frames=[]
with (r/'cycle.log').open('w') as log:
    p=subprocess.Popen([str(r/'cycle'),str(r)],stdout=log,stderr=subprocess.STDOUT);start=time.monotonic()
    while p.poll() is None and time.monotonic()-start<13:
        ok,im=cam.read();elapsed=time.monotonic()-start
        if ok and (not frames or elapsed-frames[-1]['elapsed_s']>.2):
            name='frame_'+str(len(frames)).zfill(3)+'.jpg';cv2.imwrite(str(r/name),im);frames.append(dict(elapsed_s=elapsed,file=name))
    if p.poll() is None:p.terminate()
    code=p.wait(timeout=3)
cam.release();(r/'frames.json').write_text(json.dumps(frames,indent=2));(r/'result.json').write_text(json.dumps(dict(exit_code=code,source='same .4 Nm P cap as prior sweep; outward physical location changes',body_profile='left_free_opening')));print((r/'cycle.log').read_text(),'EXIT',code,flush=True)
