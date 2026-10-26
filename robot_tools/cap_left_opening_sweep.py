"""Empty, low-arm opening identification. No body publisher, no object contact."""
from pathlib import Path
import subprocess,time,json,cv2
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=root/'left_opening_sweep_02';r.mkdir(exist_ok=False)
s=(root/'left_bounded_servo/cycle.cpp').read_text()
changes=[('elapsed>14.1f','elapsed>84.5f'),
('elapsed<6.5f?(elapsed-.5f)/6.f:(elapsed<7.1f?1.f:(elapsed<13.1f?1.f-(elapsed-7.1f)/6.f:0.f))','elapsed<40.5f?(elapsed-.5f)/40.f:(elapsed<42.5f?1.f:(elapsed<82.5f?1.f-(elapsed-42.5f)/40.f:0.f))'),
('initial+.15f*phase','initial+2.0f*phase'),('initial>.25','initial>.4'),
('c.kd=.5f/(g*g)','c.kd=1.5f/(g*g)'),
('q>initial+.19f','q>initial+2.04f'),('maximum-initial<.08f','maximum-initial<1.8f')]
for a,b in changes:assert a in s,a;s=s.replace(a,b)
(r/'cycle.cpp').write_text(s)
p=subprocess.run(['fuser','/dev/ttyUSB2'],stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert not p.stdout.strip()
subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'empty_cycle')],timeout=25)
cam=cv2.VideoCapture(0,cv2.CAP_V4L2);assert cam.isOpened();cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
for _ in range(5):ok,im=cam.read()
assert ok;cv2.imwrite(str(r/'before.jpg'),im)
with (r/'cycle.log').open('w') as log:
    p=subprocess.Popen([str(r/'empty_cycle'),str(r)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);start=time.monotonic();frames=[]
    while p.poll() is None and time.monotonic()-start<90:
        ok,im=cam.read();elapsed=time.monotonic()-start
        if ok and (not frames or elapsed-frames[-1]['elapsed_s']>.3):
            file='frame_'+str(len(frames)).zfill(3)+'.jpg';cv2.imwrite(str(r/file),im);frames.append(dict(elapsed_s=elapsed,file=file))
    if p.poll() is None:p.terminate()
    code=p.wait(timeout=5)
cam.release();(r/'frames.json').write_text(json.dumps(frames,indent=2));print((r/'cycle.log').read_text(),'EXIT',code,flush=True)
assert code==0,'No object approach after unverified opening identification'
