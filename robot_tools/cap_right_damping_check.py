from pathlib import Path
import subprocess,time,json,cv2
base=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=base/'right_damping_check';r.mkdir(exist_ok=False)
s=(base/'right_bounded_servo/cycle.cpp').read_text();assert 'c.kd=.5f/(g*g)' in s;s=s.replace('c.kd=.5f/(g*g)','c.kd=1.5f/(g*g)');(r/'cycle.cpp').write_text(s)
busy=subprocess.run(['fuser','/dev/ttyUSB1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True);assert not busy.stdout.strip()
subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'empty_cycle')],timeout=20)
cam=cv2.VideoCapture(8,cv2.CAP_V4L2);assert cam.isOpened()
for _ in range(5):ok,im=cam.read()
assert ok;cv2.imwrite(str(r/'before.jpg'),im)
p=subprocess.Popen([str(r/'empty_cycle'),str(r)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True);start=time.monotonic();frames=[]
while p.poll() is None and time.monotonic()-start<18:
 ok,im=cam.read();elapsed=time.monotonic()-start
 if ok and (not frames or elapsed-frames[-1]['elapsed_s']>.18):
  file='frame_'+str(len(frames)).zfill(3)+'.jpg';cv2.imwrite(str(r/file),im);frames.append(dict(elapsed_s=elapsed,file=file))
if p.poll() is None:p.terminate()
output=p.communicate(timeout=3)[0];cam.release();(r/'frames.json').write_text(json.dumps(frames,indent=2));(r/'cycle.log').write_text(output);print(output,'EXIT',p.returncode,flush=True)
assert p.returncode==0,'Right damping check failed; stop before contact'
