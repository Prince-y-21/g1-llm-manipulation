"""Bounded actual-position response check, using measured empty-hand stiction evidence."""
from pathlib import Path
import cv2,subprocess,time,json
base=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z');source=(base/'gripper_small_cycle.cpp').read_text();rbase=base/'bimanual_development'
for side,port,id,camindex,sign in [('left','/dev/ttyUSB2',1,0,1),('right','/dev/ttyUSB1',0,8,-1)]:
 r=rbase/(side+'_bounded_servo');r.mkdir(exist_ok=False)
 busy=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True);assert not busy.stdout.strip()
 s=source.replace('elapsed>8.3f','elapsed>14.1f').replace('elapsed<3.5f?(elapsed-.5f)/3.f:(elapsed<4.3f?1.f:(elapsed<7.3f?1.f-(elapsed-4.3f)/3.f:0.f))','elapsed<6.5f?(elapsed-.5f)/6.f:(elapsed<7.1f?1.f:(elapsed<13.1f?1.f-(elapsed-7.1f)/6.f:0.f))')
 assert 'elapsed<6.5f' in s
 # Effective position slope 15 Nm/rad inside a fixed .4 Nm P-term envelope.
 # This reduces measured ~.06-rad static error rather than loosening the endpoint test.
 s=s.replace('std::clamp(target,q-.04f,q+.04f)','(q+std::clamp(3.f*(target-q),-.08f,.08f))').replace('c.kd=.05f/(g*g)','c.kd=.5f/(g*g)').replace('std::abs(torque)>.30f','std::abs(torque)>.60f')
 if sign>0:s=s.replace('initial-.15f*phase','initial+.15f*phase').replace('q<initial-.19f||q>initial+.04f','q<initial-.04f||q>initial+.19f').replace('initial-minimum<.08f','maximum-initial<.08f')
 if side=='right':s=s.replace('/dev/ttyUSB2','/dev/ttyUSB1').replace('c.id=1','c.id=0').replace('initial<-.05||initial>.25','initial<5.0||initial>5.4')
 (r/'cycle.cpp').write_text(s)
 subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'empty_cycle')],timeout=20)
 cam=cv2.VideoCapture(camindex,cv2.CAP_V4L2);assert cam.isOpened()
 for _ in range(5):ok,im=cam.read()
 assert ok;cv2.imwrite(str(r/'before.jpg'),im)
 p=subprocess.Popen([str(r/'empty_cycle'),str(r)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True);start=time.monotonic();frames=[]
 while p.poll() is None and time.monotonic()-start<18:
  ok,im=cam.read();elapsed=time.monotonic()-start
  if ok and (not frames or elapsed-frames[-1]['elapsed_s']>.18):
   file='frame_'+str(len(frames)).zfill(3)+'.jpg';cv2.imwrite(str(r/file),im);frames.append(dict(elapsed_s=elapsed,file=file))
 if p.poll() is None:p.terminate()
 output=p.communicate(timeout=3)[0];cam.release();(r/'frames.json').write_text(json.dumps(frames,indent=2));(r/'cycle.log').write_text(output);print(side,output,'EXIT',p.returncode,flush=True)
 assert p.returncode==0,'Empty position-response check failed; stop without contact'
