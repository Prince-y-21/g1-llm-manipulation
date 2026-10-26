from pathlib import Path
import cv2,subprocess,time,json,os
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');src=r.parent/'gripper_small_cycle.cpp';assert os.access('/dev/ttyUSB2',os.R_OK|os.W_OK)
busy=subprocess.run(['fuser','/dev/ttyUSB2'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True);assert not busy.stdout.strip()
subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(src),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'gripper_small_cycle')],timeout=25)
cam=cv2.VideoCapture(0,cv2.CAP_V4L2);cam.set(cv2.CAP_PROP_FRAME_WIDTH,640);cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480);assert cam.isOpened()
for _ in range(5):ok,im=cam.read()
assert ok;cv2.imwrite(str(r/'left_cycle_before.jpg'),im)
p=subprocess.Popen([str(r/'gripper_small_cycle'),str(r)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
start=time.monotonic();frames=[]
while p.poll() is None and time.monotonic()-start<12:
 ok,im=cam.read();elapsed=time.monotonic()-start
 if ok and (not frames or elapsed-frames[-1]['elapsed_s']>.18):
  name='left_cycle_'+str(len(frames)).zfill(3)+'.jpg';cv2.imwrite(str(r/name),im);frames.append(dict(elapsed_s=elapsed,file=name))
if p.poll() is None:p.terminate()
output=p.communicate(timeout=3)[0];cam.release();(r/'left_cycle_frames.json').write_text(json.dumps(frames,indent=2));(r/'left_cycle.log').write_text(output);print(output,flush=True);print('CYCLE_EXIT',p.returncode,'FRAME_COUNT',len(frames),flush=True)
assert p.returncode==0,'Bounded empty-gripper cycle did not verify; no further motion authorized by this script'
