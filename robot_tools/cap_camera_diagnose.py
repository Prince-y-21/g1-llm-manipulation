from pathlib import Path
import subprocess,sys
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z')
src=r/'rs_probe.cpp';src.write_text('''#include <librealsense2/rs.hpp>
#include <iostream>
int main(){try{rs2::log_to_console(RS2_LOG_SEVERITY_WARN);rs2::context c;auto d=c.query_devices();std::cout<<"DEVICE_COUNT "<<d.size()<<std::endl;for(auto x:d){for(auto k:{RS2_CAMERA_INFO_NAME,RS2_CAMERA_INFO_SERIAL_NUMBER,RS2_CAMERA_INFO_FIRMWARE_VERSION,RS2_CAMERA_INFO_PHYSICAL_PORT})if(x.supports(k))std::cout<<rs2_camera_info_to_string(k)<<" "<<x.get_info(k)<<std::endl;}}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}''')
subprocess.check_call(['g++','-std=c++17',str(src),'-I/opt/ros/noetic/include','-L/opt/ros/noetic/lib/aarch64-linux-gnu','-Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu','-lrealsense2','-o',str(r/'rs_probe')],timeout=20)
p=subprocess.run([str(r/'rs_probe')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=12);(r/'rs_probe.log').write_text(p.stdout);print(p.stdout,flush=True)
# Wrist-only capture remains independent of head SDK discovery.
import cv2,time,json
for serial,index in [('DEVICE_ID_03',0),('DEVICE_ID_04',8)]:
 cam=cv2.VideoCapture(index,cv2.CAP_V4L2);cam.set(cv2.CAP_PROP_FRAME_WIDTH,640);cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
 for _ in range(6):ok,im=cam.read()
 assert ok and im is not None,('Wrist unavailable',serial,index)
 cv2.imwrite(str(r/('wrist'+str(index)+'.jpg')),im);cam.release();print('WRIST_CAPTURED',serial,index,flush=True)
