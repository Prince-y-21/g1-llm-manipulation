from pathlib import Path
import subprocess,json,shlex,time
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z');p=r/'camera.cpp';s=p.read_text();assert 'DEVICE_ID_01' in s
s=s.replace('DEVICE_ID_01','DEVICE_SERIAL_TO_CONFIGURE')
anchor='auto in=color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();'
s=s.replace(anchor,anchor+'''\n std::ofstream meta("'''+str(r)+'''/head_metadata.json");meta<<"{\\"sdk_serial\\":\\""<<profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<"\\",\\"color_frame\\":"<<color.get_frame_number()<<",\\"depth_frame\\":"<<depth.get_frame_number()<<",\\"color_timestamp_ms\\":"<<color.get_timestamp()<<",\\"depth_timestamp_ms\\":"<<depth.get_timestamp()<<",\\"fx\\":"<<in.fx<<",\\"fy\\":"<<in.fy<<",\\"cx\\":"<<in.ppx<<",\\"cy\\":"<<in.ppy<<",\\"depth_scale\\":"<<depth.get_units()<<",\\"distortion_model\\":"<<(int)in.model<<",\\"coeffs\\":[";for(int j=0;j<5;++j){if(j)meta<<",";meta<<in.coeffs[j];}meta<<"]}";meta.close();''')
p.write_text(s)
args=['g++','-std=c++17','-O2','-I/opt/ros/noetic/include',str(p),'-L/opt/ros/noetic/lib/aarch64-linux-gnu','-Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu','-lrealsense2']+shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],universal_newlines=True))+['-o',str(r/'camera')]
subprocess.check_call(args,timeout=25);subprocess.check_call([str(r/'camera')],timeout=25)
m=json.loads((r/'manifest.json').read_text());m['head_identity_note']='UVC/by-id historical serial DEVICE_ID_01 differs from live librealsense serial 254322075469; unique discovered RGB-D device, current capture pinned to SDK serial. Cause unverified.';m['head_current_sdk_serial']='DEVICE_SERIAL_TO_CONFIGURE';m['latest_capture_utc']=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());(r/'manifest.json').write_text(json.dumps(m,indent=2));print((r/'head_metadata.json').read_text(),flush=True)
