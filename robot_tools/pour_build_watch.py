from pathlib import Path
import subprocess,shlex,json
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
args=['g++','-std=c++17','-O2','-pthread','-I/opt/ros/noetic/include',str(r/'pour_watch.cpp'),'-L/opt/ros/noetic/lib/aarch64-linux-gnu','-Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu','-lrealsense2']+shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],universal_newlines=True))+['-o',str(r/'pour_watch')]
subprocess.check_call(args)
print('WATCH_BUILT_NOT_STARTED')
