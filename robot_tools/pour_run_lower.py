from pathlib import Path
import subprocess,os,time,shutil
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
if (r/'lower_tilt.log').exists():shutil.copy2(str(r/'lower_tilt.log'),str(r/('lower_tilt_previous_'+str(time.time_ns())+'.log')))
p=subprocess.Popen(['python3',str(r/'pour_lower_tilt.py')],stdout=(r/'lower_tilt.log').open('w'),stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),start_new_session=True)
print('LOW_POUR_PLAN_PID',p.pid)
