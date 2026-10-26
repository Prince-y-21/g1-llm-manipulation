from pathlib import Path
import subprocess,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
p=subprocess.Popen(['python3',str(r/'pour_extend_tilt.py')],stdout=(r/'extend_tilt.log').open('w'),stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),start_new_session=True)
print('OFFLINE_EXTENSION_PID',p.pid)
