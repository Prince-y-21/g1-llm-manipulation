from pathlib import Path
import shutil,subprocess,os
old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016');r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
shutil.copy2(str(old/'g1_29dof_rev_1_0.urdf'),str(r/'g1_29dof_rev_1_0.urdf'))
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
p=subprocess.Popen(['python3',str(r/'pour_model.py')],stdout=(r/'planning.log').open('w'),stderr=subprocess.STDOUT,env=env,start_new_session=True)
print('OFFLINE_PLANNER_PID',p.pid)
