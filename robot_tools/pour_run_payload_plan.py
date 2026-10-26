from pathlib import Path
import subprocess,os,time,shutil
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
if (r/'payload_plan.log').exists():shutil.copy2(str(r/'payload_plan.log'),str(r/('payload_plan_previous_'+str(time.time_ns())+'.log')))
p=subprocess.Popen(['python3',str(r/'pour_plan_payload.py')],stdout=(r/'payload_plan.log').open('w'),stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),start_new_session=True)
print('PAYLOAD_PLAN_PID',p.pid)
