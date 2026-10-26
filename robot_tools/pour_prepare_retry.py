from pathlib import Path
import sys,subprocess,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_action as a
a.grip(3.,.25)
a.snapshot('retry_scene')
for script,args in [('pour_leg_model.py',[]),('pour_calibrate_payload.py',['--scene-only']),('pour_retry_approach.py',[])]:
 subprocess.check_call(['python3',str(r/script)]+args,timeout=30)
env=dict(os.environ,POUR_PLAN_PREDICTED='1',OPENBLAS_NUM_THREADS='1')
subprocess.check_call(['python3',str(r/'pour_plan_payload.py')],env=env,timeout=65)
print('RETRY_ALL_PLANS_READY',flush=True)
