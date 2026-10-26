from pathlib import Path
import os,json
base=Path('/home/unitree/dex1_1_service')
for p in base.rglob('*'):
 if p.is_file() and p.suffix in ['.cpp','.h','.yaml','.yml','.json'] and p.stat().st_size<200000:
  try:ls=p.read_text(errors='replace').splitlines()
  except OSError:continue
  found=[(i,l) for i,l in enumerate(ls) if any(k in l.lower() for k in ['322','left_motor','left_id','motor_id','ttyusb','calibration','angle_max'])]
  if found:
   print('FILE',str(p))
   for i,l in found[:15]: print('\n'.join(str(j+1)+': '+ls[j] for j in range(max(0,i-2),min(len(ls),i+3))))
print('RIGHT_RW',os.access('/dev/ttyUSB1',os.R_OK|os.W_OK),'LEFT_RW',os.access('/dev/ttyUSB2',os.R_OK|os.W_OK))
print('LEFT_PLAN_LOG',Path('/home/unitree/codex_g1_tasks/pour_water_20261019/planning_left.log').read_text())
