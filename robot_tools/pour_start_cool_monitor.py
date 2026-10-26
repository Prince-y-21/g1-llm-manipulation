from pathlib import Path
import subprocess
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
p=subprocess.Popen(['python3','-u',str(r/'pour_cool_monitor.py')],stdout=(r/'cooling.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True);(r/'cool_monitor.pid').write_text(str(p.pid));print('READ_ONLY_COOL_MONITOR',p.pid)
