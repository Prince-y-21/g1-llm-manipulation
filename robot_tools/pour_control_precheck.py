from pathlib import Path
import numpy as np,json,subprocess
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
cases=[]
for duration in [.8,1.2,3.,10.]:
 for hold in [0.,.25,.5]:
  elapsed=np.linspace(0,2*duration+hold+.7,20001)
  phase=np.where(elapsed<duration,elapsed/duration,np.where(elapsed<duration+hold,1.,1-(elapsed-duration-hold)/duration))
  phase=np.clip(phase,0,1);blend=phase**2*(3-2*phase)
  delta=min(.2,.08*duration/1.5*.99);position=delta*blend
  speed=float(np.max(abs(np.diff(position)/np.diff(elapsed))))
  assert position[0]==0 and position[-1]==0 and np.min(position)>=0 and np.max(position)<=delta+1e-9
  assert speed<=.080001
  cases.append(dict(duration=duration,hold=hold,delta=delta,peak_numeric_speed=speed))
(r/'pulse_trajectory_check.json').write_text(json.dumps(dict(passed=True,cases=cases),indent=2))
print('PULSE_TRAJECTORY_PROPERTIES_PASS',len(cases))
subprocess.check_call([str(r/'pour_guard_pulse')],timeout=8)
