from pathlib import Path
import json,time,sys
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');deadline=time.monotonic()+20;last=None
while time.monotonic()<deadline:
 try:s=json.loads((r/'mission_state.json').read_text())
 except (OSError,ValueError):time.sleep(.2);continue
 key=(s['phase'],s.get('label'))
 if key!=last:print(json.dumps(s),flush=True);last=key
 if s['phase'] in ['WAIT_POUR','POUR_WAIT','DONE','RECOVERY_ATTENTION']:break
 time.sleep(.3)
