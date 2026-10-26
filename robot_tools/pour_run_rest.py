from pathlib import Path
import sys,json
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_action as a
for p in json.loads((r/'route_rest_ready.json').read_text()):a.move(p['label'])
a.log('REST_LOW_LOAD_COMPLETE',state=a.gs());print('REST_LOW_LOAD_COMPLETE',flush=True)
