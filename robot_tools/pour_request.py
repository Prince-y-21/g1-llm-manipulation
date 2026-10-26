from pathlib import Path
import sys,json,time,os
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');req=dict(request_id=int(time.time()*1000),action=sys.argv[1])
if req['action']=='continue':req['checkpoint']=sys.argv[2]
elif req['action'] in ['pulse','tilt']:req['labels']=sys.argv[2:]
p=r/'mission_request.tmp';p.write_text(json.dumps(req));os.replace(str(p),str(r/'mission_request.json'));print(json.dumps(req))
