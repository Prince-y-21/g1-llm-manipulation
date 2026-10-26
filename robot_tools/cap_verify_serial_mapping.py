"""Bounded BRAKE-query diagnostic, not passive observation or FOC movement."""
from pathlib import Path
import subprocess,time,json,os
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');probe='/home/unitree/codex_g1_tasks/pour_water_20261019/pour_probe'
records=[]
# Known historical IDs only, on the confirmed left interface. Do not guess baud or force a motion mode.
for port,id in [('/dev/ttyUSB2',1),('/dev/ttyUSB2',0),('/dev/ttyUSB1',0)]:
 busy=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
 assert not busy.stdout.strip(),('Serial busy',port,busy.stdout)
 time.sleep(.5)
 p=subprocess.run([probe,port,str(id)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=6)
 x=dict(server_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),port=port,id=id,exit=p.returncode,output=p.stdout,sends_brake=True,enabled_foc=False)
 records.append(x);print(json.dumps(x),flush=True)
(r/'serial_mapping_check.json').write_text(json.dumps(dict(records=records,other_interfaces_permissions={p:os.access(p,os.R_OK|os.W_OK) for p in ['/dev/ttyUSB0','/dev/ttyUSB3']}),indent=2))
