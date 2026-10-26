"""Capture evidence only; camera acquisition never publishes motor commands."""
import json, shutil, subprocess, sys, time
from pathlib import Path
r=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
label=sys.argv[1]
assert label.replace('_','').isalnum()
stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
out=r/'snapshots'/(stamp+'_'+label)
out.mkdir(parents=True,exist_ok=False)
before={}
for name in ['guard_status.json','gripper_status.json']:
    if (r/name).exists():before[name]=json.loads((r/name).read_text())
subprocess.run([str(r/'camera')],check=True)
for name in ['head_color.jpg','head_depth.png','head_intrinsics.txt','wrist0.jpg','wrist8.jpg','guard_status.json','gripper_status.json']:
    if (r/name).exists():shutil.copy2(str(r/name),str(out/name))
(out/'capture.json').write_text(json.dumps(dict(utc=stamp,label=label,state_before=before),indent=2))
print('SNAPSHOT',out,flush=True)
