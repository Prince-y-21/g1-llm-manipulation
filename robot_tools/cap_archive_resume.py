"""Read-only final observation/camera and task evidence archive. No motor command."""
from pathlib import Path
import subprocess, json, time, tarfile, hashlib

r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
out=r/'final_readonly';out.mkdir(exist_ok=False)
subprocess.check_call([str(r/'task_observe_fixed'),str(out/'state.json')],timeout=12)
images=out/'images';images.mkdir()
p=subprocess.run([str(r/'cap_record_rgbd'),str(images),'DEVICE_SERIAL_TO_CONFIGURE','3'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=18)
(out/'camera.log').write_text(p.stdout)
checks={}
for name,args in [('processes',['ps','-eo','pid,args']),('serial_owners',['fuser','/dev/ttyUSB1','/dev/ttyUSB2']),('service',['systemctl','show','dex1_gripper.service','-p','ActiveState','-p','SubState','-p','MainPID','-p','ExecMainStatus'])]:
    q=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=8)
    text=q.stdout
    if name=='processes':text='\n'.join(s for s in text.splitlines() if any(x in s.lower() for x in ['bimanual','gripper','cap_task','cap_record','servo','lowcmd']))
    checks[name]={'exit':q.returncode,'output':text}
checks.update(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),server_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),server_monotonic=time.monotonic(),camera_exit=p.returncode)
(out/'checks.json').write_text(json.dumps(checks,indent=2))
skip={'python_deps','wheelhouse','assets','versions_queue1','empty_calibration_01','waist_micro_005','waist_nograv_005','__pycache__'}
archive=r/'resume_remaining_evidence.tgz'
manifest=[]
with tarfile.open(str(archive),'w:gz',compresslevel=1) as tar:
    for f in sorted(r.rglob('*')):
        rel=f.relative_to(r)
        if any(x in skip for x in rel.parts) or not f.is_file() or f.suffix=='.tgz':continue
        tar.add(str(f),arcname=str(rel),recursive=False)
        manifest.append({'path':str(rel),'bytes':f.stat().st_size})
(r/'resume_remaining_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'state':json.loads((out/'state.json').read_text()),'checks':checks},indent=2),flush=True)
