"""Read-only exit verification and final scene capture; no motor commands."""
from pathlib import Path
import json, subprocess, time

R=Path('/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01')
D=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
r=R/'control'
assert (r/'body_exited').exists() and (r/'result.json').exists()
F=R/'final';F.mkdir(exist_ok=False)
ps=subprocess.check_output(['ps','-eo','pid,etimes,args'],text=True)
names=('paper_body_retry','paper_gripper_v','paper_cameras_v','phase_guardian_retry')
active=[line for line in ps.splitlines() if any(n in line for n in names)]
owners=subprocess.run(['fuser','/dev/ttyUSB1','/dev/ttyUSB2'],capture_output=True,text=True)
(F/'processes.txt').write_text(ps)
assert not active,active
assert not owners.stdout.strip(),owners.stdout
obs=subprocess.run([str(D/'task_observe_fixed'),str(F/'state.json')],capture_output=True,text=True,timeout=12)
(F/'observer.log').write_text(obs.stdout+obs.stderr)
assert obs.returncode==0,obs.stderr
s=json.loads((F/'state.json').read_text())
assert s['command_count']==0 and s['invalid_states']==0,s
I=F/'images';I.mkdir()
cam=subprocess.run([str(D/'cap_record_rgbd'),str(I),'DEVICE_SERIAL_TO_CONFIGURE','3'],capture_output=True,text=True,timeout=20)
(F/'camera.log').write_text(cam.stdout+cam.stderr)
assert cam.returncode==0,cam.stderr
summary=dict(server_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),steady=time.monotonic(),
 boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),task_processes=active,
 serial_owners=owners.stdout.strip(),lowcmd=s['command_count'],invalid_states=s['invalid_states'],
 mode=[s['mode_machine'],s['mode_pr']],soc=s['soc'],fault_joints=[j for j in s['joints'] if j['motorstate']],
 max_temperature=max(max(j['temperature']) for j in s['joints']),
 controller_result=json.loads((r/'result.json').read_text()),
 scope='Post-execution authority/scene verification only; not a repeated full boot inventory.')
(F/'verification.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2),flush=True)
print('FINAL_FILES',json.dumps([str(p.relative_to(R)) for p in F.rglob('*') if p.is_file()]),flush=True)
