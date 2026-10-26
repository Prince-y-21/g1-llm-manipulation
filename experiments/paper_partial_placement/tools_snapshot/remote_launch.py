"""Start the isolated, audited phase after a separately reviewed complete plan exists."""
from pathlib import Path
import json,subprocess,time
R=Path('/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01')
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()=='a56d8766-f726-4d4a-a91a-6fd92e496933'
assert json.loads((R/'dry_run.json').read_text())['returncode']==0
assert (R/'reviewed_plan.json').exists(),'No reviewed complete plan; do not start holding motors while developing'
review=json.loads((R/'reviewed_plan.json').read_text())
assert review['nominal_route_reviewed'] and review['dense_closure_reviewed'] and review['paper_carry_reviewed']
assert not (R/'control').exists(),'Previous runtime present; inspect, never relaunch over it'
log=(R/'guardian.log').open('w')
p=subprocess.Popen(['python3',str(R/'phase_guardian_retry_v4.py'),'--root',str(R),'--phase','control','--run','paper_resume_20261025_2041','--execute'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(R/'guardian.pid').write_text(str(p.pid))
deadline=time.monotonic()+32
while time.monotonic()<deadline and p.poll() is None and not (R/'control/ready').exists():time.sleep(.2)
if not (R/'control/ready').exists():
    print((R/'guardian.log').read_text(),flush=True)
    raise RuntimeError('Startup needs inspection; do not kill an active body as cleanup')
print('ISOLATED_PHASE_READY',p.pid,flush=True)
print((R/'control/body_status.json').read_text(),flush=True)
