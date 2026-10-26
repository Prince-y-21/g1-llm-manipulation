"""Prepare reviewed launch and normal finish scripts; no connection or motion."""
from pathlib import Path
import json,hashlib
S=Path(__file__).parent;R=S.parents[1];RUN=R/'runs/resume_20261025_2041';P=RUN/'planning'
check=json.loads((P/'safety_route_check.json').read_text());assert all(check[k] for k in ['carry_checked','empty_retract_checked','home_checked','low_lift_checked'])
assert json.loads((P/'complete_check.json').read_text())['passed']
review=dict(nominal_route_reviewed=True,dense_closure_reviewed=True,paper_carry_reviewed=True,model_thermal_relief_reviewed=True,actual_grasp_verified=False,actual_support_verified=False,requires_wrist_noncontact_edge_observation=True,boot_id='a56d8766-f726-4d4a-a91a-6fd92e496933',files={name:hashlib.sha256((P/name).read_bytes()).hexdigest() for name in ['scene.json','complete_nominal_route.json','complete_check.json','safety_route_check.json','safe_carry_waypoints.json','empty_retract_waypoints.json','empty_home_waypoints.json','low_lift_waypoints.json']})
(RUN/'reviewed_plan.json').write_text(json.dumps(review,indent=2))
old=S.parent/'retry_2054'
for name,target in [('remote_launch_retry.py','remote_launch.py'),('remote_finish_retry.py','remote_finish.py')]:
 text=(old/name).read_text(encoding='utf-8').replace('/paper_to_mousepad_20261023_01/retry_2107','/paper_to_mousepad_20261025_01').replace('8a9434a0-4555-4ba8-8135-6535de34be15','a56d8766-f726-4d4a-a91a-6fd92e496933').replace('paper_retry_20261023_2107','paper_resume_20261025_2041').replace('phase_guardian_retry.py','phase_guardian_retry_v4.py')
 (S/target).write_text(text,encoding='utf-8')
print('Prepared reviewed noncontact startup, carry, empty retreat and normal finish paths')
