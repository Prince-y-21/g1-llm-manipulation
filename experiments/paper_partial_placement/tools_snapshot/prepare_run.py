"""Reuse v4 reviewed source and tools in an isolated current-boot run. No I/O."""
from pathlib import Path
import shutil,json
S=Path(__file__).parent;R=S.parents[1];OLD=S.parent/'retry_resume_v4'
RUN=R/'runs/resume_20261025_2041';RUN.mkdir(exist_ok=True)
oldremote='/home/unitree/codex_g1_tasks/paper_to_mousepad_20261023_01/retry_resume_2314_v4'
remote='/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01'
runid='paper_resume_20261025_2041'
names=['paper_body_retry_v4.cpp','paper_gripper_v4.cpp','paper_cameras_v4.cpp','phase_guardian_retry_v4.py','retry_gate.hpp','retry_ack.hpp','retry_thermal_v4.hpp','bimanual_motion.hpp','retry_remote_v4.py','retry_call_v4.py','retry_evidence.py','compile_thermal_contract_v4.py','remote_compile_v4.py','test_contract_compiler_v4.py','test_preview_v4.py','test_thermal_v4.cpp']
for name in names:
    text=(OLD/name).read_text(encoding='utf-8').replace(oldremote,remote).replace('paper_resume_20261023_2314_v4',runid).replace('runs/retry_resume_v4','runs/resume_20261025_2041')
    (S/name).write_text(text,encoding='utf-8')
(S/'scene').mkdir(exist_ok=True)
for f in (OLD/'scene').glob('*.py'):
    text=f.read_text(encoding='utf-8').replace('runs/retry_resume_v4','runs/resume_20261025_2041').replace("RUNLOCAL/'preflight", "RUNLOCAL/'boot_check")
    (S/'scene'/f.name).write_text(text,encoding='utf-8')
(RUN/'runtime.json').write_text(json.dumps(dict(run_id=runid,remote_root=remote,boot_id='a56d8766-f726-4d4a-a91a-6fd92e496933',scope='A4纸到黑色鼠标垫，桌面目标；地面纸除外',source='v4复用并按本次实时场景重新规划，未重放旧绝对目标'),ensure_ascii=False,indent=2),encoding='utf-8')
(RUN/'boot_check/COMPLETE.json').write_text(json.dumps(dict(boot_id='a56d8766-f726-4d4a-a91a-6fd92e496933',result='Passive body/SDK/device/permissions/camera checks completed once; gripper actuator response to be verified only in task phase',do_not_repeat_full_check_in_this_boot=True),indent=2))
print('Prepared isolated current-boot sources; no controller started')
