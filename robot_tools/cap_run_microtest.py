"""Scoped, finite empty-hand body test. Not the cap-opening task controller."""
from pathlib import Path
import subprocess,time,json,os
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
s=json.loads((r/'pre_microtest_state.json').read_text());assert s['command_count']==0 and s['soc']>=70 and s['mode_machine']==5 and s['mode_pr']==0
assert all(j['motorstate']==0 and max(j['temperature'])<70 for j in s['joints'])
for port in ['/dev/ttyUSB1','/dev/ttyUSB2']:
 b=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True);assert not b.stdout.strip()
pre=dict(scope='suspended empty under-table outward microtest, not bottle contact',max_commanded_roll_change_rad=.03,automated_return_and_release=True,full_rig_confirmation='user current task',objects_held=False,model_note='Reference CAD and wrist views support small outward change; coarse initial overlaps are not asserted physical collisions; no general contact-motion permit',source_sha256=__import__('hashlib').sha256((r/'bimanual_empty_microtest.cpp').read_bytes()).hexdigest(),before=s)
(r/'microtest_preflight.json').write_text(json.dumps(pre,indent=2))
release=subprocess.run([str(old/'release_ai')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=12);(r/'microtest_release_ai.log').write_text(release.stdout);print(release.stdout,flush=True);assert release.returncode==0,'AI occupancy release not confirmed; do not start publisher'
p=subprocess.Popen([str(r/'bimanual_empty_microtest'),str(r),'--empty-outward-suspended'],stdout=(r/'microtest.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True);(r/'microtest.pid').write_text(str(p.pid));print('MICROTEST_PID',p.pid,flush=True)
try:code=p.wait(timeout=34)
except subprocess.TimeoutExpired:p.terminate();code=p.wait(timeout=8)
print((r/'microtest.log').read_text(),flush=True);print('MICROTEST_EXIT',code,flush=True)
subprocess.check_call([str(old/'observe'),str(r/'post_microtest_state.json')],timeout=12)
proc=Path('/proc')/str(p.pid);assert not proc.exists() or not (proc/'cmdline').read_bytes()
out=dict(exit=code,body_controller_exited=True,post_state=json.loads((r/'post_microtest_state.json').read_text()),status=json.loads((r/'microtest_status.json').read_text()) if (r/'microtest_status.json').exists() else None)
(r/'microtest_result.json').write_text(json.dumps(out,indent=2));print('POST_SNAPSHOT',json.dumps(dict(lowcmd=out['post_state']['command_count'],soc=out['post_state']['soc'],mode=out['post_state']['mode_machine'])),flush=True)
assert code==0,'Microtest was not verified; no progression to contact'
