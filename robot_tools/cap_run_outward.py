"""Finite empty-hand egress/return, based on audited CAD distance trend and microtest."""
from pathlib import Path
import subprocess,json,hashlib,time
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
r=root/'empty_outward_030_v2';r.mkdir(exist_ok=False)
old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
subprocess.check_call([str(old/'observe'),str(r/'before.json')],timeout=12)
before=json.loads((r/'before.json').read_text())
assert before['command_count']==0 and before['soc']>=60 and before['mode_machine']==5 and before['mode_pr']==0
assert all(j['motorstate']==0 and max(j['temperature'])<70 for j in before['joints'])
for port in ['/dev/ttyUSB1','/dev/ttyUSB2']:
    p=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert not p.stdout.strip()
source=(root/'bimanual_empty_microtest_fixed.cpp').read_text()
replacements=[('outward 0.03 rad','outward 0.30 rad'),('t>28','t>45'),
('outward[1]+=.03f;outward[8]-=.03f','outward[1]+=.30f;outward[8]-=.30f'),
('t<11','t<18'),('(t-8)/3','(t-8)/10'),('t<13','t<20'),('t<16','t<30'),('(t-13)/3','(t-20)/10'),
('t>=12.5f','t>=19.5f'),('verified_left<.006f||verified_right<.006f','verified_left<.20f||verified_right<.20f'),
('t>=18','t>=32')]
for a,b in replacements:
    assert a in source,a;source=source.replace(a,b)
# Guard total absolute arm torque during this empty motion, without loosening tracking.
source=source.replace('else if(std::abs(m.dq())>.5f)', 'else if(i>=15&&std::abs(m.tau_est())>7.f)bad="empty_arm_torque";else if(std::abs(m.dq())>.5f)')
# Position tracking is meaningful while stiffness is applied. During deliberate
# unloading, retain speed/mode/fault/temperature guards and record the motion.
needle='else if(std::abs(m.q()-desired[i])>(i<15?.035f:.065f))'
assert needle in source
source=source.replace(needle,'else if(!releasing&&std::abs(m.q()-desired[i])>(i<15?.035f:.065f))')
source=source.replace('int cycle=0;', 'int cycle=0;std::ofstream trace(dir+"feedback.csv");trace<<"t,releasing,j,q,dq,tau,qdes,tauff,scale,temperature_case,temperature_winding\\n";')
needle='  if(cycle%50==0){'
assert needle in source
source=source.replace(needle,'''  if(cycle%10==0){for(int i=0;i<29;++i){auto m=s.motor_state()[i];trace<<t<<","<<releasing<<","<<i<<","<<m.q()<<","<<m.dq()<<","<<m.tau_est()<<","<<desired[i]<<","<<ff[i]<<","<<scale<<","<<int(m.temperature()[0])<<","<<int(m.temperature()[1])<<"\\n";}trace.flush();}
'''+needle)
(r/'outward.cpp').write_text(source)
subprocess.check_call(['g++','-std=c++17','-O2','-I'+str(root),'-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/'outward.cpp'),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/'outward')],timeout=55)
pre=dict(scope='Suspended empty hands only. Outward .30rad then measured-baseline return. No object contact.',
    body_geometry='Full reference visual meshes incl finger rails, both arms and legs; task-local candidate, physical assembly not calibrated.',
    initial_near_pairs='Reference left wrist/hip hull overlap ~4mm at known resting pose. This is an egress test: outward motion monotonically separates the near pair. Not accepted as a general contact-free path.',
    max_command_speed_rad_s=.045,automatic_return=True,automatic_gain_release=True,
    source_sha256=hashlib.sha256(source.encode()).hexdigest())
(r/'preflight.json').write_text(json.dumps(pre,indent=2))
with (r/'run.log').open('w') as log:
    p=subprocess.Popen([str(r/'outward'),str(r),'--empty-outward-suspended'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (r/'pid').write_text(str(p.pid));print('EMPTY_OUTWARD_PID',p.pid,flush=True)
    try:code=p.wait(timeout=52)
    except subprocess.TimeoutExpired:p.terminate();code=p.wait(timeout=8)
print((r/'run.log').read_text(),flush=True)
subprocess.check_call([str(old/'observe'),str(r/'after.json')],timeout=12)
after=json.loads((r/'after.json').read_text());status=json.loads((r/'microtest_status.json').read_text())
record=dict(exit_code=code,process_exited=not (Path('/proc')/str(p.pid)/'cmdline').exists(),after=after,status=status)
(r/'result.json').write_text(json.dumps(record,indent=2))
print('OUTWARD_RESULT',json.dumps(dict(exit=code,lowcmd=after['command_count'],soc=after['soc'],status=status)),flush=True)
assert code==0 and not after['command_count'],'No progression after unverified empty cycle'
