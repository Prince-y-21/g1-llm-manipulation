from pathlib import Path
import subprocess,json,hashlib,sys,time,shutil
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');r=root/sys.argv[1]
assert r.parent==root and r.is_dir() and not (r/'run.log').exists()
observer=root/'task_observe_fixed'
stem='bimanual_finite_servo' if '--servo3' in sys.argv else ('bimanual_finite_waist' if '--waist-test' in sys.argv else 'bimanual_finite_profile')
src=root/(stem+'.cpp');binary=root/stem
if not binary.exists() or binary.stat().st_mtime<src.stat().st_mtime:
    subprocess.check_call(['g++','-std=c++17','-O2','-I'+str(root),'-I/usr/local/include','-I/usr/local/include/ddscxx',str(src),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(binary)],timeout=55)
dry=subprocess.run([str(binary),str(r),'--dry-run'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=12)
(r/'dry.log').write_text(dry.stdout);print(dry.stdout,flush=True);assert dry.returncode==0
subprocess.check_call([str(observer),str(r/'before.json')],timeout=12);s=json.loads((r/'before.json').read_text())
min_soc=45 if any(x in sys.argv for x in ['--left-force080','--left-fullstroke','--left-fullstroke90','--waist-test']) else 60
if '--servo3' in sys.argv:min_soc=35
assert s['command_count']==0 and s['soc']>=min_soc
if min_soc<60:
    duration=float((r/'profile.txt').read_text().strip().splitlines()[-1].split()[0]);(r/'battery_budget.json').write_text(json.dumps(dict(min_soc=min_soc,trajectory_s=duration,controlled_duration_bound_s=duration+13.3,scope='Finite empty trajectory, no object held',current_soc=s['soc'],basis='Previous 107.7s empty trajectory consumed ~1 percentage point. This is a stage reserve, not a firmware limit. Earlier 60% exploratory gate aborted before any publisher.')))
for port in ['/dev/ttyUSB1','/dev/ttyUSB2']:
    p=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert not p.stdout.strip()
(r/'binding.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),profile_sha256=hashlib.sha256((r/'profile.txt').read_bytes()).hexdigest()),indent=2))
shutil.copy2(src,r/'executed_body_source.cpp');shutil.copy2(binary,r/'executed_body_binary')
recorder=None
if '--record' in sys.argv:
    images=r/'images';images.mkdir(exist_ok=False)
    rec_log=(r/'camera.log').open('w');recorder=subprocess.Popen([str(root/'cap_record_rgbd'),str(images),'DEVICE_SERIAL_TO_CONFIGURE','225'],stdout=rec_log,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+10
    while not (images/'ready').exists() and recorder.poll() is None and time.monotonic()<deadline:time.sleep(.1)
    if not (images/'ready').exists():
        recorder.terminate();recorder.wait(timeout=8);raise RuntimeError('Passive camera not ready; no body publisher started')
with (r/'run.log').open('w') as log:
    p=subprocess.Popen([str(binary),str(r),'--execute-empty-profile'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(r/'pid').write_text(str(p.pid));print('FINITE_PROFILE_PID',p.pid,flush=True)
    if any(x in sys.argv for x in ['--left-free-space','--left-force-step','--left-force080','--left-fullstroke','--left-fullstroke90','--prepare-grippers']):
        deadline=time.monotonic()+35;triggered=False
        while p.poll() is None and time.monotonic()<deadline:
            if (r/'status.json').exists():
                st=json.loads((r/'status.json').read_text())
                if st['fault'] or st['reverse'] or st['release']:break
                if 11.5<st['phase']<18:
                    if '--prepare-grippers' in sys.argv:
                        import csv,collections
                        lines=list(collections.deque((r/'feedback.csv').open(),maxlen=29));values=[list(map(float,line.split(','))) for line in lines]
                        arms=[v for v in values if v[5]>=15]
                        if len(arms)!=14 or max(abs(v[6]-v[9]) for v in arms)>.025 or max(abs(v[7]) for v in arms)>.06:time.sleep(.1);continue
                        subprocess.run(['python3',str(root/'run_ready_grippers.py')],stdout=(r/'gripper_hook.log').open('w'),stderr=subprocess.STDOUT,timeout=80);triggered=True;break
                    helper='cap_left_fullstroke90.py' if '--left-fullstroke90' in sys.argv else ('cap_left_fullstroke.py' if '--left-fullstroke' in sys.argv else ('cap_left_force080.py' if '--left-force080' in sys.argv else ('cap_left_force_step.py' if '--left-force-step' in sys.argv else 'cap_left_free_space_cycle.py')))
                    timeout=120 if '--left-fullstroke90' in sys.argv else (80 if '--left-fullstroke' in sys.argv else 20)
                    subprocess.run(['python3',str(root/helper)],stdout=(r/'gripper_hook.log').open('w'),stderr=subprocess.STDOUT,timeout=timeout);triggered=True;break
            time.sleep(.1)
        (r/'hook_result.json').write_text(json.dumps(dict(triggered=triggered)))
    try:code=p.wait(timeout=215)
    except subprocess.TimeoutExpired:p.terminate();code=p.wait(timeout=190)
print((r/'run.log').read_text(),flush=True)
if recorder:
    (images/'stop').write_text('1')
    try:recorder.wait(timeout=8)
    except subprocess.TimeoutExpired:recorder.terminate();recorder.wait(timeout=8)
    print('PASSIVE_RECORDER_EXIT',recorder.returncode,flush=True)
subprocess.check_call([str(observer),str(r/'after.json')],timeout=12);s=json.loads((r/'after.json').read_text());status=json.loads((r/'status.json').read_text()) if (r/'status.json').exists() else None
(r/'result.json').write_text(json.dumps(dict(exit_code=code,body_controller_exited=not (Path('/proc')/str(p.pid)/'cmdline').exists(),after=s,status=status),indent=2));print('PROFILE_RESULT',code,s['command_count'],s['soc'],status,flush=True)
assert code==0 and s['command_count']==0
