"""Minimal retry supervisor: soft faults stop/hold, never kill or unload a live body.
Default actions require --execute; this file is safe to import for offline inspection.
No claim of indefinite physical safety: latched hold requires timely recovery/operator.
"""
from pathlib import Path
import argparse,json,os,signal,subprocess,threading,time,shutil,hashlib

def atomic(p,text):
    tmp=p.with_name(p.name+f'.{os.getpid()}.{threading.get_ident()}.tmp');tmp.write_text(text,encoding='utf-8');tmp.replace(p)
def read(p):
    try:return json.loads(p.read_text(encoding='utf-8'))
    except (OSError,ValueError):return None
def frame_age(path):
    try:
        with path.open('rb') as f:f.seek(max(0,path.stat().st_size-2048));lines=f.read().decode().splitlines()
        return time.monotonic()-float(lines[-1].split(',')[2])
    except (OSError,ValueError,IndexError):return float('inf')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--phase',required=True);ap.add_argument('--run',required=True);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    if not args.execute:raise SystemExit('No hardware action without --execute')
    R=args.root;r=R/args.phase;run=args.run
    if not str(R).startswith('/home/unitree/codex_g1_tasks/paper_to_mousepad_') or '/' in args.phase:raise ValueError('task directory')
    D=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
    r.mkdir(exist_ok=False);(r/'images').mkdir()
    for side in ('left','right'):(r/('gripper_'+side)).mkdir()
    subprocess.check_call([str(D/'task_observe_fixed'),str(r/'before.json')],timeout=12)
    before=read(r/'before.json')
    assert before['command_count']==0 and before['invalid_states']==0 and before['soc']>=20 and before['mode_machine']==5 and before['mode_pr']==0
    for port in ('/dev/ttyUSB2','/dev/ttyUSB1'):
        fuser=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert not fuser.stdout.strip()
    done=threading.Event();soft_signal=threading.Event();started=time.monotonic();last_activity=started;hold_reasons=set();children={};body=None;cam=None;long_task_noted=False
    def heartbeat():
        while not done.is_set():atomic(r/'guardian_heartbeat.txt',str(time.monotonic()));done.wait(.1)
    def freeze_grippers():
        for side,p in children.items():
            if p.poll() is not None:continue
            gs=read(r/('gripper_'+side)/'status.json')
            if not gs:continue
            n=max(gs.get('seq',0),gs.get('rejected_seq',0))+1
            try:
                pending=(r/('gripper_'+side)/'request.txt').read_text().split()
                if len(pending)>1:n=max(n,int(pending[1])+1)
            except (OSError,ValueError):pass
            atomic(r/('gripper_'+side)/'request.txt',f'{run} {n} HOLD_LAST_COMMAND 0 0 0 HOLD 0')
    def request_hold(reason):
        if reason in hold_reasons:return
        hold_reasons.add(reason);atomic(r/'stop_hold_requested.txt',reason)
        atomic(r/'abort_requested',reason);freeze_grippers()
        atomic(r/'guardian_reason.txt',reason);print('GUARDIAN_STOP_HOLD',reason,flush=True)
    def report(state,detail):
        atomic(r/'guardian_status.json',json.dumps(dict(steady=time.monotonic(),state=state,detail=detail,hold_reasons=sorted(hold_reasons))))
    signal.signal(signal.SIGTERM,lambda *_:soft_signal.set());signal.signal(signal.SIGINT,lambda *_:soft_signal.set())
    thread=threading.Thread(target=heartbeat,daemon=True);thread.start()
    try:
        sources=r/'executed_sources';sources.mkdir()
        for name in ('paper_body_retry_v4.cpp','retry_gate.hpp','retry_ack.hpp','retry_thermal_v4.hpp','bimanual_motion.hpp','paper_body_retry_v4','paper_gripper_v4.cpp','paper_gripper_v4','paper_cameras_v4.cpp','paper_cameras_v4','phase_guardian_retry_v4.py'):
            shutil.copy2(R/name,sources/name)
        (r/'versions.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources.iterdir()},indent=2))
        cam=subprocess.Popen([str(R/'paper_cameras_v4'),str(r/'images'),'DEVICE_SERIAL_TO_CONFIGURE','0'],stdout=(r/'camera.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+9
        while not (r/'images/ready').exists() and cam.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        assert (r/'images/ready').exists(),'camera_not_ready'
        for side in ('left','right'):
            children[side]=subprocess.Popen([str(R/'paper_gripper_v4'),str(r),run,side,'--execute'],stdout=(r/(side+'_gripper.log')).open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+6
        while time.monotonic()<deadline:
            states=[read(r/('gripper_'+side)/'status.json') for side in children]
            if all(states):break
            assert all(p.poll() is None for p in children.values()),'gripper_start_failed';time.sleep(.1)
        assert all(states),'gripper_state_unavailable'
        body=subprocess.Popen([str(R/'paper_body_retry_v4'),str(r),run,'--execute-paper-phase'],stdout=(r/'body.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
        (r/'body.pid').write_text(str(body.pid));deadline=time.monotonic()+10;state=None
        while time.monotonic()<deadline and body.poll() is None:
            state=read(r/'body_status.json')
            if state and state['elapsed']>=5.1:break
            time.sleep(.1)
        assert body.poll() is None and state and state['elapsed']>=5.1,'body_start_failed'
        atomic(r/'ready',run);print('RETRY_PHASE_READY',body.pid,flush=True)
        while body.poll() is None:
            time.sleep(.1);now=time.monotonic();state=read(r/'body_status.json')
            if soft_signal.is_set():request_hold('supervisor_signal')
            if now-started>1100 and not long_task_noted:
                long_task_noted=True
                atomic(r/'long_task_notice.json',json.dumps(dict(elapsed_s=now-started,notice='telemetry_only_no_control_transition')))
                print('LONG_TASK_NOTICE telemetry_only',now-started,flush=True)
            if not state or now-state['steady']>.8:request_hold('body_state_stale');continue
            thermal_recovery=state.get('thermal_unloading',False) or state.get('thermal_relief_done',False)
            if state['fault'] and not thermal_recovery and 'body_fault_observed' not in hold_reasons:
                hold_reasons.add('body_fault_observed');atomic(r/'abort_requested',state['reason']);freeze_grippers()
                atomic(r/'guardian_body_fault.txt',state['reason'])
            if state['soc']<20:request_hold('official_battery_reserve')
            if any(t[0]>85 or t[1]>120 for t in state['temperature']):
                atomic(r/'guardian_thermal_observation.txt','body owns thermal route selection; no second STOP/preload change')
            if cam.poll() is not None or frame_age(r/'images/head_frames.csv')>2.5:request_hold('camera_stale')
            for side,p in children.items():
                gs=read(r/('gripper_'+side)/'status.json')
                if p.poll() is not None or not gs or now-gs['steady']>.8 or not gs.get('controllable',False):request_hold('gripper_control_unavailable_'+side)
            report(state.get('hold_state','UNKNOWN'),state['reason'])
        if not (r/'body_exited').exists():
            # Lost authority is not a verified rest state. Do not terminate the grippers.
            report('UNCONTROLLABLE_NEEDS_ONSITE_RESPONSE','body exited without verified-rest completion; grippers remain independent')
            print('BODY_UNCONTROLLABLE_NO_AUTOMATIC_GRIPPER_BRAKE',flush=True)
            return 3
        # body_exited is written only by the explicit, evidenced rest-release branch.
        for side,p in children.items():
            if p.poll() is not None:continue
            gs=read(r/('gripper_'+side)/'status.json')
            if not gs or not gs.get('release_confirmed',False):
                report('GRIPPER_RELEASE_CONFIRMATION_REQUIRED',side);return 4
            n=max(gs.get('seq',0),gs.get('rejected_seq',0))+1
            atomic(r/('gripper_'+side)/'request.txt',f'{run} {n} STOP_AFTER_BODY 0 0 0 HOLD 0')
        for p in children.values():p.wait(timeout=15)
        atomic(r/'images/stop','1');cam.wait(timeout=8)
        subprocess.run([str(D/'task_observe_fixed'),str(r/'after.json')],timeout=12)
        atomic(r/'result.json',json.dumps(dict(body_exit=body.returncode,gripper_exits={side:p.returncode for side,p in children.items()},camera_exit=cam.returncode,elapsed_s=time.monotonic()-started,after=read(r/'after.json')),indent=2))
        report('FINISHED_AT_VERIFIED_REST','explicit release and shutdown acknowledged');return 0
    except Exception as exc:
        request_hold('supervisor_exception_'+type(exc).__name__);report('SUPERVISOR_ERROR_RECOVERY_REQUIRED',repr(exc))
        # Never SIGTERM/SIGKILL a live body to satisfy a cleanup timeout.
        # Remain available to the upper layer; this is not an automatic safe recovery.
        if body and body.poll() is None:
            while body.poll() is None:time.sleep(.2)
        return 5
    finally:
        # If body/control failed, child grippers deliberately remain in last-command hold
        # or uncontrollable query state. These PIDs are recorded; no promise of safety.
        alive={side:p.pid for side,p in children.items() if p.poll() is None}
        if cam and cam.poll() is None:alive['camera']=cam.pid
        atomic(r/'remaining_processes.json',json.dumps(alive,indent=2));done.set();thread.join(timeout=2)
if __name__=='__main__':raise SystemExit(main())
