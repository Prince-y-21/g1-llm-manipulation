"""Explicit, isolated retry operations; snapshot never queues a physical action."""
from pathlib import Path
import csv,json,sys,time,shutil,hashlib
from retry_evidence import snapshot,atomic,read
R=Path('/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01')
r=R/'control';run='paper_resume_20261025_2041'

def body(require_healthy=True):
    s=read(r/'body_status.json')
    assert time.monotonic()-s['steady']<1 and not s['releasing'],'body unavailable'
    if require_healthy:assert not s['fault'],s['reason']
    return s

def gripper(side,require_healthy=True):
    g=read(r/('gripper_'+side)/'status.json')
    assert g['protocol_version']>=2 and g['run']==run
    assert time.monotonic()-g['written_steady']<1,g
    if require_healthy:
        assert time.monotonic()-g['steady']<1 and g['controllable'] and g['communication_ok'],g
    return g

def main():
    action=sys.argv[1]
    atomic(r/'operator_activity.txt',str(time.monotonic()))
    if action=='snapshot':
        snapshot(r,run,sys.argv[2])
    elif action=='preview':
        label=sys.argv[2];assert label.replace('_','').isalnum()
        out=r/'checkpoints'/label;out.mkdir(parents=True,exist_ok=False)
        s=body(False);images={}
        for name in ('head','wrist0','wrist8'):
            # Last completed frame from the independently written capture index.
            with (r/'images'/(name+'_frames.csv')).open() as f:
                header=f.readline().strip().split(',')
                f.seek(max(0,(r/'images'/(name+'_frames.csv')).stat().st_size-4096))
                lines=f.read().splitlines()
            row=dict(zip(header,lines[-1].split(',')))
            stamp=float(row['steady_end']);assert time.monotonic()-stamp<2.5
            source=name+'_'+row['seq']+'.jpg';shutil.copy2(r/'images'/source,out/(name+'.jpg'))
            images[name]=dict(source=source,steady=stamp)
            if name=='head':shutil.copy2(r/'images'/('depth_'+row['seq']+'.png'),out/'depth.png')
        shutil.copy2(r/'images/camera_metadata.json',out/'camera_metadata.json')
        (out/'body_status.json').write_text(json.dumps(s,indent=2))
        (out/'grippers.json').write_text(json.dumps({side:gripper(side,False) for side in ('left','right')},indent=2))
        (out/'visual_manifest.json').write_text(json.dumps(dict(run=run,snapshot_id=label,images=images,kind='moving_observation_not_release_evidence'),indent=2))
        print(json.dumps(dict(snapshot=str(out),body=s,images=images)),flush=True)
    elif action=='status':
        print(json.dumps(dict(body=body(False),grippers={s:read(r/('gripper_'+s)/'status.json') for s in ('left','right')})),flush=True)
    elif action=='move':
        seq=int(sys.argv[2]);s=body();meta=read(r/f'stage_{seq}.json')
        assert meta['run_id']==run and s['seq']==meta['previous_seq'] and not s['moving']
        assert max(abs(a-b) for a,b in zip(meta['start_qgoal'],s['qgoal']))<.001
        for side in ('left','right'):
            g=gripper(side);assert not g['moving'] and not g['emergency'] and not g['state'].startswith('LIMIT_'),g
        atomic(r/'request.txt',f'{run} {seq} MOVE')
        deadline=time.monotonic()+meta['duration_s']+8
        while time.monotonic()<deadline:
            time.sleep(.1);s=body()
            if s['rejected_seq']==seq:raise RuntimeError('body rejected '+str(seq))
            if s['done_seq']==seq and not s['moving']:
                print(json.dumps(dict(event='BODY_TIME_PROFILE_COMPLETE_NOT_PHYSICAL_PROOF',tracking_error=max(abs(a-b) for a,b in zip(s['q'][15:],s['qgoal'])),body=s)),flush=True);return
        raise TimeoutError('body profile incomplete')
    elif action=='gripper':
        side=sys.argv[2];assert side in ('left','right')
        verb=sys.argv[3];assert verb in ('STEP_CLOSE','OPEN','HOLD_LAST_COMMAND','HOLD_MEASURED','CONFIRM_RELEASE','REARM')
        amount,speed,duration=map(float,sys.argv[4:7]);s=body(False);assert not s['moving']
        if verb=='STEP_CLOSE':
            assert not s['fault'] and not (r/'abort_requested').exists(),'Closure blocked during recovery'
        diagnostic=verb in ('REARM','HOLD_LAST_COMMAND')
        g=gripper(side,not diagnostic)
        if verb not in ('HOLD_LAST_COMMAND','HOLD_MEASURED'):assert not g['moving']
        seq=max(g['seq'],g['rejected_seq'])+1
        atomic(r/('gripper_'+side)/'request.txt',f'{run} {seq} {verb} {amount:.9g} {speed:.9g} {duration:.9g} HOLD 0')
        deadline=time.monotonic()+duration+5
        while time.monotonic()<deadline:
            time.sleep(.1);body(False);g=gripper(side,not diagnostic)
            if g['rejected_seq']==seq:raise RuntimeError('gripper rejected '+json.dumps(g))
            if g['emergency'] and verb not in ('OPEN','HOLD_LAST_COMMAND','HOLD_MEASURED','CONFIRM_RELEASE','REARM'):
                raise RuntimeError('gripper held for explicit recovery '+json.dumps(g))
            if g['state'].startswith('LIMIT_'):
                raise RuntimeError('official-limit observation froze action; explicit inspection/recovery required '+json.dumps(g))
            if g['done_seq']==seq and not g['moving']:
                if verb=='REARM':gripper(side,True)
                print(json.dumps(dict(event='GRIPPER_BOUNDED_ACTION_DONE_NOT_GRASP_PROOF',gripper=g)),flush=True);return
        raise TimeoutError('gripper transaction incomplete')
    elif action=='hold':
        s=body(False);seq=max(s['seq'],s['rejected_seq'])+1
        atomic(r/'request.txt',f'{run} {seq} STOP_HOLD');print('HOLD_REQUESTED',seq,flush=True)
    else:raise ValueError(action)
if __name__=='__main__':main()
