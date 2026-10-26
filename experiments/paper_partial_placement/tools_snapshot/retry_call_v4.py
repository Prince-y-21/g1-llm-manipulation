"""Local public audit + explicit remote call. A snapshot is a separate operation."""
from pathlib import Path
from datetime import datetime,timezone
import json,subprocess,sys,time,shutil
R=Path(__file__).resolve().parents[2];RUN=R/'runs/resume_20261025_2041'
REMOTE='/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01'
tag=datetime.now(timezone.utc).strftime('%H%M%S_%f');audit=RUN/'audit'/tag;audit.mkdir(parents=True)
ssh=['ssh','-F','C:/Users/researcher/.ssh/config','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=8','robot-g1']
scp=['scp','-F','C:/Users/researcher/.ssh/config','-q','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes']
def call(argv,timeout=180):
    i=len(list(audit.glob('call_*.json')));start=time.time()
    p=subprocess.run([str(a) for a in argv],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout)
    (audit/f'output_{i}.log').write_bytes(p.stdout)
    (audit/f'call_{i}.json').write_text(json.dumps(dict(argv=[str(a) for a in argv],start_epoch=start,end_epoch=time.time(),exit=p.returncode),ensure_ascii=False,indent=2),encoding='utf-8')
    print(p.stdout.decode(errors='replace'),flush=True)
    if p.returncode:raise RuntimeError('call failed, see '+str(audit))
    return p.stdout
action=sys.argv[1]
if action in ('snapshot','preview'):
    label=sys.argv[2]+'_'+tag;assert label.replace('_','').isalnum()
    call(ssh+[f'python3 {REMOTE}/retry_remote_v4.py {action} {label}'],15)
    dest=RUN/'checkpoints';dest.mkdir(exist_ok=True)
    call(scp+['-r','robot-g1:'+REMOTE+'/control/checkpoints/'+label,dest],30)
    live=RUN/'live';live.mkdir(exist_ok=True)
    if action=='snapshot':
        for f in ('body_status.json','grippers.json','visual_manifest.json'):shutil.copy2(dest/label/f,live/f)
    call(scp+['robot-g1:'+REMOTE+'/control/anchor.json',live/'anchor.json'],15)
    print('LOCAL_CHECKPOINT',str(dest/label),flush=True)
elif action in ('move','upload'):
    seq=int(sys.argv[2]);stage=RUN/'stages'/str(seq);meta=json.loads((stage/f'stage_{seq}.json').read_text())
    call(scp+[*stage.iterdir(),'robot-g1:'+REMOTE+'/control/'],25)
    if action=='move':call(ssh+[f'python3 {REMOTE}/retry_remote_v4.py move {seq}'],meta['duration_s']+15)
elif action in ('gripper','hold','status'):
    assert all(str(a).replace('_','').replace('.','').replace('-','').isalnum() for a in sys.argv[1:])
    timeout=150 if action=='gripper' else 15
    call(ssh+['python3 '+REMOTE+'/retry_remote_v4.py '+' '.join(sys.argv[1:])],timeout)
else:raise ValueError(action)
