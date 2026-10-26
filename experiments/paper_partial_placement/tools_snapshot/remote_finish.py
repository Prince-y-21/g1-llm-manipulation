from pathlib import Path
import sys,time,json
R=Path('/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01');r=R/'control';run='paper_resume_20261025_2041'
sys.path.insert(0,str(R));from retry_evidence import issue,read,atomic
label=sys.argv[1];assert label.replace('_','').isalnum()
s=read(r/'body_status.json');g=read(r/'gripper_left/status.json');seq=s['seq']+1
# Root must view this exact snapshot before invoking: jaws empty, arms at original
# low pose, paper/other tabletop items unchanged and outside the arms' drop space.
issue(r,run,seq,r/'checkpoints'/label,'left',g['release_transaction_seq'],True,True,False,'empty_home_verified_rest')
atomic(r/'request.txt',f'{run} {seq} RELEASE_AT_VERIFIED_REST')
deadline=time.monotonic()+18
while time.monotonic()<deadline:
    if (r/'body_exited').exists():
        print('VERIFIED_REST_BODY_EXITED',flush=True);break
    s=read(r/'body_status.json')
    if s['rejected_seq']==seq:raise RuntimeError('rest release rejected '+json.dumps(s))
    time.sleep(.2)
else:raise TimeoutError('Body still owned; do not kill')
