"""Execute only a selected prechecked route phase; append immutable events."""
import argparse,hashlib,json,os,time
from pathlib import Path
import numpy as np

ROOT=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
parser=argparse.ArgumentParser()
parser.add_argument('label')
parser.add_argument('--speed',type=float,default=.025)
parser.add_argument('--route',default='offline_route_v2.json')
parser.add_argument('--final-only',action='store_true')
args=parser.parse_args()
assert 0<args.speed<=.06
assert Path(args.route).name==args.route
route_path=ROOT/args.route
route=json.loads(route_path.read_text())
assert all(not w['collision_flags'] for w in route),'Offline route has unresolved flags'
lookup={w['label']:i for i,w in enumerate(route)}
assert args.label in lookup
carrying=args.label.split('__')[0] in ['test_lift','lift','transfer','place']
# Validate each new segment against current feedback as well as nominal route.
model={}
model_source=(ROOT/'task_plan_final.py').read_text().split('\nq0=np.array')[0]
exec(model_source.replace('deg2rad(2)','deg2rad(6)'),model)

def status():
    assert time.time()-(ROOT/'guard_status.json').stat().st_mtime<1,'Stale guard'
    s=json.loads((ROOT/'guard_status.json').read_text())
    assert not s['fault'],s['reason']
    assert not s.get('releasing',False)
    if carrying:
        assert time.time()-(ROOT/'gripper_status.json').stat().st_mtime<1,'Stale gripper'
        g=json.loads((ROOT/'gripper_status.json').read_text())
        assert not g['error'] and g['contact'],'Gripper contact feedback lost'
    return s

def event(kind,**data):
    record=dict(utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),event=kind,phase=args.label,**data)
    with (ROOT/'execution_events.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')

def atomic(name,text):
    (ROOT/(name+'.tmp')).write_text(text);os.replace(str(ROOT/(name+'.tmp')),str(ROOT/name))

def rebase():
    s=status();assert not s['moving']
    request=s['recenter_seq']+1
    atomic('recenter_request.txt',str(request)+'\n')
    deadline=time.monotonic()+4
    while True:
        s=status()
        if s['recenter_seq']>=request:return s
        assert time.monotonic()<deadline,'Static effort transfer not accepted'
        time.sleep(.1)

s=status();assert not s['moving']
start_index=lookup[args.label]
while not args.final_only and start_index>0 and route[start_index-1]['label'].startswith(args.label+'__part'):
    start_index-=1
selected=route[start_index:lookup[args.label]+1]
event('PHASE_START',route_file=args.route,route_sha256=hashlib.sha256(route_path.read_bytes()).hexdigest(),state=s)
for waypoint in selected:
    final=np.array(waypoint['q'])
    start=np.array(s['desired'][22:29]);steps=max(1,int(np.ceil(np.max(np.abs(final-start))/.18)))
    for k in range(1,steps+1):
        # Re-evaluate from actual held posture after each effort transfer.
        s=status();assert not s['moving']
        goal=start+(final-start)*(k/steps)
        current=np.array(s['desired'][22:29]);delta=float(np.max(np.abs(goal-current)))
        assert delta<.245
        duration=max(4.,delta*1.5/args.speed+.25)
        assert duration<30
        for origin in [current,np.array(s['q'][22:29])]:
            flags,_=model['segment'](origin,goal,carrying)
            assert not flags,('Live segment model rejected',flags)
        latest=status()
        assert latest['seq']==s['seq'] and not latest['moving']
        seq=s['seq']+1
        atomic('arm_target.txt',str(seq)+' '+str(duration)+' '+' '.join(map(str,goal))+'\n')
        event('TARGET_SENT',seq=seq,duration=duration,q=goal.tolist())
        print('SENT',args.label,seq,'part',k,'of',steps,'duration',round(duration,2),flush=True)
        deadline=time.monotonic()+duration+6;accepted=False;last_log=0
        while True:
            s=status()
            if s.get('accepted_seq')==seq:accepted=True
            if time.monotonic()-last_log>.5:
                event('FEEDBACK',state=s);last_log=time.monotonic()
            if accepted and not s['moving']:break
            assert time.monotonic()<deadline,'Motion timeout'
            time.sleep(.1)
        assert np.max(np.abs(np.array(s['q'][22:29])-goal))<=.04
        s=rebase()
        event('SEGMENT_VERIFIED',seq=seq,state=s)
        print('VERIFIED',seq,'shoulder_temperature',s['temperatures'][22],flush=True)
event('PHASE_COMPLETE',state=s)
print('PHASE_COMPLETE',args.label,flush=True)
