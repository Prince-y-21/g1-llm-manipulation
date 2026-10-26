"""Evidence capture / explicit upper-layer attestations. --submit-verb requests body action.
Run on the robot server to keep all freshness times in server CLOCK_MONOTONIC.
Capture does not approve action. Issue requires a human/model inspection of copied images.
"""
from pathlib import Path
import argparse,csv,hashlib,json,time,shutil,os,threading

def atomic(p,text):
    temp=p.with_name(p.name+f'.{os.getpid()}.{threading.get_ident()}.tmp');temp.write_text(text,encoding='utf-8');temp.replace(p)
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def digest(obj):return hashlib.sha256(json.dumps(obj,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def state_version(s):return dict(seq=s['seq'],qgoal=s['qgoal'],g=s['g'])
def snapshot(r,run,label):
    assert label.replace('_','').isalnum()
    s=read(r/'body_status.json');assert not s['moving'] and not s['releasing'] and time.monotonic()-s['steady']<1
    out=r/'checkpoints'/label;out.mkdir(parents=True,exist_ok=False);images={}
    for name in ('head','wrist0','wrist8'):
        latest=list(csv.DictReader((r/'images'/(name+'_frames.csv')).open()))[-1]
        stamp=float(latest['steady_end']);assert time.monotonic()-stamp<2.5
        filename=name+'_'+latest['seq']+'.jpg';shutil.copy2(r/'images'/filename,out/(name+'.jpg'))
        images[name]=dict(source=filename,steady=stamp,sha256=hashlib.sha256((out/(name+'.jpg')).read_bytes()).hexdigest())
        if name=='head':shutil.copy2(r/'images'/('depth_'+latest['seq']+'.png'),out/'depth.png')
    for filename in ('camera_metadata.json',):shutil.copy2(r/'images'/filename,out/filename)
    (out/'body_status.json').write_text(json.dumps(s,indent=2));(out/'grippers.json').write_text(json.dumps({side:read(r/('gripper_'+side)/'status.json') for side in ('left','right')},indent=2))
    manifest=dict(run=run,snapshot_id=label,captured_monotonic=time.monotonic(),body_version=state_version(s),images=images)
    (out/'visual_manifest.json').write_text(json.dumps(manifest,indent=2))
    after=read(r/'body_status.json');assert state_version(after)==state_version(s) and not after['moving'],'body changed during snapshot'
    print(json.dumps(dict(snapshot=str(out),manifest=manifest)),flush=True)

def issue(r,run,request_seq,snap,release_side,release_tx,visual_released,verified_rest,path_checked,evidence_id):
    now=time.monotonic();manifest=read(snap/'visual_manifest.json');s=read(r/'body_status.json')
    assert visual_released and (verified_rest or path_checked),'Explicit visual/plan assertion required'
    assert manifest['run']==run and time.monotonic()-s['steady']<1 and not s['moving'] and not s['releasing']
    assert state_version(s)==manifest['body_version'],'Body request/target changed since inspected snapshot'
    stamps=[]
    for name in ('head','wrist0'):
        im=manifest['images'][name];stamps.append(im['steady'])
        assert hashlib.sha256((snap/(name+'.jpg')).read_bytes()).hexdigest()==im['sha256']
    visual=min(stamps);assert 0<=now-visual<=60,'Visual evidence protocol window expired; do not falsify timestamps'
    statuses={side:read(r/('gripper_'+side)/'control_ack.json') for side in ('left','right')}
    active=statuses[release_side]
    assert active['run']==run and active['protocol_version']>=2 and active['side']==release_side
    assert 0<=now-active['steady']<1 and active['controllable'] and not active['moving'] and active['release_confirmed']
    assert active['release_transaction_seq']==release_tx and active['done_seq']>=release_tx
    # Rest release shuts down body plus both grippers; both require explicit confirmations.
    if verified_rest:
        for side,gs in statuses.items():
            assert gs['run']==run and 0<=now-gs['steady']<1 and gs['controllable'] and not gs['moving'] and gs['release_confirmed'] and gs['release_transaction_seq']>0
    # Recheck after all reads. Any intervening body movement invalidates the attestation.
    latest=read(r/'body_status.json');assert state_version(latest)==state_version(s) and not latest['moving']
    gs=read(r/('gripper_'+release_side)/'control_ack.json')
    assert gs['release_transaction_seq']==release_tx and gs['release_confirmed'] and gs['controllable'] and not gs['moving']
    issued=time.monotonic();assert issued-visual<=60
    assert evidence_id and evidence_id!='UNVERIFIED' and ' ' not in evidence_id
    evidence=dict(run=run,request_seq=request_seq,body_seq=s['seq'],issued_monotonic=issued,visual_monotonic=visual,snapshot_id=manifest['snapshot_id'],snapshot_manifest_sha256=digest(manifest),body_version=state_version(s),release_side=release_side,release_transaction_seq=release_tx,gripper_versions={side:dict(seq=v['seq'],release_transaction_seq=v['release_transaction_seq']) for side,v in statuses.items()},visually_released=True,verified_rest=verified_rest,path_checked=path_checked,evidence_id=evidence_id)
    atomic(r/f'verified_{request_seq}.json',json.dumps(evidence,indent=2))
    # Body loader consumes each monotonically increasing request sequence once.
    line=f'PAPER_EVIDENCE_V1 {run} {request_seq} {s["seq"]} {issued:.9f} {visual:.9f} {release_tx} {release_side} 1 1 {int(verified_rest)} {int(path_checked)} {evidence_id}'
    atomic(r/f'verified_{request_seq}.txt',line);print(json.dumps(evidence),flush=True)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=('snapshot','issue'));ap.add_argument('phase',type=Path);ap.add_argument('run');ap.add_argument('--label');ap.add_argument('--request-seq',type=int);ap.add_argument('--snapshot',type=Path);ap.add_argument('--release-side',choices=('left','right'),default='left');ap.add_argument('--release-tx',type=int);ap.add_argument('--visually-released',action='store_true');ap.add_argument('--verified-rest',action='store_true');ap.add_argument('--path-checked',action='store_true');ap.add_argument('--evidence-id');ap.add_argument('--submit-verb',choices=('RECOVER','RELEASE_AT_VERIFIED_REST'));a=ap.parse_args()
    if a.action=='snapshot':snapshot(a.phase,a.run,a.label)
    else:
        if a.submit_verb=='RELEASE_AT_VERIFIED_REST':assert a.verified_rest
        if a.submit_verb=='RECOVER':assert a.path_checked
        issue(a.phase,a.run,a.request_seq,a.snapshot,a.release_side,a.release_tx,a.visually_released,a.verified_rest,a.path_checked,a.evidence_id)
        if a.submit_verb:
            atomic(a.phase/'request.txt',f'{a.run} {a.request_seq} {a.submit_verb}')
            print('BODY_REQUEST_SUBMITTED',a.submit_verb,a.request_seq,flush=True)
if __name__=='__main__':main()
