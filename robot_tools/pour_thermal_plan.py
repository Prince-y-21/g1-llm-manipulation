from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_payload_model as m
s=json.loads((r/'guard_status.json').read_text());assert s['fault'] and s['reason'].startswith('TEMPERATURE')
g=json.loads((r/'gripper_status.json').read_text());assert not g['contact'] and g['q']>5.0
old=json.loads((r/'route_safe_ready.json').read_text());by={p['label']:p for p in old};current=np.array(s['q'][22:29]);path=[]
labels=['bridge','outboard','return_safe_level','return_fold'];index=0
while index<len(labels):
 valid=[]
 for j in range(len(labels)-1,index-1,-1):
  q=np.array(by[labels[j]]['q']);flags,c=m.segment(current,q,False,True)
  # Initial egress must move the open hand backwards from the bottle.
  xx=[m.payload(current*(1-w)+q*w)[0][0] for w in np.linspace(0,1,31)]
  backwards=min(np.diff(xx))<0 and max(xx)<xx[0]+.008 if not path else True
  print('RECOVERY_CANDIDATE',labels[j],flags,'backwards',backwards,flush=True)
  if not flags and backwards:valid.append(j);break
 assert valid,'No geometry-checked empty-arm recovery path'
 j=valid[0];goal=np.array(by[labels[j]]['q']);path.append(dict(label=labels[j],q=goal.tolist()));current=goal;index=j+1
targets=[];current=np.array(s['q'][22:29]);seq=s['seq']
for p in path:
 goal=np.array(p['q']);start=current.copy();steps=max(1,int(np.ceil(np.max(abs(goal-start))/.14)))
 for k in range(1,steps+1):
  q=start+(goal-start)*(k/steps);delta=float(np.max(abs(q-current)));duration=max(3.1,delta*1.5/.05+.2);seq+=1
  targets.append(dict(seq=seq,duration=duration,q=q.tolist(),label=p['label']));current=q
assert len(targets)<=40
(r/'thermal_recovery_route.json').write_text(json.dumps(dict(source=s,targets=targets,purpose='Unload and withdraw empty right arm only; no pouring or payload transport'),indent=2))
(r/'thermal_whitelist.txt').write_text('\n'.join(str(t['seq'])+' '+str(t['duration'])+' '+' '.join(map(str,t['q'])) for t in targets)+'\n')
print('THERMAL_RECOVERY_PLAN_READY',len(targets),'nominal_s',sum(t['duration']+2 for t in targets),flush=True)
