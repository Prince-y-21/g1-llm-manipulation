import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,json,heapq,itertools
import numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r))
import pour_left_model as m
route=json.loads((r/'route_left_candidate.json').read_text());startidx=next(i for i,p in enumerate(route) if p['label']=='normalize_behind_edge');goalidx=next(i for i,p in enumerate(route) if p['label']=='raise_clear_cup')
base=np.array(route[startidx]['q']);goal=np.array(route[goalidx]['q'])
axes=[[base[1],.45,.60,.75,.90],[base[3],1.05,.75,.45,.20,0.],[base[5],-.40,-.70,-1.0,-1.30]]
def qnode(node):
 q=base.copy();q[1]=axes[0][node[0]];q[3]=axes[1][node[1]];q[5]=axes[2][node[2]];return q
valid={node: not m.collision(qnode(node))[0] for node in itertools.product(*[range(len(a)) for a in axes])}
print('VALID_GRID',sum(valid.values()),'/',len(valid),flush=True)
def seg(a,b,n=21):
 return all(not m.collision(a*(1-w)+b*w)[0] for w in np.linspace(0,1,n))
start=(0,0,0);assert valid[start]
queue=[(0.,start)];cost={start:0.};parent={};end=None;cache={};iterations=0
while queue:
 c,node=heapq.heappop(queue)
 if c>cost[node]+1e-9:continue
 q=qnode(node);iterations+=1;pos,_=m.tcp_pose(q)
 if pos[2]>-.015 and seg(q,goal,41):end=node;break
 for axis in range(3):
  for s in [-1,1]:
   n=list(node);n[axis]+=s;n=tuple(n)
   if n not in valid or not valid[n]:continue
   nq=qnode(n);nc=c+float(np.linalg.norm(nq-q))
   if nc>=cost.get(n,1e9):continue
   key=tuple(sorted([node,n]))
   if key not in cache:cache[key]=seg(q,nq)
   if not cache[key]:continue
   cost[n]=nc;parent[n]=node;heapq.heappush(queue,(nc,n))
assert end is not None,'No checked prefix connector'
path=[end]
while path[-1]!=start:path.append(parent[path[-1]])
path.reverse();out=route[:startidx+1]
for k,node in enumerate(path[1:]):
 q=qnode(node);pos,rr=m.tcp_pose(q);out.append(dict(label='clear_edge_'+str(k+1),q=q.tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=False,grasp=False,flags=[]))
out+=route[goalidx:]
for i,p in enumerate(out):
 flags,c=m.segment(out[i-1]['q'],p['q'],p['carried'],p['grasp']) if i else ([],10)
 p['flags']=flags;p['clearance']=c
 print('FINAL',p['label'],flags,flush=True)
assert not any(p['flags'] for p in out),'Unsafe final path'
(r/'route_left_ready.json').write_text(json.dumps(out,indent=2))
print('READY',len(out),'visited',iterations,flush=True)
