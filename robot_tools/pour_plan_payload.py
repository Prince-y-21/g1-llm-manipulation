import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,json,numpy as np
from scipy.optimize import least_squares
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_payload_model as m
s=json.loads((r/'guard_status.json').read_text());assert not s['fault'] and not s['moving'];g=json.loads((r/'gripper_status.json').read_text());predicted=os.environ.get('POUR_PLAN_PREDICTED')=='1';assert g['contact'] or predicted
q0=np.array(json.loads((r/'retry_approach_ready.json').read_text())[-1]['q'] if predicted else s['q'][22:29]);mouth0,axis0=m.payload(q0);N=m.normal;direction=np.array([1.,0.,0.]);direction-=N*direction.dot(N);direction/=np.linalg.norm(direction)
out=[]
def ik(mouth,axis,seed):
 def residual(q):
  p,a=m.payload(q);t,pts=m.fk(q);pen=[]
  for aa,bb in [(pts[3],pts[5]),(pts[5],pts[6])]:
   pp=aa*(1-np.linspace(0,1,7)[:,None])+bb*np.linspace(0,1,7)[:,None]
   outside=np.maximum.reduce([pp[:,0]-.135,abs(pp[:,1])-.135,pp[:,2]-.245,-.125-pp[:,2]])
   pen.extend(np.maximum(0,.008-outside)*200)
  for aa,bb in [(pts[1],pts[3]),(pts[3],pts[5]),(pts[5],pts[6])]:
   pp=aa*(1-np.linspace(0,1,9)[:,None])+bb*np.linspace(0,1,9)[:,None]
   outside=np.maximum.reduce([pp[:,0]-.170,-.130-pp[:,0],abs(pp[:,1])-.140,pp[:,2]-.630,.230-pp[:,2]])
   pen.extend(np.maximum(0,.006-outside)*500)
  margin_cost=np.r_[np.maximum(0,.12-(q-m.lower))*2,np.maximum(0,.12-(m.upper-q))*2]
  return np.r_[(p-mouth)*1000,(a-axis)*180,pen,margin_cost,q[4:]*.10,(q-seed)*.10]
 sol=least_squares(residual,np.clip(seed,m.lower+1e-7,m.upper-1e-7),bounds=(m.lower,m.upper),max_nfev=350,ftol=1e-10,xtol=1e-10,gtol=1e-10)
 p,a=m.payload(sol.x);pe=float(np.linalg.norm(p-mouth));ae=float(np.arccos(np.clip(a.dot(axis),-1,1)))
 if pe>.001 or ae>.01:raise RuntimeError('PAYLOAD_IK '+str([pe,ae,sol.x.tolist()]))
 return sol.x
def add(label,q,carried=True,upright=False):
 flags,c=m.segment(out[-1]['q'],q,carried,True) if out else ([],10.);f,_=m.collision(q,carried,True);p,a=m.payload(q)
 if upright and out:
  max_tilt=max(float(np.arccos(np.clip(m.payload(np.array(out[-1]['q'])*(1-w)+q*w)[1].dot(N),-1,1))) for w in np.linspace(0,1,81));assert max_tilt<np.deg2rad(6),('Transport tilt',max_tilt)
 node=dict(label=label,q=np.array(q).tolist(),carried=carried,grasp=True,flags=flags,endpoint_flag=f,clearance=c,mouth=p.tolist(),bottle_axis=a.tolist(),upright=upright)
 out.append(node);(r/('route_payload_predicted_partial.json' if predicted else 'route_payload_partial.json')).write_text(json.dumps(out,indent=2));print('PAYLOAD',label,flags,f,'mouth',np.round(p,4).tolist(),flush=True);assert not flags and not f,'Geometry rejected '+label
def pose(label,p,a,upright=False):
 seed=np.array(out[-1]['q']);q=ik(np.array(p),a,seed);add(label,q,True,upright)
add('supported_grasp',q0)
pose('test_lift',mouth0+N*.030,N,True)
pose('lift_mid',mouth0+N*.100,N,True)
high=mouth0+N*max(.16,float((m.cup+N*(m.H+.060)-mouth0).dot(N)))
pose('lift_high',high,N,True)
aim=m.cup-direction*.015;upper=aim+N*(m.H+.060)
start=high.copy()
for i in range(1,5):pose('transfer_'+str(i),start+(upper-start)*(i/4),N,True)
pose('pour_approach',aim+N*(m.H+.035),N,True)
angles=list(range(5,61,5))+list(range(62,101,2))
for deg in angles:
 rad=np.deg2rad(deg);axis=N*np.cos(rad)+direction*np.sin(rad);ss=np.linspace(0,m.H,101);radii=m.radius(ss);inside=.015+ss*np.sin(rad)<m.cup_radius+radii+.010
 minheight=max(.110,float(np.max((ss*np.cos(rad)+radii*np.sin(rad)+.015)[inside]))+.015)
 seed=np.array(out[-1]['q']);last_error=''
 for attempt in range(10):
  height=minheight+attempt*.010;mouth=aim+N*height
  try:q=ik(mouth,axis,seed)
  except RuntimeError as e:last_error=str(e);continue
  flags,c=m.segment(seed,q,True,True);f,_=m.collision(q,True,True)
  if not flags and not f:break
  last_error=str([flags,f])
 else:raise RuntimeError('No safe tilt '+str(deg)+' '+last_error)
 add('tilt_'+str(deg),q)
# Reverse carrying segments are the same checked geometry. Placement returns to
# the observed supported grasp instead of the uncalibrated original target.
ref=dict(out[0]);ref['label']='putdown_reference';ref['reference_only']=True;ref['required_predecessor']='test_lift';out.append(ref)
(r/('route_payload_predicted_candidate.json' if predicted else 'route_payload_candidate.json')).write_text(json.dumps(out,indent=2))
# putdown_reference is an endpoint reference; its direct edge from final tilt is
# never an authorized shortcut. Recovery executes the recorded reverse phases.
(r/('route_payload_predicted_ready.json' if predicted else 'route_payload_ready.json')).write_text(json.dumps(out,indent=2))
if not predicted:(r/'active_control.json').write_text(json.dumps(dict(side='right',indices=[22,29],route='route_payload_ready.json',model='pour_payload_model')))
print('PAYLOAD_ROUTE_READY',len(out),flush=True)
