import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,json,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_safety as safe
m=safe.base
old=json.loads((r/'route_safe_ready.json').read_text());cut=next(i for i,p in enumerate(old) if p['label']=='pour_approach')
out=old[:cut+1];returns=[p for p in old if p['label'].startswith('return_')];seed=np.array(out[-1]['q'])
for deg in range(5,101,5):
 height=float(np.interp(deg,[0,5,10,15,20,25,30,35,40,45,50,55,60,100],[.257,.255,.250,.245,.235,.235,.215,.200,.190,.180,.170,.165,.160,.160]))
 axis=Rotation.from_euler('x',-np.deg2rad(deg)).as_dcm()[:,2];mouth=np.array([.387,.048,height]);tcp=mouth-axis*(m.bottle_height-m.grip_height)
 def res(q):
  pos,rr=m.tcp_pose(q);t,pts=m.fk(q);body=[]
  for a,b in [(pts[3],pts[5]),(pts[5],pts[6])]:
   for w in np.linspace(0,1,7):
    p=a*(1-w)+b*w;outside=max(p[0]-.135,abs(p[1])-.135,p[2]-.245,-.125-p[2]);body.append(max(0.,.008-outside)*200)
  for a,b in [(pts[1],pts[3]),(pts[3],pts[5]),(pts[5],pts[6])]:
   for w in np.linspace(0,1,9):
    p=a*(1-w)+b*w;outside=max(p[0]-.170,-.130-p[0],abs(p[1])-.140,p[2]-.630,.230-p[2]);body.append(max(0.,.006-outside)*500)
  return np.r_[(pos-tcp)*1000,(rr[:,2]-axis)*180,body,(q-seed)*.08]
 sol=least_squares(res,np.clip(seed,m.lower+1e-7,m.upper-1e-7),bounds=(m.lower,m.upper),max_nfev=500,ftol=1e-10,xtol=1e-10,gtol=1e-10)
 pos,rr=m.tcp_pose(sol.x);pe=float(np.linalg.norm(pos-tcp));ae=float(np.arccos(np.clip(rr[:,2].dot(axis),-1,1)));flags,c=safe.segment(seed,sol.x,True,True);f,_=safe.collision(sol.x,True,True)
 print('LOWER_TILT',deg,'pe',pe,'angle',ae,'flags',flags,f,'step',float(np.max(abs(sol.x-seed))),flush=True)
 assert pe<.001 and ae<.01 and not flags and not f,'Invalid lower-pour path'
 out.append(dict(label='tilt_'+str(deg),q=sol.x.tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=True,grasp=True,flags=[],endpoint_flag='',clearance=c,mouth=mouth.tolist(),orientation_constraint='bottle axis; yaw free'))
 seed=sol.x
(r/'route_low_pour_ready.json').write_text(json.dumps(out+returns,indent=2))
(r/'active_control.json').write_text(json.dumps(dict(side='right',indices=[22,29],route='route_low_pour_ready.json',model='pour_safety')))
print('LOW_POUR_READY',len(out),flush=True)
