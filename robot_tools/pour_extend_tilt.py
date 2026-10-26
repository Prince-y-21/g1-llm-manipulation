"""Offline: exploit rotational symmetry about bottle axis, retaining geometry."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import json,sys
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_current_model as m
route=json.loads((r/'route_right_ready.json').read_text());seed=np.array(route[-1]['q'])
for deg in range(72,101,2):
 axis=Rotation.from_euler('x',-np.deg2rad(deg)).as_dcm()[:,2]
 mouth=np.array([.387,.048,.255-(deg-45)*.0017]);tcp=mouth-axis*(m.bottle_height-m.grip_height)
 def residual(q):
  pos,rr=m.tcp_pose(q);t,pts=m.fk(q);body=[]
  for a,b in [(pts[3],pts[5]),(pts[5],pts[6])]:
   for w in np.linspace(0,1,7):
    p=a*(1-w)+b*w;out=max(p[0]-.135,abs(p[1])-.135,p[2]-.245,-.125-p[2]);body.append(max(0.,.008-out)*200)
  for a,b in [(pts[1],pts[3]),(pts[3],pts[5]),(pts[5],pts[6])]:
   for w in np.linspace(0,1,9):
    p=a*(1-w)+b*w;out=max(p[0]-.170,-.130-p[0],abs(p[1])-.140,p[2]-.630,.230-p[2]);body.append(max(0.,.006-out)*500)
  return np.r_[(pos-tcp)*1000,(rr[:,2]-axis)*180,body,(q-seed)*.08]
 sol=least_squares(residual,np.clip(seed,m.lower+1e-7,m.upper-1e-7),bounds=(m.lower,m.upper),max_nfev=500,ftol=1e-10,xtol=1e-10,gtol=1e-10)
 pos,rr=m.tcp_pose(sol.x);pe=float(np.linalg.norm(pos-tcp));ae=float(np.arccos(np.clip(rr[:,2].dot(axis),-1,1)))
 flags,c=m.segment(seed,sol.x,True,True);f,_=m.collision(sol.x,True,True)
 print('AXIS_TILT',deg,'pe',pe,'angle_error',ae,'flags',flags,f,'max_delta',float(np.max(abs(sol.x-seed))),flush=True)
 if pe>.001 or ae>.01 or flags or f:break
 route.append(dict(label='tilt_'+str(deg),q=sol.x.tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=True,grasp=True,flags=flags,endpoint_flag=f,clearance=c,orientation_constraint='bottle_axis; yaw free'))
 seed=sol.x
assert len(route)>29,'No valid tilt extension'
(r/'route_right_extended.json').write_text(json.dumps(route,indent=2))
print('EXTENDED_READY',route[-1]['label'],flush=True)
