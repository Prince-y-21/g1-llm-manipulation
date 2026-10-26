"""Read camera files and estimate scene. No robot command output."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import numpy as np, cv2, json
from scipy.spatial.transform import Rotation
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
d=cv2.imread(str(r/'head_depth.png'),-1)*.001
fx,fy,cx,cy=606.174,606.15,332.784,244.873
R=Rotation.from_euler('y',.8307767239493009).as_dcm().dot(np.array([[0,0,1],[-1,0,0],[0,-1,0]]))
t=np.array([.0576235,.01753,.42987])
v,u=np.mgrid[:480,:640]
cam=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
pts=cam.dot(R.T)+t
mask=(u>50)&(u<625)&(v>165)&(v<440)&(d>.3)&(d<1.0)&(pts[:,:,2]<-.03)&(pts[:,:,2]>-.16)
p=pts[mask][::3];rng=np.random.RandomState(12);best=None
for k in range(500):
 a,b,c=p[rng.choice(len(p),3,False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
 if length<1e-8:continue
 n/=length
 if n[2]<0:n=-n
 if n[2]<.94:continue
 err=abs((p-a).dot(n));count=np.sum(err<.004)
 if best is None or count>best[0]:best=(count,n,a,err)
inl=p[best[3]<.004];a=inl.mean(0);_,_,vh=np.linalg.svd(inl-a,full_matrices=False);n=vh[-1];n*=np.sign(n[2])
def ray(u,v,z):return np.array([(u-cx)*z/fx,(v-cy)*z/fy,z]).dot(R.T)+t
def med(u,v,h=4):
 z=d[v-h:v+h+1,u-h:u+h+1];z=z[z>.2]
 return {'pixel':[u,v],'depth_med':float(np.median(z)) if len(z) else None,'depth_range':[float(np.percentile(z,10)),float(np.percentile(z,90))] if len(z) else [],'xyz':ray(u,v,np.median(z)).tolist() if len(z) else None}
def intersect(u,v,height=0):
 direction=ray(u,v,1)-t
 z=(np.dot(n,a-t)+height)/np.dot(n,direction)
 return (t+z*direction).tolist()
out=dict(table_plane_body_normal=n.tolist(),table_plane_body_point=a.tolist(),plane_residual_mm=float(np.sqrt(np.mean(((inl-a).dot(n))**2))*1000),inliers=len(inl),samples={name:med(u,v,h) for name,u,v,h in [('bottle_mouth',364,223,4),('bottle_neck',363,242,4),('bottle_middle',364,303,8),('bottle_base',363,374,4),('cup_rim_rear',143,254,3),('cup_rim_front',145,336,3),('cup_rim_left',89,289,3),('cup_rim_right',196,291,3),('cup_inner',145,293,10)]},bottle_base_table_intersection=intersect(363,374),cup_base_table_intersection=intersect(150,403),front_edge_points=[intersect(u,v) for u,v in [(80,477),(300,456),(600,400)]])
(r/'scene_estimate.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
