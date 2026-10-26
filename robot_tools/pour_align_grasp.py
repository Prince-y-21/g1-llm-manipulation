from pathlib import Path
import sys,json,numpy as np,cv2
from scipy.spatial.transform import Rotation
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_safety as safe
m=safe.base;s=json.loads((r/'guard_status.json').read_text());assert not s['moving'] and not s['fault']
depth=cv2.imread(str(r/'head_depth.png'),-1)*.001;patch=depth[190:201,470:481];z=patch[(patch>.35)&(patch<.65)];assert len(z)>20
med=float(np.median(z));spread=float(np.percentile(z,90)-np.percentile(z,10));assert spread<.025
R=Rotation.from_euler('y',.8307767239493009).as_dcm().dot(np.array([[0,0,1],[-1,0,0],[0,-1,0]]));t=np.array([.0576235,.01753,.42987]);u,v=475,195
mouth=R.dot([(u-332.784)*med/606.174,(v-244.873)*med/606.15,med])+t
assert np.linalg.norm(mouth[:2]-m.bottle_xy)<.035,'Unexpected bottle shift'
goal=np.array([mouth[0]+.002,mouth[1],m.table_z(mouth[0],mouth[1])+m.grip_height]);q=m.ik(goal,np.eye(3),np.array(s['q'][22:29]))
for origin in [np.array(s['q'][22:29]),np.array(s['desired'][22:29])]:
 flags,c=safe.segment(origin,q,False,True);assert not flags,flags
cfg=json.loads((r/'active_control.json').read_text());route=json.loads((r/cfg['route']).read_text());p=next(p for p in route if p['label']=='grasp');p['q']=q.tolist();p['tcp']=goal.tolist();p['alignment']=dict(mouth_pixel=[u,v],mouth_xyz=mouth.tolist(),median_depth=med,depth_spread=spread)
idx=route.index(p);flags,c=safe.segment(q,np.array(route[idx+1]['q']),True,True);assert not flags,flags
(r/'grasp_alignment.json').write_text(json.dumps(p,indent=2));(r/'route_aligned_ready.json').write_text(json.dumps(route,indent=2));cfg['route']='route_aligned_ready.json';(r/'active_control.json').write_text(json.dumps(cfg))
print(json.dumps(p,indent=2))
