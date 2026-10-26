"""Offline image/depth analysis only. Approximate camera-frame geometry."""
from pathlib import Path
import cv2,numpy as np,json,time
r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z')
meta=json.loads((r/'head_metadata.json').read_text());im=cv2.imread(str(r/'head_color.jpg'));z=cv2.imread(str(r/'head_depth.png'),-1)*meta['depth_scale'];v,u=np.indices(z.shape)
xyz=np.stack([(u-meta['cx'])*z/meta['fx'],(v-meta['cy'])*z/meta['fy'],z],axis=-1)
mask=(z>.30)&(z<1.5)&(v<390)&~((u>390)&(u<565)&(v>170))&~((u<300)&(v<200))
points=xyz[mask][::3];rng=np.random.RandomState(21);best=None
for _ in range(600):
 p=points[rng.choice(len(points),3,False)];n=np.cross(p[1]-p[0],p[2]-p[0]);norm=np.linalg.norm(n)
 if norm<1e-8:continue
 n/=norm;inside=abs((points-p[0]).dot(n))<.004;count=int(inside.sum())
 if best is None or count>best[0]:best=(count,inside)
assert best and best[0]>3000,('No robust table plane',best[0] if best else 0)
pts=points[best[1]];center=np.median(pts,axis=0);_,_,vh=np.linalg.svd(pts-center,full_matrices=False);n=vh[-1]
if n.dot(-center)<0:n=-n
residual=np.std((pts-center).dot(n))
gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);dark=((gray<65)&(u>445)&(u<550)&(v>175)&(v<275)).astype(np.uint8)
num,labels,stats,centers=cv2.connectedComponentsWithStats(dark,8);ids=[i for i in range(1,num) if 400<stats[i,cv2.CC_STAT_AREA]<4000];assert ids
i=max(ids,key=lambda k:stats[k,cv2.CC_STAT_AREA]);cmask=labels==i;uv=centers[i];depths=z[cmask&(z>.30)&(z<1.2)]
outline=cv2.findContours(cmask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2];contour=max(outline,key=cv2.contourArea);ellipse=cv2.fitEllipse(contour)
out=dict(analysis='Current static RGB-D, camera frame only; not tool calibration or motion clearance',table_normal_camera=n.tolist(),table_point_camera=center.tolist(),table_fit_residual_m=float(residual),table_inlier_count=best[0],cap_center_pixel=uv.tolist(),cap_ellipse_pixels=dict(center=list(ellipse[0]),axes=list(ellipse[1]),angle=ellipse[2]),cap_valid_depth_samples=len(depths),cap_depth_quantiles=np.percentile(depths,[10,50,90]).tolist() if len(depths) else [],body_face_orientation_estimated=False,cap_thread_pitch_measured=False,cap_opening_torque_measured=False)
if len(depths)>30:
 zz=float(np.median(depths));p=np.array([(uv[0]-meta['cx'])*zz/meta['fx'],(uv[1]-meta['cy'])*zz/meta['fy'],zz]);out.update(cap_center_camera_approx=p.tolist(),cap_height_over_table_approx_m=float(n.dot(p-center)))
cv2.ellipse(im,ellipse,(0,0,255),2);cv2.putText(im,'CAP: image detection only',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2);cv2.imwrite(str(r/'cap_detection.jpg'),im)
(r/'scene_analysis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2),flush=True)
