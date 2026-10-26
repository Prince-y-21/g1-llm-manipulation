"""Opaque label depth face hypotheses; reject sparse or curved observations."""
from cap_cad_geometry import ROOT,OUT
import numpy as np,json
from PIL import Image
from scipy import ndimage
folder=OUT/'empty_calibration_01/images';meta=json.loads((folder/'camera_metadata.json').read_text());cal=json.loads((OUT/'head_registration_candidate.json').read_text())['T_torso_camera'];r=np.array(cal['R']);t=np.array(cal['t']);rng=np.random.default_rng(94);results=[]
for idx in [0,50,100,150,300,400]:
    rgb=np.array(Image.open(folder/f'head_{idx}.jpg'));z=np.array(Image.open(folder/f'depth_{idx}.png'))*meta['depth_scale'];v,u=np.indices(z.shape)
    dark=(rgb.mean(2)<65)&(u>440)&(u<555)&(v>175)&(v<270);labels,n=ndimage.label(dark);counts=np.bincount(labels.ravel());counts[0]=0;capmask=labels==counts.argmax();cu=u[capmask].mean();cv=v[capmask].mean();zz=float(np.median(z[capmask&(z>.3)&(z<.7)]));cap=r@np.array([(cu-meta['cx'])*zz/meta['fx'],(cv-meta['cy'])*zz/meta['fy'],zz])+t
    cloud=np.stack([(u-meta['cx'])*z/meta['fx'],(v-meta['cy'])*z/meta['fy'],z],-1)@r.T+t
    mask=(u>410)&(u<550)&(v>250)&(v<390)&(z>.35)&(z<.75)&(cloud[:,:,2]>-.065)&(cloud[:,:,2]<.035)&(np.linalg.norm(cloud[:,:,:2]-cap[:2],axis=2)<.065)
    points=cloud[mask];best=None
    for _ in range(800):
        pair=points[rng.choice(len(points),2,False),:2];axis=pair[1]-pair[0];length=np.linalg.norm(axis)
        if length<.02:continue
        axis/=length;normal=np.array([-axis[1],axis[0]]);d=float(normal@pair[0]);radius=abs(normal@cap[:2]-d)
        if not .018<radius<.045:continue
        good=abs(points[:,:2]@normal-d)<.002;count=int(good.sum())
        if best is None or count>best[0]:best=(count,good,normal,d,radius)
    if best:
        pts=points[best[1]];_,_,vh=np.linalg.svd(pts[:,:2]-np.mean(pts[:,:2],0),full_matrices=False);normal=vh[-1];angle=float(np.degrees(np.arctan2(normal[1],normal[0]))%90)
        results.append(dict(frame=idx,points=len(points),face_inliers=best[0],inlier_fraction=best[0]/len(points),vertical_face_yaw_mod90_deg=angle,normal_xy=normal.tolist(),distance_to_cap_axis_m=best[4],span_m=float(np.ptp(pts[:,:2]@vh[0])),cap_torso=cap.tolist()))
report=dict(hypotheses_only=True,independent_face_pair_not_verified=True,results=results);(OUT/'bottle_face_hypotheses.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
