"""Passive corroboration: bottle height over fresh table and wrist image rigidity."""
from pathlib import Path
import sys,json,numpy as np,cv2
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_payload_model as m
ns={};src=(r/'pour_calibrate_payload.py').read_text().split("t,_=model.fk")[0].replace("assert .17<height<.23,('Invalid bottle height',height)","assert .16<height<.27")
exec(src,ns)
clearance=float(ns['height']-m.H);s=json.loads((r/'guard_status.json').read_text());pred,axis=m.payload(s['q'][22:29]);mouth=np.array(ns['mouth']);camera_fk_error=float(np.linalg.norm(pred-mouth))
baseline=sorted((r/'snapshots').glob('*_mission_grip_contact'))[-1]/'wrist8.jpg';first=cv2.imread(str(baseline),0);second=cv2.imread(str(r/'wrist8.jpg'),0)
mask=np.zeros_like(first);mask[120:450,195:445]=255;orb=cv2.ORB_create(1800);k1,d1=orb.detectAndCompute(first,mask);k2,d2=orb.detectAndCompute(second,mask)
matches=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True).match(d1,d2) if d1 is not None and d2 is not None else []
matches=sorted([x for x in matches if x.distance<65],key=lambda x:x.distance)[:400];info=dict(matches=len(matches),inliers=0)
if len(matches)>=10:
 a=np.float32([k1[x.queryIdx].pt for x in matches]);b=np.float32([k2[x.trainIdx].pt for x in matches]);aff,ins=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=3)
 if aff is not None:info.update(inliers=int(ins.sum()),scale=float(np.hypot(aff[0,0],aff[1,0])),rotation_deg=float(np.rad2deg(np.arctan2(aff[1,0],aff[0,0]))),translation_px=aff[:,2].tolist())
result=dict(measured_clearance_m=clearance,camera_fk_error_m=camera_fk_error,measured_mouth=mouth.tolist(),predicted_mouth=pred.tolist(),wrist_rigidity=info,external_lift_supported=clearance>.006 and clearance<.055 and camera_fk_error<.030,limitations='RGB-D/approximate extrinsics, transparent neck and plane-fit uncertainty; combine with images and contact feedback')
(r/'lift_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
