"""Approximate bottle-in-hand and fresh scene calibration from a supported grasp."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,json,numpy as np,cv2,hashlib,time,shutil
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_current_model as model
scene_only='--scene-only' in sys.argv
s=json.loads((r/'guard_status.json').read_text());g=json.loads((r/'gripper_status.json').read_text());assert not s['moving'] and not s['fault'];assert scene_only or g['contact']
ns={};exec((r/'pour_scene.py').read_text().split('out=dict')[0],ns)
n=ns['n'];plane=ns['a'];pts=ns['pts'];u=ns['u'];v=ns['v'];d=ns['d']
# Neck-rim depth cluster; avoid treating a transparent-hole background as neck.
reference=cv2.imread(str(r/'snapshots'/'20261019T114807Z_grip_contact'/'head_color.jpg'),0);imagegray=cv2.imread(str(r/'head_color.jpg'),0);template=reference[141:173,450:487];bestneck=None
for scale in [.90,.95,1.,1.05,1.10]:
 temp=cv2.resize(template,None,fx=scale,fy=scale);search=imagegray[80:250,390:550];response=cv2.matchTemplate(search,temp,cv2.TM_CCOEFF_NORMED);_,score,_,loc=cv2.minMaxLoc(response)
 if bestneck is None or score>bestneck[0]:bestneck=(score,int(390+loc[0]+18*scale),int(80+loc[1]+15*scale),scale)
assert bestneck[0]>.60,('Neck-template uncertainty',bestneck)
_,mu,mv,neck_scale=bestneck;zs=d[mv-5:mv+6,mu-5:mu+6];zs=zs[(zs>.30)&(zs<.65)]
hist,bins=np.histogram(zs,np.arange(.30,.651,.005));center=(bins[np.argmax(hist)]+bins[np.argmax(hist)+1])/2
cluster=zs[abs(zs-center)<.008];assert len(cluster)>15
depth=float(np.median(cluster));mouth=ns['ray'](mu,mv,depth);height=float(n.dot(mouth-plane));assert .17<height<.23,('Invalid bottle height',height)
# Cup: fit a circular rim/upper-wall cross-section to valid RGB-D points.
h=np.sum((pts-plane)*n,axis=2)
mask=(u>205)&(u<330)&(v>205)&(v<330)&(d>.3)&(d<.8)&(h>.105)&(h<.155)
cp=pts[mask];ch=h[mask];assert len(cp)>35
rng=np.random.RandomState(21);best=None
for _ in range(300):
 ids=rng.choice(len(cp),3,False);xy=cp[ids,:2]
 try:sol=np.linalg.solve(np.c_[2*xy,np.ones(3)],np.sum(xy*xy,axis=1))
 except np.linalg.LinAlgError:continue
 radius2=sol[2]+np.sum(sol[:2]**2)
 if not .035**2<radius2<.060**2 or np.linalg.norm(sol[:2]-[.387,.071])>.045:continue
 errors=abs(np.linalg.norm(cp[:,:2]-sol[:2],axis=1)-np.sqrt(radius2));count=int(np.sum(errors<.004))
 if best is None or count>best[0]:best=(count,errors<.004)
assert best is not None and best[0]>25
xy=cp[best[1],:2];sol=np.linalg.lstsq(np.c_[2*xy,np.ones(len(xy))],np.sum(xy*xy,axis=1),rcond=None)[0];radius=float(np.sqrt(sol[2]+np.sum(sol[:2]**2)));cupxy=sol[:2]
cupheight=float(np.percentile(ch[best[1]],93));cupz=float(plane[2]-(n[0]*(cupxy[0]-plane[0])+n[1]*(cupxy[1]-plane[1]))/n[2]+cupheight/n[2])
# Table-front line from the long visible left section, excluding the hand.
img=cv2.imread(str(r/'head_color.jpg'));edge=cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),(5,5),0),40,100);edge[:330]=0;edge[:,380:]=0
lines=cv2.HoughLinesP(edge,1,np.pi/180,70,minLineLength=140,maxLineGap=35);candidates=[]
if lines is not None:
 for x1,y1,x2,y2 in lines[:,0]:
  if abs(x2-x1)<100:continue
  slope=(y2-y1)/(x2-x1);intercept=y1-slope*x1
  if -.4<slope<-.1 and 430<intercept<520:candidates.append((abs(x2-x1),slope,intercept))
assert candidates,'No reliable desk edge'
_,slope,intercept=max(candidates);edgepts=np.array([ns['intersect'](uu,slope*uu+intercept) for uu in [60,200,340]])
edgecoef=np.polyfit(edgepts[:,1],edgepts[:,0],1)
t,_=model.fk(s['q'][22:29]);localmouth=t[:3,:3].T.dot(mouth-t[:3,3]);localaxis=t[:3,:3].T.dot(n)
if not scene_only:assert .09<localmouth[0]<.28 and abs(localmouth[1])<.12 and -.02<localmouth[2]<.20
cal=dict(table_normal=n.tolist(),table_point=plane.tolist(),table_edge=edgecoef.tolist(),table_fit_residual_mm=ns['out']['plane_residual_mm'] if 'out' in ns else float(np.std((ns['inl']-plane).dot(n))*1000),mouth_pixel=[mu,mv],neck_template_score=bestneck[0],mouth_depth=depth,mouth_body=mouth.tolist(),mouth_local=localmouth.tolist(),axis_local=localaxis.tolist(),bottle_height=height,cup=[float(cupxy[0]),float(cupxy[1]),cupz],cup_radius=radius+.003,cup_fit_inliers=best[0],cup_height=cupheight,grasp_q=s['q'][22:29],grasp_state=s,gripper_state=g,calibration_kind='Approximate supported-grasp RGB-D/FK registration; not metrology',image_sha256=hashlib.sha256((r/'head_color.jpg').read_bytes()).hexdigest())
old=json.loads((r/'payload_calibration.json').read_text());history=r/'calibration_history';history.mkdir(exist_ok=True);shutil.copy2(str(r/'payload_calibration.json'),str(history/(str(time.time_ns())+'.json')))
if scene_only:
 for key in ['mouth_local','axis_local','grasp_q']:cal[key]=old[key]
 cal['calibration_kind']='Fresh supported-bottle scene; retained successful wrist/bottle registration'
else:
 cal['registration_delta_m']=float(np.linalg.norm(np.array(cal['mouth_local'])-old['mouth_local']))
 assert cal['registration_delta_m']<.030,'Unexpected tool/payload registration change'
(r/'payload_calibration.json').write_text(json.dumps(cal,indent=2))
if not scene_only:(r/'grasp_table_reference.json').write_text(json.dumps(s,indent=2))
print(json.dumps({k:val for k,val in cal.items() if k not in ['grasp_state','gripper_state']},indent=2))
