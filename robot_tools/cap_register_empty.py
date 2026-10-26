"""Multi-pose CAD registration candidate; preserves calibration assumptions."""
from cap_cad_geometry import *
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw
import csv

run=OUT/'empty_calibration_01';imdir=run/'images';sc=Scene();rng=np.random.default_rng(52)
trace=np.loadtxt(run/'feedback.csv',delimiter=',',skiprows=1).reshape(-1,29,14)
frames=list(csv.DictReader((imdir/'head_frames.csv').open()));meta=json.loads((imdir/'camera_metadata.json').read_text())
depth0=np.array(Image.open(imdir/'depth_0.png'))*meta['depth_scale']
rp=ROOT/'observations/cap_open_20261021';plan=json.loads((OUT/'calibration_profile.json').read_text());times=np.cumsum(plan['durations']);target=[(times[k-1] if k else 0)+plan['durations'][k]/2 for k in range(len(times)) if plan['durations'][k]<1.]
model_geoms=[i for i,n in enumerate(sc.gnames) if n.startswith('left_') and any(s in n for s in ['dex1','wrist_roll','wrist_pitch','wrist_yaw','elbow']) and 'camera' not in n]
templates={}
for gid in model_geoms:
    meshid=sc.m.geom_dataid[gid]
    if meshid<0:continue
    a=sc.m.mesh_vertadr[meshid];n=sc.m.mesh_vertnum[meshid];v=sc.m.mesh_vert[a:a+n]
    a=sc.m.mesh_faceadr[meshid];n=sc.m.mesh_facenum[meshid];f=sc.m.mesh_face[a:a+n];tri=v[f]
    areas=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1);p=areas/areas.sum();chosen=tri[rng.choice(n,5000,p=p)];r=rng.random((5000,2));bad=r.sum(1)>1;r[bad]=1-r[bad]
    templates[gid]=chosen[:,0]+r[:,0,None]*(chosen[:,1]-chosen[:,0])+r[:,1,None]*(chosen[:,2]-chosen[:,0])
poses=[];selected=set()
for phase in target:
    candidates=np.where((abs(trace[:,0,2]-phase)<.28)&(trace[:,0,4]==0))[0]
    if not len(candidates):continue
    ti=candidates[np.argmin(np.max(abs(trace[candidates,15:,7]),axis=1))];now=trace[ti,0,0]
    row=min(frames,key=lambda f:abs((float(f['steady_start'])+float(f['steady_end']))/2-now));seq=int(row['seq'])
    if seq in selected:continue
    selected.add(seq);when=(float(row['steady_start'])+float(row['steady_end']))/2;ti=np.argmin(abs(trace[:,0,0]-when));q=trace[ti,:,6]
    z=np.array(Image.open(imdir/f'depth_{seq}.png'))*meta['depth_scale'];rgb=np.array(Image.open(imdir/f'head_{seq}.jpg'));y,x=np.indices(z.shape)
    mask=(z>.17)&(z<.85)&(x<390)&((depth0==0)|(depth0-z>.035))&(rgb.mean(2)<220)
    coords=np.argwhere(mask);cloud=np.c_[(coords[:,1]-meta['cx'])*z[mask]/meta['fx'],(coords[:,0]-meta['cy'])*z[mask]/meta['fy'],z[mask]]
    if len(cloud)<100:continue
    if len(cloud)>2200:cloud=cloud[rng.choice(len(cloud),2200,replace=False)]
    sc.setq(q,-.020);torso=sc.m.body('torso_link').id;rt=sc.d.xmat[torso].reshape(3,3);pt=sc.d.xpos[torso];parts=[]
    for gid,v in templates.items():
        world=v@sc.d.geom_xmat[gid].reshape(3,3).T+sc.d.geom_xpos[gid];parts.append((world-pt)@rt)
    model=np.concatenate(parts);poses.append(dict(seq=seq,phase=float(trace[ti,0,2]),cloud=cloud,model=model,tree=cKDTree(model),q=q))

r0=Rotation.from_euler('y',.830776724).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
t0=np.array([.0576235,.01753,.42987]);r=r0.copy();t=t0.copy();history=[]
assert len(poses)>=4,('insufficient visible poses',len(poses))
# Hold out one wrist orientation from the fit; keep a second duplicate for repeatability.
holdout=len(poses)//2;fit=[p for i,p in enumerate(poses) if i!=holdout]
for step in range(55):
    src=[];dst=[];weights=[]
    for p in fit:
        transformed=p['cloud']@r.T+t;d,idx=p['tree'].query(transformed)
        cutoff=min(.12 if step<8 else .05,max(.008,float(np.quantile(d,.65))));good=d<cutoff
        if good.sum()<60:continue
        src.append(p['cloud'][good]);dst.append(p['model'][idx[good]]);weights.append(np.minimum(1,.008/np.maximum(d[good],.0001)))
    assert src,'No registration correspondences'
    a=np.concatenate(src);b=np.concatenate(dst);w=np.concatenate(weights);w/=w.sum();ac=np.sum(a*w[:,None],0);bc=np.sum(b*w[:,None],0)
    u,s,vh=np.linalg.svd(((a-ac)*w[:,None]).T@(b-bc));rn=vh.T@np.diag([1,1,np.linalg.det(vh.T@u.T)])@u.T;tn=bc-rn@ac
    error=float(np.sqrt(np.sum(w*np.sum((a@rn.T+tn-b)**2,axis=1))));history.append(error)
    if np.linalg.norm(tn-t)+np.linalg.norm(rn-r)<1e-7:r,t=rn,tn;break
    r,t=rn,tn

results=[]
for i,p in enumerate(poses):
    d0,_=p['tree'].query(p['cloud']@r0.T+t0);d,_=p['tree'].query(p['cloud']@r.T+t)
    results.append(dict(seq=p['seq'],path_phase=p['phase'],points=len(d),holdout=i==holdout,before_median_m=float(np.median(d0)),after_quantiles_m=np.quantile(d,[.25,.5,.75,.9]).tolist(),fraction_within_10mm=float(np.mean(d<.01))))
    rgb=Image.open(imdir/f"head_{p['seq']}.jpg").convert('RGB');draw=ImageDraw.Draw(rgb)
    for rr,tt,color in [(r0,t0,(0,70,255)),(r,t,(255,70,0))]:
        camera=(p['model']-tt)@rr;valid=camera[:,2]>.1;camera=camera[valid][::12];uv=camera[:,:2]/camera[:,2,None]*[meta['fx'],meta['fy']]+[meta['cx'],meta['cy']]
        for x,y in uv:
            if 0<=x<640 and 0<=y<480:draw.point((int(x),int(y)),fill=color)
    rgb.save(OUT/f"registration_overlay_{p['seq']}.jpg")
report=dict(status='registration_candidate_not_accepted_as_physical_calibration',assumptions=['Reference Dex1 mount and left opening remain assumed','Depth on black fingers can be sparse','Trimmed ICP residual is not end-to-end TCP accuracy'],pose_count=len(poses),T_torso_camera=dict(R=r.tolist(),t=t.tolist()),translation_change_m=float(np.linalg.norm(t-t0)),rotation_change_rad=float(Rotation.from_matrix(r0.T@r).magnitude()),residual_history=history,poses=results)
(OUT/'head_registration_candidate.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='residual_history'},indent=2))
