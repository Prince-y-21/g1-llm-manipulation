"""Endpoint feasibility, not an execution plan or a contact permission."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from PIL import Image
from scipy import ndimage

sc=Scene();q0=np.array([j['q'] for j in json.loads((OUT/'empty_calibration_01/after.json').read_text())['joints']]);q=q0.copy()
cam=json.loads((OUT/'head_registration_candidate.json').read_text())['T_torso_camera'];rc=np.array(cam['R']);tc=np.array(cam['t']);folder=OUT/'empty_calibration_01/images';files=list(folder.glob('head_*.jpg'));last=max(files,key=lambda f:int(f.stem.split('_')[-1]));idx=last.stem.split('_')[-1]
rgb=np.array(Image.open(last));z=np.array(Image.open(folder/('depth_'+idx+'.png')))*.001;y,x=np.indices(z.shape);mask=(rgb.mean(2)<65)&(x>440)&(x<555)&(y>175)&(y<270);labels,n=ndimage.label(mask);cnt=np.bincount(labels.ravel());cnt[0]=0;mask=labels==cnt.argmax();uv=np.array([x[mask].mean(),y[mask].mean()]);meta=json.loads((folder/'camera_metadata.json').read_text());zz=float(np.median(z[mask&(z>.3)&(z<.7)]));pc=np.r_[(uv-[meta['cx'],meta['cy']])*zz/[meta['fx'],meta['fy']],zz];cap=rc@pc+tc
targets={'left':(cap+np.array([0,0,.1795]),Rotation.from_euler('y',np.pi/2).as_matrix()),'right':(np.array([cap[0]-.1615,cap[1],-.025]),np.eye(3))}
rng=np.random.default_rng(21);solutions={}
for side,off in [('left',15),('right',22)]:
    target,rd=targets[side];lo=np.array([-2.28,-.77,-2.37,-.87,-1.86,-1.47,-1.47] if side=='left' else [-2.28,-1.88,-2.37,-.87,-1.86,-1.47,-1.47]);hi=np.array([.67,1.88,2.37,1.77,1.86,1.47,1.47] if side=='left' else [.67,.77,2.37,1.77,1.86,1.47,1.47]);seed=q0[off:off+7].copy();candidates=[]
    def fun(a):
        q[off:off+7]=a;sc.setq(q);p,r=sc.pose(side);return np.r_[8*(p-target),Rotation.from_matrix(rd.T@r).as_rotvec(),.0003*(a-seed)]
    for k in range(14):
        start=seed if k==0 else np.clip(seed+rng.normal(0,.8,7),lo+.001,hi-.001)
        result=least_squares(fun,start,bounds=(lo,hi),max_nfev=180);q[off:off+7]=result.x;sc.setq(q);p,r=sc.pose(side);pe=np.linalg.norm(p-target);re=Rotation.from_matrix(rd.T@r).magnitude()
        candidates.append(dict(q=result.x.tolist(),position_error_m=float(pe),orientation_error_rad=float(re),valid=bool(pe<.004 and re<.035)))
    candidates.sort(key=lambda a:0 if a['valid'] else 1);solutions[side]=candidates
    best=min(candidates,key=lambda a:(not a['valid'],a['position_error_m']+a['orientation_error_rad']*.1,np.linalg.norm(np.array(a['q'])-seed)))
    q[off:off+7]=best['q'];print(side,json.dumps(best),flush=True)
sc.setq(q,.0)
def hops(a,b):
    pa=[];pb=[]
    while a:pa.append(a);a=int(sc.m.body_parentid[a])
    while b:pb.append(b);b=int(sc.m.body_parentid[b])
    return next((k+pb.index(v) for k,v in enumerate(pa) if v in pb),99)
distances=[]
for a in range(sc.m.ngeom):
    for b in range(a):
        if hops(int(sc.m.geom_bodyid[a]),int(sc.m.geom_bodyid[b]))<=2:continue
        distances.append((mujoco.mj_geomDistance(sc.m,sc.d,a,b,.3,None),sc.gnames[a],sc.gnames[b]))
distances.sort();report=dict(candidate_only=True,cap_camera=pc.tolist(),cap_torso=cap.tolist(),pixel=uv.tolist(),depth_m=zz,targets={s:dict(position=p.tolist(),R=r.tolist()) for s,(p,r) in targets.items()},solutions=solutions,candidate_body_q=q.tolist(),closest_self_pairs=distances[:20],contact_plan_ready=False)
(OUT/'bimanual_grasp_ik_candidate.json').write_text(json.dumps(report,indent=2));print('CAP_TORSO',cap);print('SELF_CLOSEST',distances[:8])
