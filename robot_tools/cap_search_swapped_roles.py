"""Feasibility of left bottle support and vertical right cap approach."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();p=json.loads((OUT/'bimanual_grasp_ik_candidate.json').read_text());cap=np.array(p['cap_torso']);q0=np.array([j['q'] for j in json.loads((OUT/'left_force080_after.json').read_text())['joints']]);q=q0.copy();rng=np.random.default_rng(89);rows=[]
tasks=[]
for yaw in [-45,45]:
    for height in [-.025,.04,.06]:
        for flip in [0,180]:
            r=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',flip,degrees=True).as_matrix();tasks.append(('left',f'body_yaw{yaw}_z{height}_roll{flip}',np.array([cap[0],cap[1],height])-.1615*r[:,0],r))
for yaw in [-90,-45,0,45,90]:
    r=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('y',90,degrees=True).as_matrix();tasks.append(('right',f'cap_vertical_yaw{yaw}',cap+np.array([0,0,.1795]),r))
for side,name,target,rd in tasks:
    off=15 if side=='left' else 22;lo=np.array([-2.28,-.77,-2.37,-.87,-1.86,-1.47,-1.47] if side=='left' else [-2.28,-1.88,-2.37,-.87,-1.86,-1.47,-1.47]);hi=np.array([.67,1.88,2.37,1.77,1.86,1.47,1.47] if side=='left' else [.67,.77,2.37,1.77,1.86,1.47,1.47]);seed=q0[off:off+7].copy();best=None
    def fun(x):
        q[off:off+7]=x;sc.setq(q);actual,r=sc.pose(side);return np.r_[8*(actual-target),Rotation.from_matrix(rd.T@r).as_rotvec(),.0005*(x-seed)]
    for k in range(8):
        init=seed if k==0 else np.clip(seed+rng.normal(0,1,7),lo+.001,hi-.001)
        result=least_squares(fun,init,bounds=(lo,hi),max_nfev=200);e=np.linalg.norm(fun(result.x)[:6])
        if best is None or e<best[0]:best=(e,result.x.copy())
    q[off:off+7]=best[1];sc.setq(q);actual,r=sc.pose(side);pe=np.linalg.norm(actual-target);re=Rotation.from_matrix(rd.T@r).magnitude();row=dict(side=side,name=name,target=target.tolist(),R=rd.tolist(),q=best[1].tolist(),position_error_m=float(pe),rotation_error_rad=float(re),valid=bool(pe<.004 and re<.035));rows.append(row)
    print(name,'VALID',row['valid'],'err',round(pe,5),round(re,5),flush=True)
(OUT/'swapped_roles_ik.json').write_text(json.dumps(dict(candidate_only=True,cap_torso=cap.tolist(),rows=rows),indent=2))
