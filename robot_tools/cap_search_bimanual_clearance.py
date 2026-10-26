from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();prior=json.loads((OUT/'bimanual_grasp_ik_candidate.json').read_text());cap=np.array(prior['cap_torso']);q0=np.array([j['q'] for j in json.loads((OUT/'empty_calibration_01/after.json').read_text())['joints']]);approaches=json.loads((OUT/'approach_search.json').read_text())['left'];q=q0.copy();q[15:22]=next(x['q'] for x in approaches if x['valid'] and x['yaw_deg']==-90 and x['pitch_deg']==30)
def hops(a,b):
    pa=[];pb=[]
    while a:pa.append(a);a=int(sc.m.body_parentid[a])
    while b:pb.append(b);b=int(sc.m.body_parentid[b])
    return next((k+pb.index(v) for k,v in enumerate(pa) if v in pb),99)
pairs=[(a,b) for a in range(sc.m.ngeom) for b in range(a) if hops(int(sc.m.geom_bodyid[a]),int(sc.m.geom_bodyid[b]))>2 and any(j in sc.gnames[a]+sc.gnames[b] for j in ['shoulder','elbow','wrist','dex1','camera'])]
def evaluate():
    sc.setq(q)
    for side,value in [('left',0.),('right',.012)]:
        for k in [1,2]:sc.d.qpos[sc.m.joint(side+'_dex1_finger_joint_'+str(k)).qposadr[0]]=value
    mujoco.mj_forward(sc.m,sc.d)
    return sorted((mujoco.mj_geomDistance(sc.m,sc.d,a,b,.3,None),sc.gnames[a],sc.gnames[b]) for a,b in pairs)
rows=[];lo=[-2.28,-1.88,-2.37,-.87,-1.86,-1.47,-1.47];hi=[.67,.77,2.37,1.77,1.86,1.47,1.47]
for yaw in [0,10,20,30,40]:
    r=Rotation.from_euler('z',yaw,degrees=True).as_matrix();target=np.array([cap[0],cap[1],-.025])-.1615*r[:,0]
    for seed in [np.array([-.5,-.3,.3,1.2,0,-.7,0]),np.array([-.6,-.6,-.4,1.5,.1,-.8,.5]),q0[22:].copy()]:
        def fun(x):
            q[22:]=x;sc.setq(q);p,rd=sc.pose('right');return np.r_[8*(p-target),Rotation.from_matrix(r.T@rd).as_rotvec(),.015*(x-seed)]
        result=least_squares(fun,seed,bounds=(lo,hi),max_nfev=220);q[22:]=result.x;sc.setq(q);p,rd=sc.pose('right');pe=np.linalg.norm(p-target);re=Rotation.from_matrix(r.T@rd).magnitude();ds=evaluate()
        rows.append(dict(right_yaw_deg=yaw,position_error_m=float(pe),orientation_error_rad=float(re),right_q=result.x.tolist(),left_q=q[15:22].tolist(),nearest=ds[:8],valid=bool(pe<.004 and re<.035)))
rows.sort(key=lambda x:(not x['valid'],-x['nearest'][0][0]));(OUT/'bimanual_clearance_search.json').write_text(json.dumps(dict(candidate_only=True,rows=rows),indent=2));print('BEST_CANDIDATES')
for row in rows[:5]:print(json.dumps(row))
