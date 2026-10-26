"""Right cap grip, 5..15 degrees from vertical; no actuation."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();cap=np.array(json.loads((OUT/'bimanual_grasp_ik_candidate.json').read_text())['cap_torso']);prior=json.loads((OUT/'swapped_roles_ik.json').read_text());q0=np.array([j['q'] for j in json.loads((OUT/'waist_nograv_005/after.json').read_text())['joints']]);q=q0.copy();q[15:22]=next(p['q'] for p in prior['rows'] if p['name']=='body_yaw-45_z0.04_roll180');start=np.array(next(p['q'] for p in prior['rows'] if p['name']=='cap_vertical_yaw90'));lo=np.array([-2.28,-1.88,-2.37,-.87,-1.86,-1.47,-1.47]);hi=np.array([.67,.77,2.37,1.77,1.86,1.47,1.47]);rows=[];rng=np.random.default_rng(18)
for pitch in [75,80,85]:
    seed=start.copy()
    for angle in range(-60,136,15):
        rd=Rotation.from_euler('z',angle,degrees=True).as_matrix()@Rotation.from_euler('y',pitch,degrees=True).as_matrix();target=cap-.1795*rd[:,0]
        def fun(x):
            q[22:]=x;sc.setq(q);p,r=sc.pose('right');return np.r_[8*(p-target),Rotation.from_matrix(rd.T@r).as_rotvec(),.001*(x-seed)]
        best=None
        for initial in [seed,start,np.clip(start+rng.normal(0,.6,7),lo+.001,hi-.001)]:
            result=least_squares(fun,np.clip(initial,lo+.001,hi-.001),bounds=(lo,hi),max_nfev=180);error=np.linalg.norm(fun(result.x)[:6]);
            if best is None or error<best[0]:best=(error,result.x.copy())
        q[22:]=best[1];sc.setq(q);p,r=sc.pose('right');pe=np.linalg.norm(p-target);re=Rotation.from_matrix(rd.T@r).magnitude();valid=bool(pe<.003 and re<.025)
        for side,value in [('left',.0021),('right',-.0059)]:
            for j in [1,2]:sc.d.qpos[sc.m.joint(side+'_dex1_finger_joint_'+str(j)).qposadr[0]]=value
        mujoco.mj_forward(sc.m,sc.d);distance=sc.nearest(sc.tool['left'],sc.tool['right']);rows.append(dict(pitch_deg=pitch,yaw_deg=angle,q=best[1].tolist(),target=target.tolist(),R=rd.tolist(),valid=valid,position_error_m=float(pe),rotation_error_rad=float(re),intertool_distance=distance));seed=best[1].copy()
    print('NEAR_VERTICAL',pitch,'valid',[r['yaw_deg'] for r in rows if r['pitch_deg']==pitch and r['valid']],flush=True)
(OUT/'near_vertical_cap_search.json').write_text(json.dumps(dict(candidate_only=True,left_body_q=q[15:22].tolist(),rows=rows,cap_torso=cap.tolist()),indent=2))
