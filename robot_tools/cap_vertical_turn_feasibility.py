from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();prior=json.loads((OUT/'swapped_roles_ik.json').read_text());cap=np.array(prior['cap_torso']);q0=np.array([j['q'] for j in json.loads((OUT/'left_force080_after.json').read_text())['joints']]);q=q0.copy();left=next(x for x in prior['rows'] if x['name']=='body_yaw-45_z0.04_roll180');right=next(x for x in prior['rows'] if x['name']=='cap_vertical_yaw90');q[15:22]=left['q'];q[22:]=right['q'];seed=q[22:].copy();lo=np.array([-2.28,-1.88,-2.37,-.87,-1.86,-1.47,-1.47]);hi=np.array([.67,.77,2.37,1.77,1.86,1.47,1.47]);target=cap+[0,0,.1795];rows=[]
for angle in np.arange(90,271,10):
    rd=Rotation.from_euler('z',angle,degrees=True).as_matrix()@Rotation.from_euler('y',90,degrees=True).as_matrix()
    def fun(x):
        q[22:]=x;sc.setq(q);actual,r=sc.pose('right');return np.r_[8*(actual-target),Rotation.from_matrix(rd.T@r).as_rotvec(),.002*(x-seed)]
    result=least_squares(fun,seed,bounds=(lo,hi),max_nfev=220);q[22:]=result.x;sc.setq(q);p,r=sc.pose('right');pe=np.linalg.norm(p-target);re=Rotation.from_matrix(rd.T@r).magnitude();valid=bool(pe<.004 and re<.035)
    for side,value in [('left',.003),('right',-.006)]:
        for j in [1,2]:sc.d.qpos[sc.m.joint(side+'_dex1_finger_joint_'+str(j)).qposadr[0]]=value
    mujoco.mj_forward(sc.m,sc.d);distance=sc.nearest(sc.tool['left'],sc.tool['right'])
    rows.append(dict(angle_deg=int(angle),valid=valid,q=result.x.tolist(),position_error_m=float(pe),rotation_error_rad=float(re),intertool_distance=distance));seed=result.x.copy()
    print(angle,valid,round(pe,5),round(re,5),distance,flush=True)
(OUT/'vertical_turn_candidate.json').write_text(json.dumps(dict(candidate_only=True,left_body=left,rows=rows,cap_torso=cap.tolist()),indent=2))
