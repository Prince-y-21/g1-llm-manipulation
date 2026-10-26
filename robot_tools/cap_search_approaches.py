from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();prior=json.loads((OUT/'bimanual_grasp_ik_candidate.json').read_text());cap=np.array(prior['cap_torso']);q0=np.array([j['q'] for j in json.loads((OUT/'empty_calibration_01/after.json').read_text())['joints']]);q=q0.copy();q[22:]=prior['solutions']['right'][0]['q'];lo=np.array([-2.28,-.77,-2.37,-.87,-1.86,-1.47,-1.47]);hi=np.array([.67,1.88,2.37,1.77,1.86,1.47,1.47]);rows=[]
for pitch_deg in [30,45,60,75]:
    for yaw_deg in [-90,-60,-30,0,30]:
        rd=Rotation.from_euler('zy',[yaw_deg,pitch_deg],degrees=True).as_matrix()
        # Explicit Rz(yaw) Ry(pitch), so the approach comes from robot front/left.
        rd=Rotation.from_euler('z',yaw_deg,degrees=True).as_matrix()@Rotation.from_euler('y',pitch_deg,degrees=True).as_matrix();target=cap-.1795*rd[:,0]
        seed=np.array([-.9,.3,-.8,1.2,-.5,.6,0]);best=None
        def fun(x):
            q[15:22]=x;sc.setq(q);p,r=sc.pose('left');return np.r_[8*(p-target),Rotation.from_matrix(rd.T@r).as_rotvec(),.0003*(x-seed)]
        for initial in [seed,np.array([-1.5,.5,1.0,.6,.5,.5,.6])]:
            result=least_squares(fun,initial,bounds=(lo,hi),max_nfev=180);error=np.linalg.norm(fun(result.x)[:6]);
            if best is None or error<best[0]:best=(error,result.x.copy())
        q[15:22]=best[1];sc.setq(q,-.012)
        left=sc.tool['left'];right=sc.tool['right'];inter=sc.nearest(left,right)
        pose,rot=sc.pose('left');pe=np.linalg.norm(pose-target);re=Rotation.from_matrix(rd.T@rot).magnitude()
        rows.append(dict(yaw_deg=yaw_deg,pitch_deg=pitch_deg,q=best[1].tolist(),position_error_m=float(pe),orientation_error_rad=float(re),valid=bool(pe<.004 and re<.035),tool_distance=inter))
(OUT/'approach_search.json').write_text(json.dumps(dict(candidate_only=True,cap_torso=cap.tolist(),left=rows),indent=2));print('VALID_APPROACHES')
for row in rows:
    if row['valid']:print(json.dumps(row))
