"""Offline workspace check. This script does not command the waist or arms."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();q0=np.array([j['q'] for j in json.loads((OUT/'left_fullstroke90_after.json').read_text())['joints']]);cap=np.array(json.loads((OUT/'bimanual_grasp_ik_candidate.json').read_text())['cap_torso']);prior=json.loads((OUT/'swapped_roles_ik.json').read_text());seed0=np.array(next(r for r in prior['rows'] if r['name']=='cap_vertical_yaw90')['q']);sc.setq(q0);ti=sc.m.body('torso_link').id;rt0=sc.d.xmat[ti].reshape(3,3).copy();pt0=sc.d.xpos[ti].copy();cw=rt0@cap+pt0;axis_world=rt0[:,2];lo=np.array([-2.28,-1.88,-2.37,-.87,-1.86,-1.47,-1.47]);hi=np.array([.67,.77,2.37,1.77,1.86,1.47,1.47]);allrows=[]
for lean in [.10,.15,.20]:
    q=q0.copy();q[14]+=lean;sc.setq(q);rt=sc.d.xmat[ti].reshape(3,3).copy();pt=sc.d.xpos[ti].copy();target=rt.T@(cw+.1795*axis_world-pt);seed=seed0.copy();rows=[]
    for angle in np.arange(-90,271,15):
        rd=rt.T@rt0@Rotation.from_euler('z',angle,degrees=True).as_matrix()@Rotation.from_euler('y',90,degrees=True).as_matrix()
        def fun(x):
            q[22:]=x;sc.setq(q);p,r=sc.pose('right');return np.r_[8*(p-target),Rotation.from_matrix(rd.T@r).as_rotvec(),.001*(x-seed)]
        best=None
        for init in [seed,seed0,np.array([-1.6,-.4,.5,1.0,-.6,.5,.3])]:
            result=least_squares(fun,np.clip(init,lo+.001,hi-.001),bounds=(lo,hi),max_nfev=180);e=np.linalg.norm(fun(result.x)[:6]);
            if best is None or e<best[0]:best=(e,result.x.copy())
        q[22:]=best[1];sc.setq(q);p,r=sc.pose('right');pe=np.linalg.norm(p-target);re=Rotation.from_matrix(rd.T@r).magnitude();valid=bool(pe<.004 and re<.035);rows.append(dict(angle=int(angle),q=best[1].tolist(),position_error_m=float(pe),rotation_error_rad=float(re),valid=valid));seed=best[1].copy()
    entry=dict(waist_pitch_delta=lean,right_cap_target=target.tolist(),rows=rows);allrows.append(entry);print('WAIST_CANDIDATE',lean,'VALID_ANGLES',[r['angle'] for r in rows if r['valid']],flush=True)
(OUT/'waist_workspace_candidate.json').write_text(json.dumps(dict(offline_only=True,rows=allrows),indent=2))
