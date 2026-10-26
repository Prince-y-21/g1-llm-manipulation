"""Empty right wrist camera viewing the outward left gripper; candidate only."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sc=Scene();q0=np.array([j['q'] for j in json.loads((OUT/'left_force_step_after.json').read_text())['joints']]);out=q0.copy();out[16]+=.4;out[23]-=.4;sc.setq(out);p,r=sc.pose('left');aim=p+r@np.array([.15,0,0]);middle=out.copy();middle[22:]=[.2168,-1.3668,.4323,.0582,.7859,-.6349,-.0194];q=middle.copy()
basis=np.c_[np.array([0,-1.,0]),np.array([-.66,0,-.75])/np.linalg.norm([.66,0,.75]),np.array([.75,0,-.66])/np.linalg.norm([.66,0,.75])]
lo=np.array([-2.28,-1.88,-2.37,-.87,-1.70,-1.35,-.95]);hi=np.array([.67,.77,2.37,1.77,1.70,1.35,.95]);rows=[]
for position in [[.22,-.24,.18],[.20,-.28,.16],[.25,-.22,.20],[.18,-.24,.16]]:
    p=np.array(position);eye=p.copy()
    for _ in range(8):
        f=aim-eye;f/=np.linalg.norm(f);right=np.cross(f,[0,0,1]);right/=np.linalg.norm(right);down=np.cross(f,right);rd=np.c_[right,down,f]@basis.T;eye=p+rd@np.array([.0915,0,.075])
    seed=middle[22:].copy()
    def fun(x):
        q[22:]=x;sc.setq(q);actual,r=sc.pose('right');eye=actual+r@np.array([.0915,0,.075]);direction=aim-eye;direction/=np.linalg.norm(direction);forward=r@basis[:,2]
        return np.r_[8*(actual-p),np.cross(forward,direction),max(0,-forward@direction),.01*(x-seed)]
    result=least_squares(fun,seed,bounds=(lo,hi),max_nfev=300);q[22:]=result.x;sc.setq(q);actual,r=sc.pose('right');pe=np.linalg.norm(actual-p);eye=actual+r@np.array([.0915,0,.075]);direction=aim-eye;direction/=np.linalg.norm(direction);re=np.arccos(np.clip((r@basis[:,2])@direction,-1,1));rows.append(dict(position=position,q=result.x.tolist(),position_error_m=float(pe),rotation_error_rad=float(re),valid=bool(pe<.004 and re<.1)))
(OUT/'inspection_ik_candidate.json').write_text(json.dumps(dict(aim_torso=aim.tolist(),rows=rows,initial=q0.tolist(),outward=out.tolist(),middle=middle.tolist()),indent=2))
print(json.dumps(rows,indent=2))
