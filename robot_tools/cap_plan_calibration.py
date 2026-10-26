"""Kinematic candidate only; no transport, no actuation."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sc=Scene();q0=np.array([j['q'] for j in json.loads((OUT/'post_microtest_state.json').read_text())['joints']]);qs=q0.copy();qs[16]+=.4;qs[23]-=.4
lo=np.array([-2.31,-.8,-2.4,-.9,-1.89,-1.5,-1.5]);hi=np.array([.7,1.91,2.4,1.8,1.89,1.5,1.5])
rdes=Rotation.from_euler('y',1.266).as_matrix()

def solve(pos,seed):
    q=qs.copy()
    def fun(x):
        q[15:22]=x;sc.setq(q);p,r=sc.pose('left')
        return np.r_[8*(p-pos),.4*Rotation.from_matrix(rdes.T@r).as_rotvec(),.01*(x-seed)]
    opt=least_squares(fun,seed,bounds=(lo,hi),max_nfev=220)
    q[15:22]=opt.x;sc.setq(q);p,r=sc.pose('left')
    return q,dict(pos=p.tolist(),target=list(pos),pos_error=float(np.linalg.norm(p-pos)),orientation_error=float(np.linalg.norm(Rotation.from_matrix(rdes.T@r).as_rotvec())),q=opt.x.tolist())

q1,a=solve(np.array([.12,.32,.18]),qs[15:22]);q2,b=solve(np.array([.25,.26,.18]),q1[15:22])
way=[q0,qs,q1,q2];rows=[]
for leg,(s,e) in enumerate(zip(way[:-1],way[1:])):
    for v in np.linspace(0,1,61):
        q=s+(e-s)*v
        for finger in [-.02,.0245]:
            sc.setq(q,finger);rows.append(dict(leg=leg,phase=float(v),finger=finger,tool=sc.tool_clearance()))
report=dict(candidate_only=True,ik=[a,b],waypoints=[q.tolist() for q in way],checks=rows)
(OUT/'calibration_plan_candidate.json').write_text(json.dumps(report,indent=2))
print('IK',json.dumps([a,b]))
for leg in range(3):
    for s in ['left','right']:
        for k in ['legs','torso','other_tool']:
            vals=[(r['tool'][s][k][0],r['phase'],r['tool'][s][k][1:]) for r in rows if r['leg']==leg];print(leg,s,k,min(vals))
