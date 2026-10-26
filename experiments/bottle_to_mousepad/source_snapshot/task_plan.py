"""Offline model checks for this scene. Never sends robot commands."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
scene = json.loads((ROOT/'scene_initial.json').read_text())
state = json.loads((ROOT/'initial_state.json').read_text())
urdf = ET.parse(ROOT/'g1_29dof_rev_1_0.urdf').getroot()
names = ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']
origins, axes, lower, upper = [], [], [], []
for name in names:
    j=urdf.find("joint[@name='right_"+name+"_joint']")
    o=j.find('origin');t=np.eye(4)
    t[:3,3]=np.fromstring(o.get('xyz'),sep=' ')
    t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_dcm()
    origins.append(t);axes.append(np.fromstring(j.find('axis').get('xyz'),sep=' '))
    lower.append(float(j.find('limit').get('lower')))
    upper.append(float(j.find('limit').get('upper')))

def fk(q):
    t=np.eye(4);points=[]
    for o,a,v in zip(origins,axes,q):
        t=t.dot(o);points.append(t[:3,3].copy())
        r=np.eye(4);r[:3,:3]=Rotation.from_rotvec(a*v).as_dcm();t=t.dot(r)
    return t,np.array(points)

normal=np.array(scene['table_plane_body_normal'])
plane=np.array(scene['table_plane_body_point'])
edge=np.array(scene['table_front_x_as_function_y'])

def table_z(x,y):
    return float(plane[2]-(normal[0]*(x-plane[0])+normal[1]*(y-plane[1]))/normal[2])

def collision(q, bottle=False, allow_target=False):
    t,points=fk(q);samples=[]
    # Arm link centerline samples with conservative local radii.
    for a,b,rad,self_check in [(points[1],points[3],.045,False),(points[3],points[5],.04,True),(points[5],points[6],.04,True)]:
        for w in np.linspace(0,1,9):samples.append((a*(1-w)+b*w,rad,self_check))
    # Approximate open gripper envelope, including the camera above the wrist.
    for x in np.linspace(.025,.225,9):
        half_width=.03 if x<.10 else (.045 if bottle else .065)
        for y in [-half_width,0,half_width]:
            for z in [-.04,.035,.09 if x<.14 else .035]:
                samples.append((t[:3,3]+t[:3,:3].dot([x,y,z]),.008,True))
    clearance=10.
    for p,rad,self_check in samples:
        if p[0]+rad>np.polyval(edge,p[1])-.012:
            h=float(normal.dot(p-plane))-rad
            clearance=min(clearance,h)
            if h<.018:return 'TABLE',clearance
        # Fixed white bottle approximation; also preserve a geometric margin.
        if table_z(.39,-.04)-.01<p[2]<.13:
            if np.linalg.norm(p[:2]-[.39,-.04])<.035+rad+.012:return 'WHITE_BOTTLE',clearance
        # Tool/forearm must stay in front of the torso when crossing its center.
        if self_check and abs(p[1])<.11 and -.12<p[2]<.24 and p[0]<.09+rad:
            return 'TORSO',clearance
    if bottle:
        g=t[:3,3]+t[:3,:3].dot([.19,0,0])
        if np.linalg.norm(g[:2]-[.39,-.04])<.034+.035+.022:return 'CARRIED_BOTTLE_NEAR_WHITE',clearance
    return '',clearance

def ik(tcp,yaw,seed):
    rot=Rotation.from_euler('z',yaw).as_dcm()
    wrist=np.array(tcp)-rot.dot([.19,0,0])
    def residual(q):
        t,pts=fk(q);body=[]
        for a,b in [(pts[3],pts[5]),(pts[5],pts[6])]:
            for w in np.linspace(0,1,9):
                p=a*(1-w)+b*w
                outside=max(p[0]-.13,abs(p[1])-.13,p[2]-.24,-.12-p[2])
                body.append(max(0.,.008-outside)*100)
        return np.r_[(t[:3,3]-wrist)*1000,
                     Rotation.from_dcm(rot.T.dot(t[:3,:3])).as_rotvec()*100,
                     body,(q-seed)*.015]
    sol=least_squares(residual,seed,bounds=(lower,upper),max_nfev=1000,ftol=1e-11,xtol=1e-11,gtol=1e-11)
    t,_=fk(sol.x)
    error=float(np.linalg.norm(t[:3,3]+t[:3,:3].dot([.19,0,0])-tcp))
    if error>.001:raise RuntimeError('IK_ERROR '+str(error))
    return sol.x,error

q0=np.array([j['q'] for j in state['joints'][22:29]])
waypoints=[('initial',q0,False)]
q=q0.copy();q[1]=-.25;waypoints.append(('outward',q.copy(),False))
q[0]=.5;waypoints.append(('retract_below_edge',q.copy(),False))
q[3]=0.;waypoints.append(('raise_forearm_behind_edge',q.copy(),False))
q[5]=-.4;waypoints.append(('level_tool_behind_edge',q.copy(),False))

goals=[
 ('pregrasp',[.345,-.166,table_z(.40,-.166)+.13],0.,False),
 ('grasp',[.403,-.166,table_z(.40,-.166)+.105],0.,False),
 ('lift',[.403,-.166,table_z(.40,-.166)+.16],0.,True),
 ('front_corridor',[.28,-.166,table_z(.40,-.166)+.16],0.,True),
 ('cross_mid_right',[.26,-.07,table_z(.40,-.166)+.16],.55,True),
 ('cross_mid_left',[.26,.03,table_z(.40,-.166)+.16],.9,True),
 ('cross_front',[.28,.09,table_z(.40,-.166)+.16],1.05,True),
 ('above_mat',[.38,.09,table_z(.38,.09)+.16],.45,True),
 ('place',[.38,.09,table_z(.38,.09)+.105],.45,True),
 ('empty_lift',[.38,.09,table_z(.38,.09)+.16],.45,False),
 ('empty_front',[.24,.09,table_z(.40,-.166)+.16],1.25,False),
 ('empty_cross_mid_left',[.24,.03,table_z(.40,-.166)+.16],.95,False),
 ('empty_cross_mid_right',[.24,-.08,table_z(.40,-.166)+.16],.55,False),
 ('empty_cross',[.26,-.23,table_z(.40,-.166)+.16],0.,False)]
for label,tcp,yaw,carried in goals:
    print('SOLVING',label,flush=True)
    start_q=q.copy();q,error=ik(np.array(tcp),yaw,q)
    def segment_ok(a,b):
        return all(not collision(a*(1-w)+b*w,bottle=carried)[0] for w in np.linspace(0,1,81))
    if segment_ok(start_q,q):
        waypoints.append((label,q.copy(),carried))
    else:
        start_t,_=fk(start_q);start_tcp=start_t[:3,3]+start_t[:3,:3].dot([.19,0,0])
        end_r=Rotation.from_euler('z',yaw).as_dcm()
        rotvec=Rotation.from_dcm(start_t[:3,:3].T.dot(end_r)).as_rotvec()
        count=max(3,int(np.ceil(np.linalg.norm(np.array(tcp)-start_tcp)/.015)),int(np.ceil(np.linalg.norm(rotvec)/.10)))
        seq=[start_q];seed=start_q
        for k in range(1,count+1):
            w=k/count
            # All carrying/placement rotations in this plan are pure yaw.
            r=start_t[:3,:3].dot(Rotation.from_rotvec(rotvec*w).as_dcm())
            step_yaw=float(np.arctan2(r[1,0],r[0,0]))
            seed,_=ik(start_tcp*(1-w)+np.array(tcp)*w,step_yaw,seed)
            seq.append(seed.copy())
        current=0;part=0
        while current<len(seq)-1:
            valid=[j for j in range(current+1,len(seq)) if segment_ok(seq[current],seq[j])]
            if not valid:raise RuntimeError('NO_COLLISION_FREE_SEGMENT '+label+' '+str(current))
            current=max(valid);part+=1
            waypoints.append((label if current==len(seq)-1 else label+'__part'+str(part),seq[current].copy(),carried))
        q=seq[-1].copy()

for label,target,carried in reversed(waypoints[:5]):
    waypoints.append(('return_'+label,target.copy(),False))

report=[]
for i,(label,q,bottle) in enumerate(waypoints):
    t,_=fk(q);bad=[];minimum=10.
    if i:
        prev=waypoints[i-1][1]
        for w in np.linspace(0,1,101):
            reason,c=collision(prev*(1-w)+q*w,bottle=bottle)
            minimum=min(minimum,c)
            if reason:bad.append(reason)
    report.append({'label':label,'q':q.tolist(),'tcp':(t[:3,3]+t[:3,:3].dot([.19,0,0])).tolist(),
                   'collision_flags':sorted(set(bad)),'minimum_table_clearance_m':minimum})
(ROOT/'offline_route.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))


