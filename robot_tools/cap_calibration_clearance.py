"""Candidate empty calibration path with all-body CAD and enlarged table box."""
from cap_cad_geometry import *
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import tempfile,sys
statepath=Path(sys.argv[1]) if len(sys.argv)>1 else OUT/'pre_outward_state.json'
sc=Scene();q0=np.array([j['q'] for j in json.loads(statepath.read_text())['joints']]);sc.setq(q0)
temp=Path(tempfile.gettempdir())/'g1_cap_calibration_candidate.xml';mujoco.mj_saveLastXML(str(temp),sc.m);xml=ET.fromstring(temp.read_text());world=xml.find('worldbody')
t=sc.m.body('torso_link').id;rt=sc.d.xmat[t].reshape(3,3);pt=sc.d.xpos[t];quat=Rotation.from_matrix(rt).as_quat()[[3,0,1,2]]
def envbox(name,center,size):
    p=pt+rt@np.array(center);ET.SubElement(world,'geom',name=name,type='box',pos=' '.join(map(str,p)),quat=' '.join(map(str,quat)),size=' '.join(map(str,size)),rgba='.5 .2 .2 1')
# Approx table from current RGB-D: expanded closer and higher than its fitted edge.
envbox('table_uncertainty_box',[.915,0,-.12],[.585,1,.06])
envbox('bottle_uncertainty_box',[.373,-.103,.005],[.05,.05,.145])
assets={str(f.relative_to(ASSET)).replace('\\','/'):f.read_bytes() for f in ASSET.rglob('*.STL')}
sc.m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'),assets);sc.d=mujoco.MjData(sc.m);sc.qids=[sc.m.joint(n).qposadr[0] for n in joint_names()];sc.gnames=[sc.m.geom(i).name for i in range(sc.m.ngeom)]

def hops(a,b):
    pa=[];pb=[]
    while a:pa.append(a);a=int(sc.m.body_parentid[a])
    while b:pb.append(b);b=int(sc.m.body_parentid[b])
    return next((n+pb.index(x) for n,x in enumerate(pa) if x in pb),99)
armids=[i for i,n in enumerate(sc.gnames) if any(n.startswith(s+'_'+j) for s in ['left','right'] for j in ['shoulder','elbow','wrist','dex1','camera'])]
robot=[i for i,n in enumerate(sc.gnames) if not n.endswith('uncertainty_box')]
pairs=sorted(set((min(a,b),max(a,b)) for a in armids for b in robot if a!=b and hops(int(sc.m.geom_bodyid[a]),int(sc.m.geom_bodyid[b]))>2))
q=q0.copy();q[16]+=.4;q[23]-=.4;way=[q0.copy(),q.copy()];ik=[];seed=q[15:22].copy()
for target in [np.array([.15,.32,.12]),np.array([.14,.18,.16])]:
    def fun(x):
        q[15:22]=x;sc.setq(q);p,r=sc.pose('left');return np.r_[8*(p-target),.04*(x-seed)]
    lo=[-2.28,-.77,-2.37,-.87,-1.86,-1.47,-1.47];hi=[.67,1.88,2.37,1.77,1.86,1.47,1.47]
    opt=least_squares(fun,seed,bounds=(lo,hi));q[15:22]=opt.x;seed=opt.x.copy();sc.setq(q);p,r=sc.pose('left');ik.append(dict(target=target.tolist(),pos=p.tolist(),q=seed.tolist()));way.append(q.copy())
q2=way[-1].copy();qy=q2.copy();qy[21]+=.15;qr=q2.copy();qr[19]+=.15;way.extend([qy,qr])
rows=[]
for leg,(a,b) in enumerate(zip(way[:-1],way[1:])):
    for phase in np.linspace(0,1,81):
        q=a+(b-a)*phase
        for opening in [-.02,0,.0245]:
            sc.setq(q,opening)
            ds=sorted((mujoco.mj_geomDistance(sc.m,sc.d,x,y,.5,None),sc.gnames[x],sc.gnames[y]) for x,y in pairs)
            env={n:sc.nearest(armids,[sc.m.geom(n).id]) for n in ['table_uncertainty_box','bottle_uncertainty_box']}
            rows.append(dict(leg=leg,phase=float(phase),opening=opening,self_nearest=ds[0],env=env))
report=dict(candidate_only=True,unverified_mount=True,anchor_source=str(statepath),whole_visual_meshes=True,neighbor_hop_exclusion=2,ik=ik,waypoints=[x.tolist() for x in way],checks=rows)
(OUT/'calibration_fullbody_candidate.json').write_text(json.dumps(report,indent=2))
print('IK',json.dumps(ik))
for leg in range(len(way)-1):
    rr=[r for r in rows if r['leg']==leg]
    print('LEG',leg,'SELF',min((r['self_nearest'][0],r['phase'],r['self_nearest'][1:]) for r in rr))
    for n in ['table_uncertainty_box','bottle_uncertainty_box']:print(n,min((r['env'][n][0],r['phase'],r['env'][n][1:]) for r in rr))
