"""Offline waist .05rad test planning with both empty hands outward."""
from cap_cad_geometry import *
from scipy.spatial.transform import Rotation
import tempfile,sys
state=Path(sys.argv[1]) if len(sys.argv)>1 else OUT/'left_fullstroke90_after.json';prefix=sys.argv[2] if len(sys.argv)>2 else 'waist_micro';no_waist_gravity='--no-waist-gravity' in sys.argv
sc=Scene();q0=np.array([j['q'] for j in json.loads(state.read_text())['joints']]);sc.setq(q0);ids=[sc.m.joint(n).dofadr[0] for n in joint_names()];g0=sc.d.qfrc_bias[ids].copy();a=q0.copy();a[16]+=.4;a[23]-=.4;b=a.copy();b[14]+=.05;way=[q0,a,b,b,a,a,q0,q0];dur=[10.5,4,2,4,1,10.5,2];samples=[];t0=0
for start,end,dt in zip(way[:-1],way[1:],dur):
    for k in range(int(round(dt/.05))):
        x=k*.05/dt;q=start+(end-start)*x*x*(3-2*x);sc.setq(q);g=sc.d.qfrc_bias[ids]-g0;g[:15 if no_waist_gravity else 14]=0;samples.append((t0+k*.05,q.copy(),g.copy()))
    t0+=dt
samples.append((t0,q0,np.zeros(29)))
# Expanded environment and full visual meshes, including moving rails and cameras.
tmp=Path(tempfile.gettempdir())/'g1_waist_micro_scene.xml';mujoco.mj_saveLastXML(str(tmp),sc.m);u=ET.fromstring(tmp.read_text());sc.setq(q0);ti=sc.m.body('torso_link').id;rt=sc.d.xmat[ti].reshape(3,3);pt=sc.d.xpos[ti];quat=Rotation.from_matrix(rt).as_quat()[[3,0,1,2]]
for name,center,size in [('table',[.915,0,-.12],[.585,1,.06]),('bottle',[.3736,-.0816,.005],[.05,.05,.145])]:
    xyz=pt+rt@np.array(center);ET.SubElement(u.find('worldbody'),'geom',name=name+'_obstacle',type='box',pos=' '.join(map(str,xyz)),quat=' '.join(map(str,quat)),size=' '.join(map(str,size)))
assets={str(f.relative_to(ASSET)).replace('\\','/'):f.read_bytes() for f in ASSET.rglob('*.STL')};sc.m=mujoco.MjModel.from_xml_string(ET.tostring(u,encoding='unicode'),assets);sc.d=mujoco.MjData(sc.m);sc.qids=[sc.m.joint(n).qposadr[0] for n in joint_names()];sc.gnames=[sc.m.geom(i).name for i in range(sc.m.ngeom)];env=[sc.m.geom(n+'_obstacle').id for n in ['table','bottle']];robot=[i for i in range(sc.m.ngeom) if i not in env]
def hops(a,b):
    pa=[];pb=[]
    while a:pa.append(a);a=int(sc.m.body_parentid[a])
    while b:pb.append(b);b=int(sc.m.body_parentid[b])
    return next((k+pb.index(v) for k,v in enumerate(pa) if v in pb),99)
pairs=[(a,b) for a in robot for b in robot if a>b and hops(int(sc.m.geom_bodyid[a]),int(sc.m.geom_bodyid[b]))>2];checks=[]
for k,(t,q,g) in enumerate(samples):
    if k%4 and k!=len(samples)-1:continue
    for opening in [-.02,0,.0245]:
        sc.setq(q,opening);dist=[(mujoco.mj_geomDistance(sc.m,sc.d,a,b,.4,None),sc.gnames[a],sc.gnames[b]) for a,b in pairs];custom=[d for d in dist if any(s in d[1]+d[2] for s in ['dex1','camera'])];body=[d for d in dist if d not in custom];checks.append(dict(t=t,custom=min(custom),body=min(body),environment=sc.nearest(robot,env)))
# Initial/final under-table outward egress was separately physically verified.
middle=[x for x in checks if 10.5<=x['t']<=21.5]
print('WAIST_CLEARANCE',min((x['body'][0],x['t'],x['body'][1:]) for x in middle),min((x['custom'][0],x['t'],x['custom'][1:]) for x in middle),flush=True)
assert min(x['custom'][0] for x in middle)>.03
assert min(x['body'][0] for x in middle)>.015
assert min(x['environment'][0] for x in checks)>.03
with (OUT/(prefix+'_profile.txt')).open('w') as f:
    f.write('EMPTY_WAIST_ARMS_VALIDATION_V2 '+str(len(samples))+'\n'+' '.join(map(str,q0))+'\n')
    for t,q,g in samples:f.write(' '.join(map(str,np.r_[t,q,g]))+'\n')
(OUT/(prefix+'_plan.json')).write_text(json.dumps(dict(scope='empty .05rad waist-pitch microtest; no mode switch',waist_gravity_delta_enabled=not no_waist_gravity,factory_body_pair_margin_m=.015,custom_tool_pair_margin_m=.03,environment_margin_m=.03,margin_basis='Custom mounts retain 30mm uncertainty margin; same-model factory body joints use 15mm separation. Initial near wrist/hip is only the previously tested outward egress.',root_rotation_abort_rad=.035,waypoints=[q.tolist() for q in way],duration=t0,checks=checks),indent=2));print('WAIST_MICRO_PLANNED',t0,'min_body',min(x['body'][0] for x in checks),'min_env',min(x['environment'][0] for x in checks))
