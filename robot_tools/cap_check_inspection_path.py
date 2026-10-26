from cap_cad_geometry import *
from scipy.spatial.transform import Rotation
import tempfile
sc=Scene();c=json.loads((OUT/'inspection_ik_candidate.json').read_text());q0=np.array(c['initial']);out=np.array(c['outward']);mid=np.array(c['middle']);sc.setq(q0)
temp=Path(tempfile.gettempdir())/'cap_inspection_scene.xml';mujoco.mj_saveLastXML(str(temp),sc.m);xml=ET.fromstring(temp.read_text());t=sc.m.body('torso_link').id;rt=sc.d.xmat[t].reshape(3,3);pt=sc.d.xpos[t];pos=pt+rt@np.array([.915,0,-.12]);quat=Rotation.from_matrix(rt).as_quat()[[3,0,1,2]]
ET.SubElement(xml.find('worldbody'),'geom',name='table_uncertainty_box',type='box',pos=' '.join(map(str,pos)),quat=' '.join(map(str,quat)),size='.585 1 .06')
pos=pt+rt@np.array([.3736,-.0816,.005]);ET.SubElement(xml.find('worldbody'),'geom',name='bottle_uncertainty_box',type='box',pos=' '.join(map(str,pos)),quat=' '.join(map(str,quat)),size='.05 .05 .145')
assets={str(f.relative_to(ASSET)).replace('\\','/'):f.read_bytes() for f in ASSET.rglob('*.STL')};sc.m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'),assets);sc.d=mujoco.MjData(sc.m);sc.qids=[sc.m.joint(n).qposadr[0] for n in joint_names()];sc.gnames=[sc.m.geom(i).name for i in range(sc.m.ngeom)];table=sc.m.geom('table_uncertainty_box').id
def hops(a,b):
    pa=[];pb=[]
    while a:pa.append(a);a=int(sc.m.body_parentid[a])
    while b:pb.append(b);b=int(sc.m.body_parentid[b])
    return next((k+pb.index(v) for k,v in enumerate(pa) if v in pb),99)
arms=[i for i,n in enumerate(sc.gnames) if any(j in n for j in ['shoulder','elbow','wrist','dex1','camera'])];envs=[sc.m.geom(n).id for n in ['table_uncertainty_box','bottle_uncertainty_box']];pairs=[(a,b) for a in range(sc.m.ngeom) for b in range(a) if a not in envs and b not in envs and (a in arms or b in arms) and hops(int(sc.m.geom_bodyid[a]),int(sc.m.geom_bodyid[b]))>2]
paths=[]
for idx in [2,1,0,3]:
    row=c['rows'][idx]
    if not row['valid']:continue
    goal=out.copy();goal[22:]=row['q']
    for intermediate in [False,True]:
        way=[q0,out]+([mid] if intermediate else [])+[goal];checks=[]
        for leg,(a,b) in enumerate(zip(way[:-1],way[1:])):
            for phase in np.linspace(0,1,81):
                for finger in [-.02,0,.0245]:
                    sc.setq(a+(b-a)*phase,finger);ds=sorted((mujoco.mj_geomDistance(sc.m,sc.d,x,y,.4,None),sc.gnames[x],sc.gnames[y]) for x,y in pairs);env=sc.nearest(arms,envs);checks.append(dict(leg=leg,phase=float(phase),finger=finger,self_nearest=ds[0],table=env))
        minimum_self=min(x['self_nearest'][0] for x in checks if x['leg']>0);minimum_table=min(x['table'][0] for x in checks);passed=minimum_self>.03 and minimum_table>.03
        entry=dict(target_index=idx,via_middle=intermediate,passed=passed,minimum_self_m=minimum_self,minimum_table_m=minimum_table,waypoints=[x.tolist() for x in way],checks=checks);paths.append(entry);print(idx,intermediate,passed,minimum_self,minimum_table,flush=True)
        if passed:break
    if paths[-1]['passed']:break
passed=[p for p in paths if p['passed']];report=dict(candidate_only=True,physical_mount_not_fully_validated=True,table_expanded=True,paths=paths,selected=passed[0] if passed else None);(OUT/'inspection_clearance_candidate.json').write_text(json.dumps(report,indent=2))
