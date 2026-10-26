"""Reference CAD attachment screen; candidate, not a local calibration result."""
from pathlib import Path
import importlib.util,json,numpy as np,struct
p=Path(__file__).with_name('cap_outward_geometry.py');spec=importlib.util.spec_from_file_location('screen',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
root=m.base/'observations/cap_open_20261021/assets/public_g1_dex1';mesh=root/'dex1_1/meshes';cloud=[]
def box(lo,hi):
 vals=[np.linspace(a,b,max(2,int(np.ceil((b-a)/.006))+1)) for a,b in zip(lo,hi)];return np.array(np.meshgrid(*vals,indexing='ij')).reshape(3,-1).T
for file,axis in [('Dex1_base_link.STL',0),('Dex1_finger_link_1.STL',-1),('Dex1_finger_link_2.STL',1)]:
 b=(mesh/file).read_bytes();n=struct.unpack_from('<I',b,80)[0];v=np.ndarray(n,dtype=[('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')],buffer=b,offset=84)['vertices'].reshape(-1,3);lo=v.min(0);hi=v.max(0)
 if axis:lo[1]+=min(axis*(-.02),axis*.0245);hi[1]+=max(axis*(-.02),axis*.0245)
 lo[0]+=.0415;hi[0]+=.0415;cloud.append(box(lo,hi))
# Reference wrist camera at (0.0915,0,.075) in wrist coordinates; additional box is a candidate coverage assumption.
cloud.append(box([.0615,-.030,.045],[.1215,.030,.105]));m.tool=np.concatenate(cloud)
rows=[]
for a in np.linspace(0,.4,21):
 q=m.q0.copy();q[16]+=a;q[23]-=a;rows.append(dict(roll_delta_rad=float(a),geometry=m.evaluate(q)))
out=dict(stage='reference_CAD_attachment_candidate',source=str(root/'source.json'),mount_reference=[.0415,0,0],camera_box_is_unverified=True,full_opening_union=True,local_mount_validated=False,uncertainty_not_included_in_raw_distances=True,rows=rows,execution_permit=False)
(m.r/'reference_geometry_screen.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print('REFERENCE_CAD_SCREEN')
for row in rows[::5]:print(row['roll_delta_rad'],{k:round(v['minimum_candidate_leg_clearance_m'],4) for k,v in row['geometry'].items()},{k:v['candidate_points_near_torso'] for k,v in row['geometry'].items()})
