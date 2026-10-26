from pathlib import Path
import sys,json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');root=ET.parse(r/'g1_29dof_rev_1_0.urdf').getroot();s=json.loads((r/'guard_status.json').read_text())
def transform(name,value):
 j=root.find("joint[@name='"+name+"']");o=j.find('origin');T=np.eye(4);T[:3,3]=np.fromstring(o.get('xyz'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_dcm();a=np.fromstring(j.find('axis').get('xyz'),sep=' ');R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(a*value).as_dcm();return T.dot(R)
waist=np.eye(4)
for name,i in [('waist_yaw_joint',12),('waist_roll_joint',13),('waist_pitch_joint',14)]:waist=waist.dot(transform(name,s['q'][i]))
out={}
for side,start in [('left',0),('right',6)]:
 T=np.eye(4);pts=[]
 for name,i in zip(['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll'],range(start,start+6)):
  T=T.dot(transform(side+'_'+name+'_joint',s['q'][i]));p=np.linalg.inv(waist).dot(T)[:3,3];pts.append(p.tolist())
 out[side]=pts
(r/'leg_geometry.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
