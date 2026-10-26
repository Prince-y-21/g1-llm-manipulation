"""Offline candidate test only. Fails closed when attachment geometry cannot be cleared."""
from pathlib import Path
import xml.etree.ElementTree as ET,json,numpy as np
base=Path(__file__).resolve().parent.parent;r=base/'observations/cap_open_20261021/resume';u=ET.parse(base/'observations/g1_29dof_rev_1_0.urdf').getroot();s=json.loads((r/'resume_initial.json').read_text());q0=np.array([x['q'] for x in s['joints']])
def rot(a,v):
 a=np.array(a,dtype=float);a/=np.linalg.norm(a);k=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]]);return np.eye(3)+np.sin(v)*k+(1-np.cos(v))*k.dot(k)
def transform(name,val):
 j=u.find("joint[@name='"+name+"']");o=j.find('origin');rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ');t=np.eye(4);t[:3,3]=np.fromstring(o.get('xyz'),sep=' ');t[:3,:3]=rot([0,0,1],rpy[2])@rot([0,1,0],rpy[1])@rot([1,0,0],rpy[0]);a=np.eye(4);a[:3,:3]=rot(np.fromstring(j.find('axis').get('xyz'),sep=' '),val);return t@a
waist=np.eye(4)
for i,name in enumerate(['yaw','roll','pitch']):waist=waist@transform('waist_'+name+'_joint',q0[12+i])
caps=[]
for side,off in [('left',0),('right',6)]:
 t=np.eye(4);pts=[]
 for i,name in enumerate(['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll']):t=t@transform(side+'_'+name+'_joint',q0[off+i]);pts.append((np.linalg.inv(waist)@t)[:3,3])
 for i,j,rad in [(0,2,.095),(2,3,.090),(3,5,.080)]:caps.append((pts[i],pts[j],rad))
names=['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw'];tool=np.array([[x,y,z] for x in np.linspace(0,.26,14) for y in np.linspace(-.095,.095,7) for z in np.linspace(-.10,.10,7)])
def evaluate(q):
 result={}
 for side,off in [('left',15),('right',22)]:
  t=np.eye(4)
  for i,name in enumerate(names):t=t@transform(side+'_'+name+'_joint',q[off+i])
  p=tool@t[:3,:3].T+t[:3,3];mins=[]
  for a,b,rad in caps:
   ab=b-a;w=np.clip((p-a)@ab/(ab@ab),0,1);mins.append(float(np.min(np.linalg.norm(p-(a+w[:,None]*ab),axis=1)-rad)))
  result[side]=dict(wrist_torso=t[:3,3].tolist(),minimum_candidate_leg_clearance_m=min(mins),candidate_points_near_torso=int(np.sum((abs(p[:,1])<.13)&(p[:,0]<.10)&(p[:,2]>-.15)&(p[:,2]<.24))))
 return result
rows=[]
for a in np.linspace(0,.02,21):
 q=q0.copy();q[16]+=a;q[23]-=a;rows.append(dict(roll_delta_rad=float(a),geometry=evaluate(q)))
out=dict(stage='offline_clearance_screen_no_actuation',initial=rows[0],final=rows[-1],scope='Candidate wrist box 0..260mm x, +/-95mm y, +/-100mm z; widened leg capsules. Mount and exact coverage unverified; neither a certified model nor a contact-motion permit.',rows=rows,passed=False,reason='Actual mounting transform and gripper geometry require independent validation; any initial candidate overlap prevents treating this as an ordinary collision-free path')
(r/'outward_geometry_screen.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in out.items() if k!='rows'},ensure_ascii=False,indent=2))
