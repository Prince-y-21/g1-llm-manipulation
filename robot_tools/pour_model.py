"""Kinematics and conservative geometry for the current pouring scene only."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import json, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
urdf=ET.parse(ROOT/'g1_29dof_rev_1_0.urdf').getroot()
state=json.loads((ROOT/'initial_state.json').read_text())
scene=json.loads((ROOT/'scene_estimate.json').read_text())
names=['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']
def chain(side):
 origins=[];axes=[];lo=[];hi=[]
 for name in names:
  j=urdf.find("joint[@name='"+side+'_'+name+"_joint']");o=j.find('origin');t=np.eye(4)
  t[:3,3]=np.fromstring(o.get('xyz'),sep=' ')
  t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_dcm()
  origins.append(t);axes.append(np.fromstring(j.find('axis').get('xyz'),sep=' '))
  lo.append(float(j.find('limit').get('lower')));hi.append(float(j.find('limit').get('upper')))
 return origins,axes,np.array(lo),np.array(hi)
right=chain('right');left=chain('left')
lower=np.maximum(right[2]+.04,[-2.31,-1.91,-2.40,-.90,-1.89,-1.50,-1.50])+.005
upper=np.minimum(right[3]-.04,[.70,.80,2.40,1.80,1.89,1.50,1.50])-.005
def fk(q,which=right):
 t=np.eye(4);pts=[]
 for o,a,v in zip(which[0],which[1],q):
  t=t.dot(o);pts.append(t[:3,3].copy());s=np.eye(4);s[:3,:3]=Rotation.from_rotvec(a*v).as_dcm();t=t.dot(s)
 return t,np.array(pts)
normal=np.array(scene['table_plane_body_normal']);plane=np.array(scene['table_plane_body_point'])
edgepts=np.array(scene['front_edge_points']);edge=np.polyfit(edgepts[:,1],edgepts[:,0],1)
def table_z(x,y):return float(plane[2]-(normal[0]*(x-plane[0])+normal[1]*(y-plane[1]))/normal[2])
cup=np.array([.370,.183,.032]);cup_radius=.050
bottle_xy=np.array([.399,-.008]);bottle_height=.190;grip_height=.095
lt,lp=fk([j['q'] for j in state['joints'][15:22]],left)
left_samples=[]
for a,b in [(lp[1],lp[3]),(lp[3],lp[5]),(lp[5],lp[6])]:
 left_samples.extend([a*(1-w)+b*w for w in np.linspace(0,1,12)])
left_samples.extend([lt[:3,3]+lt[:3,:3].dot([x,y,z]) for x in [.03,.10,.16,.225] for y in [-.05,.05] for z in [-.035,.06]])
left_samples=np.array(left_samples)
def tcp_pose(q):
 t,_=fk(q);return t[:3,3]+t[:3,:3].dot([.19,0,0]),t[:3,:3]
def collision(q,carried=False,grasp=False):
 t,pts=fk(q);samples=[]
 for a,b,rad,self_check in [(pts[1],pts[3],.045,False),(pts[3],pts[5],.04,True),(pts[5],pts[6],.04,True)]:
  for w in np.linspace(0,1,13):samples.append((a*(1-w)+b*w,rad,self_check))
 for x in np.linspace(.025,.225,9):
  hw=.03 if x<.10 else (.045 if carried else .065)
  for y in [-hw,0,hw]:
   for z in [-.04,.035,.09 if x<.14 else .035]:samples.append((t[:3,3]+t[:3,:3].dot([x,y,z]),.008,True))
 clearance=10.
 for p,rad,self_check in samples:
  if p[0]+rad>np.polyval(edge,p[1])-.012:
   h=float(normal.dot(p-plane))-rad;clearance=min(clearance,h)
   if h<.016:return 'TABLE',clearance
  if self_check and abs(p[1])<.11 and -.12<p[2]<.24 and p[0]<.09+rad:return 'TORSO',clearance
  if -.08-rad<p[0]<.12+rad and abs(p[1])<.09+rad and .28-rad<p[2]<.58+rad:return 'HEAD',clearance
  if p[2]-rad<cup[2]+.010 and p[2]+rad>table_z(*cup[:2]):
   if np.linalg.norm(p[:2]-cup[:2])<cup_radius+rad+.012:return 'CUP',clearance
  if np.min(np.linalg.norm(left_samples-p,axis=1))<rad+.035:return 'LEFT_ARM',clearance
  if not grasp and not carried and p[2]-rad<table_z(*bottle_xy)+bottle_height and p[2]+rad>table_z(*bottle_xy):
   if np.linalg.norm(p[:2]-bottle_xy)<.035+rad+.008:return 'BOTTLE',clearance
 if carried:
  g=t[:3,3]+t[:3,:3].dot([.19,0,0])
  for z in np.linspace(-grip_height,bottle_height-grip_height,13):
   p=g+t[:3,:3].dot([0,0,z]);rad=.034 if z<.055 else .020
   lowest=p[2]-rad*np.sqrt(max(0,1-t[2,2]**2))
   if lowest<table_z(*p[:2])-.002:return 'CARRIED_TABLE',clearance
   if lowest<cup[2]+.015 and np.linalg.norm(p[:2]-cup[:2])<cup_radius+rad+.01:return 'CARRIED_CUP',clearance
 return '',clearance
def segment(a,b,carried=False,grasp=False):
 a=np.array(a);b=np.array(b)
 flags=set();clearance=10
 for w in np.linspace(0,1,81):
  f,c=collision(a*(1-w)+b*w,carried,grasp)
  if f:flags.add(f)
  clearance=min(clearance,c)
 return sorted(flags),clearance
def ik(tcp,rot,seed):
 wrist=np.array(tcp)-rot.dot([.19,0,0])
 def residual(q):
  t,pts=fk(q)
  body=[]
  for a,b in [(pts[3],pts[5]),(pts[5],pts[6])]:
   for w in np.linspace(0,1,7):
    p=a*(1-w)+b*w
    outside=max(p[0]-.135,abs(p[1])-.135,p[2]-.245,-.125-p[2])
    body.append(max(0.,.008-outside)*200)
  for a,b in [(pts[1],pts[3]),(pts[3],pts[5]),(pts[5],pts[6])]:
   for w in np.linspace(0,1,9):
    p=a*(1-w)+b*w
    outside=max(p[0]-.170,-.130-p[0],abs(p[1])-.140,p[2]-.630,.230-p[2])
    body.append(max(0.,.006-outside)*500)
  return np.r_[(t[:3,3]-wrist)*800,Rotation.from_dcm(rot.T.dot(t[:3,:3])).as_rotvec()*120,body,(q-seed)*.02]
 sol=least_squares(residual,np.clip(seed,lower+1e-7,upper-1e-7),bounds=(lower,upper),max_nfev=400,ftol=1e-10,xtol=1e-10,gtol=1e-10)
 pos,rr=tcp_pose(sol.x);pe=float(np.linalg.norm(pos-tcp));ae=float(np.linalg.norm(Rotation.from_dcm(rot.T.dot(rr)).as_rotvec()))
 if pe>.001 or ae>.01:raise RuntimeError('IK '+str([pe,ae]))
 return sol.x
if __name__=='__main__':
 q0=np.array([j['q'] for j in state['joints'][22:29]])
 points=[]
 def add(label,q,carried=False,grasp=False):
  flags,c=segment(points[-1]['q'],q,carried,grasp) if points else ([],10)
  pos,rr=tcp_pose(q);f,_=collision(q,carried,grasp)
  x=dict(label=label,q=np.array(q).tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=carried,grasp=grasp,flags=flags,endpoint_flag=f,clearance=c)
  points.append(x);print(label,'flags',flags,'end',f,'tcp',np.round(pos,4).tolist(),flush=True)
 def pose(label,pos,rot,carried=False,grasp=False):
  q=ik(pos,rot,np.array(points[-1]['q']));add(label,q,carried,grasp)
 add('initial',q0)
 q=q0.copy();q[1]=-.30;add('outward',q)
 q[0]=.50;add('retract',q)
 q[3]=0;add('unfold',q)
 q[5]=-.4;add('level',q)
 rot=np.eye(3)
 pose('outboard',[.30,-.29,.05],rot)
 rot=Rotation.from_euler('z',.60).as_dcm()
 pose('bridge',[.31,-.17,.065],rot)
 pose('pregrasp',[.349,-.042,table_z(.399,-.008)+.115],rot,grasp=True)
 pose('grasp',[.399,-.008,table_z(.399,-.008)+grip_height],rot,grasp=True)
 for z in [.04,.10,.18,.22]:pose('lift_'+str(z),[.399,-.008,z],rot,True,True)
 rot=Rotation.from_euler('z',0.).as_dcm()
 pose('pour_rotate',[.399,-.008,.22],rot,True,True)
 pose('pour_approach',[.375,.065,.21],rot,True,True)
 for deg in [15,30,40,45,50,55,60,65,70,75,80,85,90,95,100]:
  rr=Rotation.from_euler('x',-np.deg2rad(deg)).as_dcm().dot(rot)
  mouth=np.array([.370,.160,.25 if deg<=45 else .25-(deg-45)*.0017])
  # Before tipping, keep a lower segment of the bottle away from the cup.
  pos=mouth-rr.dot([0,0,bottle_height-grip_height])
  pose('tilt_'+str(deg),pos,rr,True,True)
 (ROOT/'route_candidate.json').write_text(json.dumps(points,indent=2))
 print('CHECKED',len(points),'unsafe',[(p['label'],p['flags']) for p in points if p['flags']],flush=True)
