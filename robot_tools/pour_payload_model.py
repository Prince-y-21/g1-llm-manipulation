"""Vectorized geometry with observed payload-in-wrist registration."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys,json,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_current_model as b
cal=json.loads((r/'payload_calibration.json').read_text())
normal=np.array(cal['table_normal']);plane=np.array(cal['table_point']);edge=np.array(cal['table_edge']);cup=np.array(cal['cup']);cup_radius=cal['cup_radius'];H=cal['bottle_height'];mouth_local=np.array(cal['mouth_local']);axis_local=np.array(cal['axis_local'])
lower=b.lower;upper=b.upper;fk=b.fk
state=cal['grasp_state'];lt,lp=b.fk(state['q'][15:22],b.left)
other=[]
for a,z in [(lp[1],lp[3]),(lp[3],lp[5]),(lp[5],lp[6])]:other.extend(a*(1-w)+z*w for w in np.linspace(0,1,12))
other.extend(lt[:3,3]+lt[:3,:3].dot([x,y,z]) for x in [.03,.10,.16,.225] for y in [-.05,.05] for z in [-.035,.06]);other=np.array(other)
legs=json.loads((r/'leg_geometry.json').read_text());caps=[]
for leg in legs.values():
 leg=np.array(leg)
 for i,j,rad in [(0,2,.075),(2,3,.070),(3,5,.060)]:caps.append((leg[i],leg[j],rad))
def table_z(x,y):return float(plane[2]-(normal[0]*(x-plane[0])+normal[1]*(y-plane[1]))/normal[2])
def payload(q):
 t,pts=fk(q);return t[:3,3]+t[:3,:3].dot(mouth_local),t[:3,:3].dot(axis_local)
def radius(s):return np.where(s<.022,.013,np.where(s<.054,.013+(s-.022)/.032*.021,.034))
def tooltemplate(carried):
 vals=[]
 for x in np.linspace(.025,.225,9):
  hw=.03 if x<.1 else (.045 if carried else .065)
  for y in [-hw,0,hw]:
   for z in [-.04,.035,.09 if x<.14 else .035]:vals.append([x,y,z])
 vals=np.array(vals)
 offset=mouth_local-axis_local*(H-.105)-[.19,0,0]
 offset=np.clip(offset,[-.04,-.07,-.03],[.06,.07,.03])
 return np.concatenate([vals,vals+offset])
templates={False:tooltemplate(False),True:tooltemplate(True)}
EMPTY_CLOSED=False
def collision(q,carried=False,grasp=False):
 t,pts=fk(q);arr=[];rads=[];selfcheck=[]
 for a,z,rad,sc in [(pts[1],pts[3],.045,False),(pts[3],pts[5],.04,True),(pts[5],pts[6],.04,True)]:
  ww=np.linspace(0,1,13)[:,None];arr.append(a*(1-ww)+z*ww);rads.extend([rad]*13);selfcheck.extend([sc]*13)
 shape=templates[carried or EMPTY_CLOSED];arr.append(shape.dot(t[:3,:3].T)+t[:3,3]);rads.extend([.008]*len(shape));selfcheck.extend([True]*len(shape))
 p=np.concatenate(arr);rad=np.array(rads);sc=np.array(selfcheck);height=(p-plane).dot(normal)-rad
 desk=p[:,0]+rad>np.polyval(edge,p[:,1])-.012;clear=float(np.min(height[desk])) if np.any(desk) else 10.
 if np.any(desk&(height<.016)):return 'TABLE',clear
 if np.any(sc&(abs(p[:,1])<.11)&(p[:,2]>-.12)&(p[:,2]<.24)&(p[:,0]<.09+rad)):return 'TORSO',clear
 head=(p[:,0]>-.08-rad)&(p[:,0]<.12+rad)&(abs(p[:,1])<.09+rad)&(p[:,2]>.28-rad)&(p[:,2]<.58+rad)
 if np.any(head):return 'HEAD',clear
 dr=p-cup;ax=dr.dot(normal);horizontal=dr-ax[:,None]*normal
 if np.any((ax-rad<.010)&((p-plane).dot(normal)+rad>0)&(np.linalg.norm(horizontal,axis=1)<cup_radius+rad+.012)):return 'CUP',clear
 if np.any(np.linalg.norm(p[:,None,:]-other[None,:,:],axis=2).min(1)<rad+.035):return 'OTHER_ARM',clear
 for a,z,rr in caps:
  ab=z-a;w=np.clip((p-a).dot(ab)/ab.dot(ab),0,1);distance=np.linalg.norm(p-(a+w[:,None]*ab),axis=1)-rad-rr
  if np.any(distance<.012):return 'LEG',float(np.min(distance))
 if carried:
  mouth,axis=payload(q);ss=np.linspace(0,H,31);rr=radius(ss);bp=mouth-ss[:,None]*axis;sin=float(np.sqrt(max(0.,1-float(normal.dot(axis))**2)))
  bh=(bp-plane).dot(normal)-rr*sin
  if np.any(bh<-.002):return 'BOTTLE_TABLE',float(np.min(bh))
  dr=bp-cup;ax=dr.dot(normal);horizontal=dr-ax[:,None]*normal
  if np.any((ax-rr*sin<.015)&(np.linalg.norm(horizontal,axis=1)<cup_radius+rr+.010)):return 'BOTTLE_CUP',clear
  if np.any((bp[:,0]>-.08-rr)&(bp[:,0]<.12+rr)&(abs(bp[:,1])<.09+rr)&(bp[:,2]>.28-rr)&(bp[:,2]<.58+rr)):return 'BOTTLE_HEAD',clear
  if np.any((abs(bp[:,1])<.11+rr)&(bp[:,2]>-.12-rr)&(bp[:,2]<.24+rr)&(bp[:,0]<.09+rr+.012)):return 'BOTTLE_TORSO',clear
 elif not grasp:
  bottom=np.array(cal['mouth_body'])-normal*H;dr=p-bottom;ax=dr.dot(normal);hor=dr-ax[:,None]*normal
  if np.any((ax+rad>0)&(ax-rad<H)&(np.linalg.norm(hor,axis=1)<.034+rad+.008)):return 'BOTTLE',clear
 return '',clear
def segment(a,z,carried=False,grasp=False):
 a=np.array(a);z=np.array(z);flags=set();cl=10.
 for w in np.linspace(0,1,81):
  f,c=collision(a*(1-w)+z*w,carried,grasp)
  if f:flags.add(f)
  cl=min(cl,c)
 return sorted(flags),cl
