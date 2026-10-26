"""Additional lower-body capsule checks, expressed in torso coordinates."""
from pathlib import Path
import json,sys,numpy as np
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');sys.path.insert(0,str(r));import pour_current_model as base
legs=json.loads((r/'leg_geometry.json').read_text())
def leg_clearance(q,carried=False):
 t,pts=base.fk(q);samples=[];radii=[]
 for a,b,rad in [(pts[1],pts[3],.045),(pts[3],pts[5],.04),(pts[5],pts[6],.04)]:
  for w in np.linspace(0,1,13):samples.append(a*(1-w)+b*w);radii.append(rad)
 for x in np.linspace(.025,.225,9):
  hw=.03 if x<.1 else (.045 if carried else .065)
  for y in [-hw,0,hw]:
   for z in [-.04,.035,.09 if x<.14 else .035]:samples.append(t[:3,3]+t[:3,:3].dot([x,y,z]));radii.append(.008)
 p=np.array(samples);rr=np.array(radii);minimum=10.
 for leg in legs.values():
  leg=np.array(leg)
  for a,b,rad in [(leg[0],leg[2],.075),(leg[2],leg[3],.070),(leg[3],leg[5],.060)]:
   ab=b-a;w=np.clip((p-a).dot(ab)/ab.dot(ab),0,1);nearest=a+w[:,None]*ab
   minimum=min(minimum,float(np.min(np.linalg.norm(p-nearest,axis=1)-rr-rad)))
 return minimum
def collision(q,carried=False,grasp=False):
 f,c=base.collision(q,carried,grasp)
 if f:return f,c
 clearance=leg_clearance(q,carried)
 return ('LEG' if clearance<.012 else ''),min(c,clearance)
def segment(a,b,carried=False,grasp=False):
 flags=set();cl=10.;a=np.array(a);b=np.array(b)
 for w in np.linspace(0,1,81):
  f,c=collision(a*(1-w)+b*w,carried,grasp)
  if f:flags.add(f)
  cl=min(cl,c)
 return sorted(flags),cl
