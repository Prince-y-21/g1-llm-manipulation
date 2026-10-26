"""Offline CAD scene. Reference assembly, explicitly not a physical calibration."""
from pathlib import Path
import copy,json,xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent.parent
ASSET=ROOT/'observations/cap_open_20261021/assets/public_g1_dex1'
OUT=ROOT/'observations/cap_open_20261021/resume'

def make_model():
    u=ET.fromstring((ASSET/'g1_29dof_with_dex1_1.urdf').read_text())
    compiler=u.find('mujoco/compiler');compiler.set('fusestatic','false');compiler.set('discardvisual','true')
    # Complete visual shapes cover the rails omitted by the source finger collision mesh.
    # Convex hulls are conservative for these rigid components; assembly remains unverified.
    for link in u.findall('link'):
        visual=link.findall('visual')
        if visual:
            for c in link.findall('collision'):link.remove(c)
            for v in visual:
                c=copy.deepcopy(v);c.tag='collision';c.set('name',link.get('name')+'_full');mat=c.find('material')
                if mat is not None:c.remove(mat)
                link.append(c);link.remove(v)
    for side in ['left','right']:
        link=u.find("link[@name='%s_wrist_yaw_link']"%side)
        c=ET.SubElement(link,'collision',name=side+'_camera_candidate')
        ET.SubElement(c,'origin',xyz='.0915 0 .075',rpy='0 0 0')
        ET.SubElement(ET.SubElement(c,'geometry'),'box',size='.060 .060 .060')
    # Sensor mounting difference does not alter arm FK. Preserve previous body model frame.
    j=u.find("joint[@name='mid360_joint']/origin")
    if j is not None:j.set('xyz','.0002835 .00003 .428434');j.set('rpy','3.141592654 .0511207 0')
    text=ET.tostring(u,encoding='unicode');(OUT/'cad_candidate.urdf').write_text(text)
    assets={str(f.relative_to(ASSET)).replace('\\','/'):f.read_bytes() for f in ASSET.rglob('*.STL')}
    return mujoco.MjModel.from_xml_string(text,assets=assets)

def joint_names():
    leg=['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll']
    arm=['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']
    return [s+'_'+j+'_joint' for s in ['left','right'] for j in leg]+['waist_'+j+'_joint' for j in ['yaw','roll','pitch']]+[s+'_'+j+'_joint' for s in ['left','right'] for j in arm]

class Scene:
    def __init__(self):
        self.m=make_model();self.d=mujoco.MjData(self.m)
        self.qids=[self.m.joint(n).qposadr[0] for n in joint_names()]
        self.gnames=[self.m.geom(i).name for i in range(self.m.ngeom)]
        self.tool={s:[i for i,n in enumerate(self.gnames) if n.startswith(s+'_dex1') or n==s+'_camera_candidate'] for s in ['left','right']}
        self.legs=[i for i,n in enumerate(self.gnames) if any(j in n for j in ['hip_','knee_','ankle_','pelvis'])]
        self.torso=[i for i,n in enumerate(self.gnames) if any(j in n for j in ['torso','waist','head','logo','mid360'])]
    def setq(self,q,finger=0.):
        self.d.qpos[self.qids]=q
        for s in ['left','right']:
            for k in [1,2]:self.d.qpos[self.m.joint(s+'_dex1_finger_joint_'+str(k)).qposadr[0]]=finger
        mujoco.mj_forward(self.m,self.d)
    def nearest(self,a,b):
        results=[]
        for x in a:
            for y in b:
                if x==y:continue
                dist=mujoco.mj_geomDistance(self.m,self.d,x,y,1.,None)
                results.append((dist,self.gnames[x],self.gnames[y]))
        return min(results) if results else None
    def tool_clearance(self):
        return {s:dict(legs=self.nearest(self.tool[s],self.legs),torso=self.nearest(self.tool[s],self.torso),other_tool=self.nearest(self.tool[s],self.tool['right' if s=='left' else 'left'])) for s in ['left','right']}
    def pose(self,side):
        b=self.m.body(side+'_wrist_yaw_link').id;t=self.m.body('torso_link').id
        r=self.d.xmat[t].reshape(3,3);w=self.d.xmat[b].reshape(3,3)
        return r.T@(self.d.xpos[b]-self.d.xpos[t]),r.T@w

if __name__=='__main__':
    sc=Scene();state=json.loads((OUT/'post_microtest_state.json').read_text());q=np.array([j['q'] for j in state['joints']]);rows=[]
    for delta in np.linspace(0,.4,21):
        p=q.copy();p[16]+=delta;p[23]-=delta;frows=[]
        for finger in [-.020,0,.0245]:
            sc.setq(p,finger);frows.append(dict(finger_m=finger,distances=sc.tool_clearance()))
        rows.append(dict(roll_delta_rad=float(delta),opening_samples=frows))
    record=dict(reference_only=True,physical_mount_not_validated=True,camera_box_unverified=True,uncertainty_margin_not_applied=True,all_visual_meshes_used=True,nbody=sc.m.nbody,ngeom=sc.m.ngeom,rows=rows)
    (OUT/'cad_outward_screen.json').write_text(json.dumps(record,indent=2))
    for row in rows[::5]:print(row['roll_delta_rad'],json.dumps(row['opening_samples']))
