from pathlib import Path
import os,sys,json,time
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');sys.path.insert(0,str(root));import cv2,numpy as np,scipy,mujoco,cap_cad_geometry as geom
geom.ASSET=root/'assets/public_g1_dex1';geom.OUT=root/'planner_validation';geom.OUT.mkdir(exist_ok=True);s=geom.Scene();s.m.opt.disableflags|=mujoco.mjtDisableBit.mjDSBL_CONTACT
test=np.load(root/'geometry_reference.npz');ids=[s.m.joint(n).dofadr[0] for n in geom.joint_names()];errs=[]
for i,q in enumerate(test['q']):
    s.setq(q);lp,lr=s.pose('left');rp,rr=s.pose('right');errs.append([np.max(abs(lp-test['left_p'][i])),np.max(abs(lr-test['left_r'][i])),np.max(abs(rp-test['right_p'][i])),np.max(abs(rr-test['right_r'][i])),np.max(abs(s.d.qfrc_bias[ids]-test['gravity'][i]))])
assert np.max(errs)<1e-5,errs
start=time.monotonic()
for _ in range(1000):s.setq(test['q'][0])
seconds=time.monotonic()-start
record=dict(scope='CPU geometry only, no DDS or motor calls',numpy=np.__version__,scipy=scipy.__version__,mujoco=mujoco.__version__,opencv=cv2.__version__,max_fk_and_gravity_difference=float(np.max(errs)),forward_1000_s=seconds,gl_rendering_disabled=True,libgomp_preloaded=True,dependency_path=str(root/'python_deps'),system_packages_modified=False,explicit_geometry_distance_api=hasattr(mujoco,'mj_geomDistance'))
(root/'runtime_versions.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2),flush=True)
