"""Task-local CPU geometry dependencies. No robot interfaces or system changes."""
from pathlib import Path
import subprocess,sys,os,json,time,tarfile
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');dep=root/'python_deps';wheels=sorted((root/'wheelhouse').glob('*.whl'));assert len(wheels)>=9
subprocess.check_call([sys.executable,'-m','pip','install','--disable-pip-version-check','--no-index','--no-deps','--no-compile','--target',str(dep)]+[str(p) for p in wheels],timeout=100)
with tarfile.open(root/'runtime_assets.tgz') as archive:
    for member in archive.getmembers():assert not member.name.startswith('/') and '..' not in Path(member.name).parts
    archive.extractall(root)
os.environ['MUJOCO_GL']='disable';sys.path.insert(0,str(dep));import numpy,scipy,mujoco,cv2
assert hasattr(mujoco,'mj_geomDistance')
m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size=".1"/><geom type="sphere" size=".1" pos="1 0 0"/></worldbody></mujoco>');d=mujoco.MjData(m);mujoco.mj_forward(m,d);distance=mujoco.mj_geomDistance(m,d,0,1,2.,None);assert abs(distance-.8)<1e-6
record=dict(scope='CPU geometry only, no publisher',python=sys.version,numpy=numpy.__version__,scipy=scipy.__version__,mujoco=mujoco.__version__,opencv=cv2.__version__,sphere_distance_test=distance,rendering_disabled=True,dependency_path=str(dep),system_packages_modified=False)
(root/'runtime_versions.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2),flush=True)
