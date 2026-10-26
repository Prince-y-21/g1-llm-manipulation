"""Rebuild the normal-stop/thermal-tail change and unit tests; no hardware I/O."""
from pathlib import Path
import subprocess,json,hashlib
R=Path('/home/unitree/codex_g1_tasks/paper_to_mousepad_20261025_01')
D=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
assert not (R/'control').exists()
cmd=['g++','-std=c++17','-O2','-Wall','-Wextra','-I'+str(R),'-I'+str(D),'-I/usr/local/include','-I/usr/local/include/ddscxx',str(R/'paper_body_retry_v4.cpp'),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(R/'paper_body_retry_v4')]
p=subprocess.run(cmd,capture_output=True,text=True,timeout=60);(R/'compile_body_tail.log').write_text(p.stdout+p.stderr)
assert p.returncode==0,(p.stdout+p.stderr)[-3000:]
subprocess.check_call(['g++','-std=c++17','-O2',str(R/'test_thermal_v4.cpp'),'-o',str(R/'test_thermal_v4')],timeout=20)
subprocess.check_call([str(R/'test_thermal_v4')],timeout=5)
versions=json.loads((R/'versions.json').read_text())
for name in ['paper_body_retry_v4.cpp','paper_body_retry_v4','retry_thermal_v4.hpp']:
 versions[name]=hashlib.sha256((R/name).read_bytes()).hexdigest()
(R/'versions.json').write_text(json.dumps(versions,indent=2));print('REBUILT_BODY_TAIL_AND_TESTS_OK_NO_PUBLISHER',flush=True)
