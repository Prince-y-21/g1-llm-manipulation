from pathlib import Path
import subprocess,time
base=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z');s=(base/'gripper_small_cycle.cpp').read_text()
changes={'initial-.15f*phase':'initial+.15f*phase','q<initial-.19f||q>initial+.04f':'q<initial-.04f||q>initial+.19f','initial-minimum<.08f':'maximum-initial<.08f'}
for old,new in changes.items():assert old in s;s=s.replace(old,new)
(base/'gripper_positive_cycle.cpp').write_text(s)
# Preserve the first failed direction test; the next bounded test reverses sign without increasing drive limits.
code=(base/'cap_run_left_cycle_source.py').read_text()
old="r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development');src=r.parent/'gripper_small_cycle.cpp';"
new="r=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development/left_positive_test');r.mkdir(exist_ok=False);src=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/gripper_positive_cycle.cpp');"
assert old in code;code=code.replace(old,new);exec(compile(code,'left_positive_cycle','exec'))
