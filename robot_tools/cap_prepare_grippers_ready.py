from pathlib import Path
import subprocess,json
root=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z/bimanual_development')
base=(root/'left_fullstroke90_080/cycle.cpp').read_text()
for side in ['left','right']:
    r=root/(side+'_ready_cycle');r.mkdir(exist_ok=False);s=base
    s=s.replace('target-q>.08f&&std::abs(velocity)<.015f&&torque>.70f','std::abs(target-q)>.08f&&std::abs(velocity)<.015f&&std::abs(torque)>.70f')
    if side=='left':
        s=s.replace('elapsed>104.5f','elapsed>52.5f').replace('elapsed<50.5f?(elapsed-.5f)/50.f:(elapsed<52.5f?1.f:(elapsed<102.5f?1.f-(elapsed-52.5f)/50.f:0.f))','elapsed<50.5f?(elapsed-.5f)/50.f:1.f')
        s=s.replace('std::abs(d.q/g-initial)>.035f','std::abs(d.q/g-(initial+4.6f))>.035f')
    else:
        s=s.replace('/dev/ttyUSB2','/dev/ttyUSB1').replace('c.id=1','c.id=0').replace('initial<-.05||initial>.7','initial<4.9||initial>5.4')
        s=s.replace('elapsed>104.5f','elapsed>64.5f').replace('elapsed<50.5f?(elapsed-.5f)/50.f:(elapsed<52.5f?1.f:(elapsed<102.5f?1.f-(elapsed-52.5f)/50.f:0.f))','elapsed<30.5f?(elapsed-.5f)/30.f:(elapsed<32.5f?1.f:(elapsed<62.5f?1.f-(elapsed-32.5f)/30.f:0.f))')
        s=s.replace('initial+4.6f*phase','initial-3.7f*phase').replace('q<initial-.04f||q>initial+4.64f','q<initial-3.74f||q>initial+.04f').replace('maximum-initial<4.45f','initial-minimum<3.55f')
    assert ('elapsed>52.5f' if side=='left' else 'elapsed>64.5f') in s
    (r/'cycle.cpp').write_text(s);subprocess.check_call(['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/'cycle.cpp'),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/'cycle')],timeout=25)
    runner=(root/'cap_left_force080.py').read_text().replace("r=root/'left_force_step_080'","r=root/'"+side+"_ready_cycle'").replace('start<17','start<70').replace('cv2.VideoCapture(0','cv2.VideoCapture('+('0' if side=='left' else '8')).replace("['fuser','/dev/ttyUSB2']","['fuser','"+('/dev/ttyUSB2' if side=='left' else '/dev/ttyUSB1')+"']")
    (root/('ready_'+side+'.py')).write_text(runner)
(root/'run_ready_grippers.py').write_text('''from pathlib import Path
import subprocess,json
r=Path("'''+str(root)+'''")
children=[]
for side in ["left","right"]:
    log=(r/(side+"_ready_parent.log")).open("w");p=subprocess.Popen(["python3",str(r/("ready_"+side+".py"))],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append((side,p))
results={}
for side,p in children:
    try:code=p.wait(timeout=75)
    except subprocess.TimeoutExpired:p.terminate();code=p.wait(timeout=5)
    results[side]=dict(runner_exit=code,cycle=json.loads((r/(side+"_ready_cycle/result.json")).read_text()) if (r/(side+"_ready_cycle/result.json")).exists() else None)
(r/"ready_grippers_result.json").write_text(json.dumps(results,indent=2));print(results,flush=True)
''')
print('BOTH_EMPTY_GRIPPER_PROFILES_BUILT_NO_COMMANDS')
