from pathlib import Path
import subprocess,concurrent.futures
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');old=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
s=(old/'task_guard_fast.cpp').read_text().replace(str(old),str(r))
s=s.replace('bool moving=false,fault=false,releasing=false;','bool moving=false,fault=false,releasing=false,pulsing=false;float pulse_hold=.25f,pulse_peak_response=0;')
anchor='  if(cycle%50==0&&!fault&&!moving&&!releasing&&Clock::now()-started>std::chrono::seconds(3)){'
assert s.count(anchor)==1
pulse='''
  if(cycle%25==0&&!fault&&!moving&&!releasing&&Clock::now()-started>std::chrono::seconds(8)){
   std::ifstream f(dir+"pulse_target.txt");long n=0;float d=0,h=0;std::array<float,7> q{};
   if(f>>n>>d>>h&&n>seq){bool valid=std::isfinite(d)&&d>=.8f&&d<=10&&std::isfinite(h)&&h>=0&&h<=.5f;
    const std::array<float,7> lo={-2.31f,-1.91f,-2.40f,-.9f,-1.89f,-1.5f,-1.5f};
    const std::array<float,7> hi={.7f,.8f,2.40f,1.8f,1.89f,1.5f,1.5f};
    for(int j=0;j<7;++j){if(!(f>>q[j]))valid=false;float delta=std::abs(q[j]-desired[j+22]);if(!std::isfinite(q[j])||q[j]<lo[j]||q[j]>hi[j]||delta>.25f||delta*1.5f/d>.08f)valid=false;}
    seq=n;if(valid){accepted_seq=n;for(int j=0;j<7;++j){from[j]=desired[j+22];observed_from[j]=s.motor_state()[j+22].q();}target=q;duration=d;pulse_hold=h;pulse_peak_response=0;pulsing=true;moving=true;motion_start=Clock::now();std::cout<<"PULSE_ACCEPTED seq="<<n<<" duration="<<d<<" hold="<<h<<std::endl;}
    else std::cerr<<"PULSE_REJECTED seq="<<n<<std::endl;
   }
  }
'''
s=s.replace(anchor,pulse+anchor)
s=s.replace('if(valid){accepted_seq=newseq;', 'if(valid){pulsing=false;accepted_seq=newseq;')
a='''   phase=std::clamp(phase,0.f,1.f);float blend=phase*phase*(3-2*phase);
   for(int j=0;j<7;++j)desired[j+22]=from[j]+(target[j]-from[j])*blend;
   settling=phase>=1;
   if(elapsed>=duration+2.f){moving=false;float max_error=0;
    for(int j=0;j<7;++j)max_error=std::max(max_error,std::abs(s.motor_state()[j+22].q()-target[j]));'''
b='''   float total_duration=duration;
   if(pulsing){
    total_duration=2*duration+pulse_hold;
    phase=elapsed<duration?elapsed/duration:(elapsed<duration+pulse_hold?1.f:1.f-(elapsed-duration-pulse_hold)/duration);
    float norm=0;for(int j=0;j<7;++j){float x=s.motor_state()[j+22].q()-observed_from[j];norm+=x*x;}pulse_peak_response=std::max(pulse_peak_response,norm);
   }
   phase=std::clamp(phase,0.f,1.f);float blend=phase*phase*(3-2*phase);
   for(int j=0;j<7;++j)desired[j+22]=from[j]+(target[j]-from[j])*blend;
   settling=elapsed>=total_duration;
   if(elapsed>=total_duration+(pulsing?.7f:2.f)){moving=false;float max_error=0;
    for(int j=0;j<7;++j)max_error=std::max(max_error,std::abs(s.motor_state()[j+22].q()-(pulsing?from[j]:target[j])));'''
assert a in s;s=s.replace(a,b)
s=s.replace('bool response=requested_norm2<.0001f||measured_norm2>=.09f*requested_norm2;','bool response=requested_norm2<.0001f||(pulsing?pulse_peak_response:measured_norm2)>=.09f*requested_norm2;')
# Additional measured effort and pulse flag in status, using existing atomic rename.
anchor='f<<"],\\\"temperatures\\\":[";'
assert anchor in s
s=s.replace(anchor,'   f<<"],\\\"pulsing\\\":"<<(pulsing?"true":"false")<<",\\\"measured_tau\\\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].tau_est();}f<<"],\\\"temperatures\\\":[";')
(r/'pour_guard_pulse.cpp').write_text(s)
s=(old/'task_gripper_wide.cpp').read_text().replace(str(old),str(r))
s=s.replace('SerialPort serial(port,16,6000000);','SerialPort serial(port,16,6000000);std::this_thread::sleep_for(std::chrono::milliseconds(100));')
s=s.replace('MotorData d;d.motorType=c.motorType;','MotorData d;d.motorType=c.motorType;d.correct=false;d.merror=0;d.temp=0;')
(r/'pour_right_gripper.cpp').write_text(s)
def build(name):
 if name=='pour_right_gripper':args=['g++','-std=c++17','-O2','-I/home/unitree/dex1_1_service/include',str(r/(name+'.cpp')),'-L/home/unitree/dex1_1_service/lib','-lUnitreeMotorSDK_Arm64','-Wl,-rpath,/home/unitree/dex1_1_service/lib','-lpthread','-o',str(r/name)]
 else:args=['g++','-std=c++17','-O2','-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/(name+'.cpp')),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/name)]
 subprocess.check_call(args);return name+' BUILT_NOT_STARTED'
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for result in pool.map(build,['pour_guard_pulse','pour_right_gripper']):print(result,flush=True)
