// G1 debug-position control. No locomotion switching. Dry run by default.
// IMPORTANT: stopping lowcmd can relax the robot after ReleaseMode.
// Real execution requires a confirmed, fully load-bearing suspension rig.
// Commands: seq duration_seconds q22 q23 q24 q25 q26 q27 q28.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

using Clock=std::chrono::steady_clock;
using State=unitree_hg::msg::dds_::LowState_;
using Cmd=unitree_hg::msg::dds_::LowCmd_;
std::atomic<bool> stop_requested{false};
void on_signal(int){stop_requested=true;}
const std::string dir="/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/";
// Match Unitree G1 terminations.hpp software helper defaults.
// These are not a statement of the motor firmware's trip limits.
constexpr int casing_cutoff=85;
constexpr int winding_cutoff=120;
const std::array<float,29> kp={60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,60,60,60,60,40,40,40,60,60,60,60,40,40,40};
const std::array<float,29> kd={1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1.5,1.5,1.5,1.5,1,1,1,1.5,1.5,1.5,1.5,1,1,1};

int main(int argc,char** argv){
 bool rest_left=argc==2&&std::string(argv[1])=="--handoff-suspended-rest-left";
 bool recenter=argc==2&&std::string(argv[1])=="--handoff-suspended-recenter";
 bool handoff=recenter||rest_left||(argc==2&&std::string(argv[1])=="--handoff-suspended");
 bool execute=handoff||(argc==2&&std::string(argv[1])=="--execute-suspended");
 if(argc>1&&!execute)return 1;
 int fd=open((dir+"lowlevel_guard.lock").c_str(),O_CREAT|O_RDWR,0600);
 if(fd<0){std::cerr<<"LOCK_ERROR"<<std::endl;return 2;}
 bool owns_lock=flock(fd,LOCK_EX|LOCK_NB)==0;
 if((!handoff&&!owns_lock)||(handoff&&owns_lock)){std::cerr<<"UNEXPECTED_GUARD_OWNERSHIP"<<std::endl;return 2;}
 signal(SIGINT,on_signal);signal(SIGTERM,on_signal);
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 std::mutex mutex;State latest;Cmd previous_command;Clock::time_point last{};
 std::atomic<int> states{0},bad_crc{0},other_commands{0};
 std::deque<uint32_t> own_crcs;
 unitree::robot::ChannelSubscriber<State> sub("rt/lowstate");
 sub.InitChannel([&](const void* p){State s=*(const State*)p;
  if(s.crc()!=crc32_core((uint32_t*)&s,(sizeof(s)>>2)-1)){++bad_crc;return;}
  std::lock_guard<std::mutex> g(mutex);latest=s;last=Clock::now();++states;
 },1);
 unitree::robot::ChannelSubscriber<Cmd> cmd_sub("rt/lowcmd");
 cmd_sub.InitChannel([&](const void* p){auto& c=*(const Cmd*)p;
  std::lock_guard<std::mutex> g(mutex);
  if(std::find(own_crcs.begin(),own_crcs.end(),c.crc())==own_crcs.end()){previous_command=c;++other_commands;}
 },1);
 std::this_thread::sleep_for(std::chrono::seconds(3));
 State start;
 {std::lock_guard<std::mutex> g(mutex);start=latest;}
 if(states<200||bad_crc||(!handoff&&other_commands)||start.mode_machine()!=5||start.mode_pr()!=0){
  std::cerr<<"PRECHECK_FAILED states="<<states.load()<<" bad_crc="<<bad_crc.load()<<" other_commands="<<other_commands.load()<<std::endl;return 2;}
 std::array<float,29> initial{},desired{},torque{},hold_correction{},unwind{};
 for(int i=0;i<29;++i){auto m=start.motor_state()[i];
  if(!std::isfinite(m.q())||!std::isfinite(m.tau_est())||std::abs(m.q())>(i<15?.3f:2.6f)||std::abs(m.dq())>.1||std::abs(m.tau_est())>10||m.motorstate()!=0||m.temperature()[0]>casing_cutoff||m.temperature()[1]>winding_cutoff){
   std::cerr<<"INITIAL_STATE_REJECTED joint="<<i<<" q="<<m.q()<<" dq="<<m.dq()<<" temperatures="<<m.temperature()[0]<<","<<m.temperature()[1]<<" status="<<m.motorstate()<<std::endl;return 2;}
  initial[i]=desired[i]=m.q();torque[i]=m.tau_est();
 }
 Cmd transferred;
 float shoulder_kp_start=kp[22],shoulder_kd_start=kd[22];
 if(handoff){
  {std::lock_guard<std::mutex> g(mutex);transferred=previous_command;}
  if(other_commands<500||transferred.mode_machine()!=5||transferred.mode_pr()!=0||transferred.crc()!=crc32_core((uint32_t*)&transferred,(sizeof(transferred)>>2)-1)){
   std::cerr<<"HANDOFF_INVALID_SOURCE"<<std::endl;return 2;
  }
  for(int i=0;i<29;++i){auto m=transferred.motor_cmd()[i];
   bool gains_ok=i==22?(m.kp()>=60&&m.kp()<=100&&m.kd()>=1.5&&m.kd()<=2.5):(m.kp()==kp[i]&&m.kd()==kd[i]);
   if(m.mode()!=1||!gains_ok||!std::isfinite(m.q())||std::abs(m.q()-initial[i])>(recenter?.04f:.03f)||!std::isfinite(m.tau())||std::abs(m.tau())>10){std::cerr<<"HANDOFF_TARGET_MISMATCH "<<i<<std::endl;return 2;}
   if(i==22){shoulder_kp_start=m.kp();shoulder_kd_start=m.kd();}
   desired[i]=m.q();torque[i]=m.tau();
   if(recenter){
    // Transfer the existing static PD effort to feedforward. At the sampled
    // position the old and new total motor torques are algebraically equal.
    float old_total=m.tau()+m.kp()*(m.q()-initial[i]);
    if(!std::isfinite(old_total)||std::abs(old_total)>10){std::cerr<<"RECENTER_TORQUE_REJECTED "<<i<<std::endl;return 2;}
    desired[i]=initial[i];torque[i]=old_total;
    std::cout<<"RECENTER joint="<<i<<" delta_q="<<desired[i]-m.q()<<" old_ff="<<m.tau()<<" new_ff="<<old_total<<std::endl;
   }
  }
  std::ifstream unwind_file(dir+"handoff_unwind.txt");
  if(unwind_file){for(int i=0;i<29;++i){if(!(unwind_file>>unwind[i])||!std::isfinite(unwind[i])||std::abs(unwind[i])>1.5f){std::cerr<<"UNWIND_INVALID"<<std::endl;return 2;}}}
 }
 std::cout<<"PRECHECK_OK all_29_targets_equal_live_position CRC=valid no_competing_lowcmd"<<std::endl;
 if(!execute){std::cout<<"DRY_RUN_NO_PUBLISHER"<<std::endl;return 0;}
 if((!handoff&&access((dir+"arm_target.txt").c_str(),F_OK)==0)||access((dir+"stop_guard").c_str(),F_OK)==0){
  std::cerr<<"STALE_CONTROL_FILE_ABORT"<<std::endl;return 2;}
 unitree::robot::ChannelPublisher<Cmd> pub("rt/lowcmd");pub.InitChannel();
 Cmd command;if(handoff)command=transferred;
 command.mode_pr()=0;command.mode_machine()=start.mode_machine();
 long seq=0,accepted_seq=0;
 if(handoff){
  std::ifstream target_file(dir+"arm_target.txt");target_file>>seq;
  // The old guard continues publishing during preparation; only this short
  // lock transfer interrupts the stream, without a posture reset.
  std::ofstream stop(dir+"stop_guard");stop<<"handoff";stop.close();
  bool acquired=false;
  for(int k=0;k<100;++k){if(flock(fd,LOCK_EX|LOCK_NB)==0){acquired=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  if(!acquired){std::cerr<<"HANDOFF_LOCK_TIMEOUT"<<std::endl;return 2;}
  unlink((dir+"stop_guard").c_str());
  {std::lock_guard<std::mutex> g(mutex);own_crcs.push_back(transferred.crc());other_commands=0;}
  std::cout<<"HANDOFF_COMPLETE_WITH_UNCHANGED_TARGETS"<<std::endl;
 }
 std::array<float,7> from{},target{},observed_from{};
 for(int j=0;j<7;++j)from[j]=target[j]=desired[j+22];
 bool moving=false,fault=false,releasing=false;std::string reason;
 long recenter_seq=0;
 auto release_start=Clock::now();
 float duration=0;auto motion_start=Clock::now();
 auto started=Clock::now(),next=Clock::now();int cycle=0,error_cycles=0;
 float left_rest_angle=0;
 for(;;){
  State s;Clock::time_point received;
  {std::lock_guard<std::mutex> g(mutex);s=latest;received=last;}
  if(Clock::now()-received>std::chrono::milliseconds(150)||s.mode_machine()!=5||s.mode_pr()!=0){
   std::cerr<<"TRANSMISSION_STOPPED_STALE_STATE_OR_MODE_CHANGE"<<std::endl;break;}
  if(other_commands.load()>0){std::cerr<<"TRANSMISSION_STOPPED_COMPETING_CONTROLLER"<<std::endl;break;}
  if(stop_requested||access((dir+"stop_guard").c_str(),F_OK)==0){
   std::cout<<"TRANSMISSION_STOPPED_BY_OPERATOR"<<std::endl;break;}
  if(!fault&&access((dir+"pause_request").c_str(),F_OK)==0){
   fault=true;moving=false;reason="REQUESTED_PAUSE";
   for(int i=0;i<29;++i)desired[i]=s.motor_state()[i].q();
   std::cerr<<"FAULT_HOLD REQUESTED_PAUSE"<<std::endl;
  }
  bool bad=false;std::string condition;
  for(int i=0;i<29;++i){auto m=s.motor_state()[i];
   if(!std::isfinite(m.q())||!std::isfinite(m.dq())){bad=true;condition="NONFINITE_JOINT_"+std::to_string(i);}
   if(m.motorstate()!=0){bad=true;condition="MOTOR_FAULT_JOINT_"+std::to_string(i)+"_"+std::to_string(m.motorstate());}
   if(std::abs(m.dq())>.5){bad=true;condition="SPEED_JOINT_"+std::to_string(i)+"_"+std::to_string(m.dq());}
   if(m.temperature()[0]>casing_cutoff||m.temperature()[1]>winding_cutoff){bad=true;condition="TEMPERATURE_JOINT_"+std::to_string(i)+"_"+std::to_string(m.temperature()[0])+"_"+std::to_string(m.temperature()[1]);}
   if(std::abs(m.q()-desired[i])>(i<15?.035f:.065f)){bad=true;condition="TRACKING_JOINT_"+std::to_string(i)+"_"+std::to_string(m.q()-desired[i]);}
  }
  error_cycles=bad?error_cycles+1:0;
  if(error_cycles>25&&!fault){fault=true;moving=false;
   reason=condition;
   for(int i=0;i<29;++i)desired[i]=s.motor_state()[i].q();
   std::cerr<<"FAULT_HOLD "<<reason<<std::endl;
  }
  if(rest_left&&!fault){
   float phase=std::clamp((std::chrono::duration<float>(Clock::now()-started).count()-3.f)/30.f,0.f,1.f);
   left_rest_angle=.95f*phase*phase*(3-2*phase);
   desired[18]=initial[18]+left_rest_angle;
   desired[20]=initial[20]*(1-phase);
  }
  if(cycle%50==0&&!fault&&!moving&&!releasing&&Clock::now()-started>std::chrono::seconds(8)){
   std::ifstream request_file(dir+"recenter_request.txt");long request=0;
   if(request_file>>request&&request>recenter_seq){
    bool valid=true;std::array<float,7> effort{};
    for(int j=0;j<7;++j){int i=j+22;auto state=s.motor_state()[i];
     float gain=i==22?100.f:kp[i];
     effort[j]=torque[i]+gain*(desired[i]-state.q());
     if(std::abs(state.dq())>.05f||std::abs(desired[i]-state.q())>.04f||!std::isfinite(effort[j])||std::abs(effort[j])>10.f)valid=false;
    }
    if(valid){
     for(int j=0;j<7;++j){desired[j+22]=s.motor_state()[j+22].q();torque[j+22]=effort[j];}
     recenter_seq=request;std::cout<<"STATIC_EFFORT_TRANSFER request="<<request<<std::endl;
    }
   }
  }
  if(!moving&&!fault&&!releasing&&access((dir+"release_request").c_str(),F_OK)==0){
   releasing=true;release_start=Clock::now();std::cout<<"BEGIN_GRADUAL_RELEASE"<<std::endl;
  }
  if(cycle%50==0&&!fault&&!moving&&!releasing&&Clock::now()-started>std::chrono::seconds(3)){
   std::ifstream f(dir+"arm_target.txt");long newseq=0;float newduration=0;std::array<float,7> q{};
   if(f>>newseq>>newduration){bool parsed=true;for(auto& v:q)if(!(f>>v))parsed=false;
    if(parsed&&newseq>seq){
     // Task route stays inside these bounds, which retain margins to the
     // factory URDF limits (pitch -3.0892, roll -2.2515, yaw +/-2.618,
     // wrist roll +/-1.97222). Wider yaw is needed for the checked mat pose.
     const std::array<float,7> low={-2.31f,-1.91f,-2.40f,-.9f,-1.89f,-1.5f,-1.5f};
     const std::array<float,7> high={.7f,.8f,2.40f,1.8f,1.89f,1.5f,1.5f};
     bool valid=std::isfinite(newduration)&&newduration>=3&&newduration<=30;
     for(int j=0;j<7;++j){float delta=std::abs(q[j]-desired[j+22]);
      if(!std::isfinite(q[j])||q[j]<low[j]||q[j]>high[j]||delta>.25f||delta*1.5f/newduration>.08f)valid=false;
     }
     seq=newseq;
     if(valid){accepted_seq=newseq;for(int j=0;j<7;++j){from[j]=desired[j+22];observed_from[j]=s.motor_state()[j+22].q();}target=q;duration=newduration;motion_start=Clock::now();moving=true;std::cout<<"ACCEPTED seq="<<seq<<" duration="<<duration<<std::endl;}
     else std::cerr<<"REJECTED seq="<<seq<<" target_or_speed_limit"<<std::endl;
    }
   }
  }
  bool settling=false;
  if(moving){float elapsed=std::chrono::duration<float>(Clock::now()-motion_start).count();float phase=elapsed/duration;
   phase=std::clamp(phase,0.f,1.f);float blend=phase*phase*(3-2*phase);
   for(int j=0;j<7;++j)desired[j+22]=from[j]+(target[j]-from[j])*blend;
   settling=phase>=1;
   if(elapsed>=duration+2.f){moving=false;float max_error=0;
    for(int j=0;j<7;++j)max_error=std::max(max_error,std::abs(s.motor_state()[j+22].q()-target[j]));
    float requested_norm2=0,measured_norm2=0;
    for(int j=0;j<7;++j){float requested=target[j]-from[j];float measured=s.motor_state()[j+22].q()-observed_from[j];
     requested_norm2+=requested*requested;measured_norm2+=measured*measured;
    }
    // Verify whole-arm response; redundant wrist corrections below static
    // friction must not be mistaken for failure of the entire Cartesian step.
    // Every joint still has an independent maximum tracking-error check.
    bool response=requested_norm2<.0001f||measured_norm2>=.09f*requested_norm2;
    // Loaded-arm static error is checked separately from measured movement.
    // The independent .065-rad runtime tracking guard remains unchanged.
    if(max_error>.04f||!response){fault=true;reason=response?"TARGET_NOT_REACHED":"NO_MEASURED_RESPONSE";for(int i=22;i<29;++i)desired[i]=s.motor_state()[i].q();}
    std::cout<<(fault?"NOT_REACHED seq=":"VERIFIED_TARGET seq=")<<seq<<" max_error="<<max_error<<" fault="<<fault<<std::endl;
   }
  }
  if(false){ // Integral adaptation is disabled in this task.
   for(int i:{22,25}){
    float error=desired[i]-s.motor_state()[i].q();
    float step=std::clamp(12.f*error*.002f,-.0006f,.0006f);
    float limit=i==22?1.5f:1.f;
    hold_correction[i]=std::clamp(hold_correction[i]+step,-limit,limit);
   }
  }
  float unwind_phase=std::clamp(std::chrono::duration<float>(Clock::now()-started).count()/6.f,0.f,1.f);
  // Gradually stiffen only the loaded shoulder; preserve all position targets.
  // The 8-second ramp and unchanged tracking/speed guards bound the response.
  float gain_phase=std::clamp(std::chrono::duration<float>(Clock::now()-started).count()/8.f,0.f,1.f);
  gain_phase=gain_phase*gain_phase*(3-2*gain_phase);
  for(int i=0;i<29;++i){auto& m=command.motor_cmd()[i];m.mode()=1;m.q()=desired[i];m.dq()=0;m.kp()=i==22?shoulder_kp_start+(100-shoulder_kp_start)*gain_phase:kp[i];m.kd()=i==22?shoulder_kd_start+(2.5f-shoulder_kd_start)*gain_phase:kd[i];m.tau()=torque[i]*((i==15||i==18||i==20)?std::cos(left_rest_angle):1.f)+hold_correction[i]+unwind[i]*unwind_phase;}
  float release_elapsed=releasing?std::chrono::duration<float>(Clock::now()-release_start).count():0.f;
  if(releasing){
   float phase=std::clamp(release_elapsed/5.f,0.f,1.f);float scale=1-phase*phase*(3-2*phase);
   for(int i=0;i<29;++i){auto& m=command.motor_cmd()[i];m.kp()*=scale;m.kd()*=scale;m.tau()*=scale;if(phase>=1)m.mode()=0;}
  }
  command.crc()=crc32_core((uint32_t*)&command,(sizeof(command)>>2)-1);
  {std::lock_guard<std::mutex> g(mutex);own_crcs.push_back(command.crc());if(own_crcs.size()>2048)own_crcs.pop_front();}
  if(!pub.Write(command)){std::cerr<<"TRANSMISSION_FAILED"<<std::endl;break;}
  if(cycle%100==0){
   std::ofstream f(dir+"guard_status.tmp");
   f<<"{\"seq\":"<<seq<<",\"accepted_seq\":"<<accepted_seq<<",\"recenter_seq\":"<<recenter_seq<<",\"releasing\":"<<(releasing?"true":"false")<<",\"moving\":"<<(moving?"true":"false")<<",\"fault\":"<<(fault?"true":"false")<<",\"reason\":\""<<reason<<"\",\"age_s\":"<<std::chrono::duration<float>(Clock::now()-started).count()<<",\"q\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].q();}f<<"],\"desired\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<desired[i];}f<<"],\"initial\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<initial[i];}f<<"],\"hold_correction\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<hold_correction[i];}f<<"],\"dq\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].dq();}f<<"],\"feedforward\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<torque[i];}f<<"],\"temperatures\":[";
   for(int i=0;i<29;++i){if(i)f<<",";f<<"["<<s.motor_state()[i].temperature()[0]<<","<<s.motor_state()[i].temperature()[1]<<"]";}f<<"]}";f.close();rename((dir+"guard_status.tmp").c_str(),(dir+"guard_status.json").c_str());
  }
  if(releasing&&release_elapsed>5.25f){std::cout<<"RELEASE_COMPLETE_ALL_MODES_ZERO"<<std::endl;break;}
  ++cycle;next+=std::chrono::milliseconds(2);
  if(Clock::now()-next>std::chrono::milliseconds(20))next=Clock::now();
  std::this_thread::sleep_until(next);
 }
 std::cout<<"GUARD_EXITED"<<std::endl;return 0;
}


