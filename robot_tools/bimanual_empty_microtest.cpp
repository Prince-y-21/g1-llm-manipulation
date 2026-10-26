// Scoped suspended EMPTY-HAND test only: shoulders roll outward 0.03 rad,
// return to the measured low-load baseline, then release. No object motion.
// One publisher controls all29. No external target queue is accepted.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include "bimanual_motion.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using Clock=std::chrono::steady_clock;using State=unitree_hg::msg::dds_::LowState_;using Cmd=unitree_hg::msg::dds_::LowCmd_;
std::atomic<bool> stopping{false};void stop_test(int){stopping=true;}
constexpr std::array<float,29> KP={60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,80,60,60,60,40,40,40,80,60,60,60,40,40,40};
constexpr std::array<float,29> KD={1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1.5,1.5,1.5,1.5,1,1,1,1.5,1.5,1.5,1.5,1,1,1};
float smooth(float x){x=std::clamp(x,0.f,1.f);return x*x*(3-2*x);}
struct MainThreadReaderClose {std::function<void()> close;~MainThreadReaderClose(){close();}};
int main(int argc,char**argv){
 if(argc!=3)return 1;const std::string dir=std::string(argv[1])+"/";bool private_test=std::string(argv[2])=="--private-dds-close-test";bool execute=private_test||std::string(argv[2])=="--empty-outward-suspended";
 if(!execute&&std::string(argv[2])!="--dry-run")return 1;
 int lock=open("/home/unitree/codex_g1_tasks/body_microtest.lock",O_CREAT|O_RDWR,0600);if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 2;
 signal(SIGINT,stop_test);signal(SIGTERM,stop_test);unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 std::mutex mutex;State latest;Clock::time_point received{};int valid=0,invalid=0,other=0;std::deque<uint32_t> own;
 auto sub=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<State>("rt/lowstate",[&](const void*p){const auto& s=*(const State*)p;std::lock_guard<std::mutex> g(mutex);if(s.crc()!=crc32_core((uint32_t*)&s,(sizeof(s)>>2)-1)){++invalid;return;}latest=s;received=Clock::now();++valid;},1);
 const std::string topic=private_test?"rt/codex_private_dds_close_test":"rt/lowcmd";
 auto commands=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Cmd>(topic,[&](const void*p){const auto& c=*(const Cmd*)p;std::lock_guard<std::mutex> g(mutex);if(std::find(own.begin(),own.end(),c.crc())==own.end())++other;},1);
 // Retain native handles and close explicitly on main while references live.
 // Avoid last-reference reader destruction on a DDS callback thread.
 MainThreadReaderClose cleanup{[&](){auto cr=commands->GetReader()->GetNative();auto sr=sub->GetReader()->GetNative();cr.listener(nullptr,dds::core::status::StatusMask::none());sr.listener(nullptr,dds::core::status::StatusMask::none());cr.close();sr.close();commands.reset();sub.reset();std::cout<<"DDS_READERS_CLOSED_ON_MAIN"<<std::endl;}};
 std::this_thread::sleep_for(std::chrono::seconds(3));State start;{std::lock_guard<std::mutex> g(mutex);start=latest;if(valid<500||invalid||other||Clock::now()-received>std::chrono::milliseconds(100))return 2;}
 if(start.mode_machine()!=5||start.mode_pr()!=0)return 2;
 bimanual::Body initial{},desired{},ff{};for(int i=0;i<29;++i){auto m=start.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est())||std::abs(m.dq())>.1f||std::abs(m.q())>(i<15?.3f:2.6f)||m.motorstate()||m.temperature()[0]>=70||m.temperature()[1]>=90||std::abs(m.tau_est())>5)return 2;initial[i]=desired[i]=m.q();ff[i]=m.tau_est();}
 std::cout<<"MICROTEST_PRECHECK_OK dry="<<!execute<<std::endl;if(!execute)return 0;
 unitree::robot::ChannelPublisher<Cmd> publisher(topic);publisher.InitChannel();Cmd cmd;cmd.mode_machine()=5;cmd.mode_pr()=0;
 if(private_test){for(int k=0;k<1000;++k){cmd.crc()=crc32_core((uint32_t*)&cmd,(sizeof(cmd)>>2)-1);{std::lock_guard<std::mutex> g(mutex);own.push_back(cmd.crc());}if(!publisher.Write(cmd))return 3;std::this_thread::sleep_for(std::chrono::milliseconds(1));}publisher.CloseChannel();std::cout<<"PRIVATE_DDS_TEST_ONLY no_rt_lowcmd_publisher"<<std::endl;return 0;}
 bimanual::Arm baseline{},outward{};bool baseline_set=false,out_checked=false,return_checked=false,releasing=false,fault=false;int failures=0;std::string reason="";
 float release_at=0,verified_left=0,verified_right=0;auto started=Clock::now(),next=started;int cycle=0;
 for(;;){
  State s;int competition;Clock::time_point when;{std::lock_guard<std::mutex> g(mutex);s=latest;when=received;competition=other;}
  float t=std::chrono::duration<float>(Clock::now()-started).count();
  if(Clock::now()-when>std::chrono::milliseconds(150)||s.mode_machine()!=5||s.mode_pr()!=0||competition){std::cerr<<"MICROTEST_STOP stale_mode_or_competition"<<std::endl;return 3;}
  std::string bad="";for(int i=0;i<29;++i){auto m=s.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||m.motorstate())bad="invalid_motor_state";else if(std::abs(m.dq())>.5f)bad="speed";else if(m.temperature()[0]>=75||m.temperature()[1]>=100)bad="thermal_reserve";else if(std::abs(m.q()-desired[i])>(i<15?.035f:.065f))bad="tracking";}
  failures=bad.empty()?0:failures+1;
  if(stopping||failures>25||t>28){fault=true;reason=stopping?"operator_stop":(t>28?"hard_timeout":bad);}
  if(!releasing&&fault){for(int i=0;i<29;++i){ff[i]+=KP[i]*(desired[i]-s.motor_state()[i].q());desired[i]=s.motor_state()[i].q();}releasing=true;release_at=t;std::cerr<<"MICROTEST_ABORT_RELEASE "<<reason<<std::endl;}
  if(!releasing&&!baseline_set&&t>=8){
   bool stable=true;for(int j=0;j<14;++j){int i=j+15;auto m=s.motor_state()[i];if(std::abs(m.dq())>.05f||std::abs(m.q()-initial[i])>.035f)stable=false;}
   if(!stable){fault=true;reason="initial_hold_not_stable";continue;}
   for(int j=0;j<14;++j){int i=j+15;ff[i]+=KP[i]*(desired[i]-s.motor_state()[i].q());baseline[j]=desired[i]=s.motor_state()[i].q();outward[j]=baseline[j];}
   outward[1]+=.03f;outward[8]-=.03f;baseline_set=true;std::cout<<"BASELINE_CAPTURED"<<std::endl;
  }
  if(!releasing&&baseline_set){
   if(t<11)bimanual::apply(desired,baseline,outward,(t-8)/3);
   else if(t<13)bimanual::apply(desired,baseline,outward,1);
   else if(t<16)bimanual::apply(desired,outward,baseline,(t-13)/3);
   else bimanual::apply(desired,outward,baseline,1);
   if(!out_checked&&t>=12.5f){verified_left=s.motor_state()[16].q()-baseline[1];verified_right=baseline[8]-s.motor_state()[23].q();out_checked=true;std::cout<<"MEASURED_OUTWARD left="<<verified_left<<" right="<<verified_right<<std::endl;if(verified_left<.006f||verified_right<.006f){fault=true;reason="outward_response_failed";}}
   if(!return_checked&&t>=18){float error=0;for(int j=0;j<14;++j)error=std::max(error,std::abs(s.motor_state()[j+15].q()-baseline[j]));return_checked=true;std::cout<<"MEASURED_RETURN error="<<error<<std::endl;if(error>.035f){fault=true;reason="return_failed";}releasing=true;release_at=t;}
  }
  float scale=releasing?1-smooth((t-release_at)/5):smooth(t/4);
  for(int i=0;i<29;++i){auto& m=cmd.motor_cmd()[i];m.mode()=scale<=0&&releasing?0:1;m.q()=desired[i];m.dq()=0;m.kp()=KP[i]*scale;m.kd()=KD[i]*scale;m.tau()=ff[i]*scale;}
  cmd.crc()=crc32_core((uint32_t*)&cmd,(sizeof(cmd)>>2)-1);{std::lock_guard<std::mutex> g(mutex);own.push_back(cmd.crc());if(own.size()>2048)own.pop_front();}
  if(!publisher.Write(cmd)){std::cerr<<"PUBLISH_FAILED"<<std::endl;return 3;}
  if(cycle%50==0){std::ofstream f(dir+"microtest_status.tmp");f<<"{\"elapsed_s\":"<<t<<",\"fault\":"<<(fault?"true":"false")<<",\"reason\":\""<<reason<<"\",\"releasing\":"<<(releasing?"true":"false")<<",\"left_outward\":"<<verified_left<<",\"right_outward\":"<<verified_right<<",\"q\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].q();}f<<"]}";f.close();rename((dir+"microtest_status.tmp").c_str(),(dir+"microtest_status.json").c_str());}
  if(releasing&&t-release_at>5.25f){publisher.CloseChannel();std::cout<<"MICROTEST_RELEASED exited fault="<<fault<<" baseline="<<baseline_set<<" out="<<out_checked<<" return="<<return_checked<<std::endl;return fault?4:0;}
  ++cycle;next+=std::chrono::milliseconds(2);if(Clock::now()-next>std::chrono::milliseconds(20))next=Clock::now();std::this_thread::sleep_until(next);
 }
}
