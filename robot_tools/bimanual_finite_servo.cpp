// Empty-hand CAD validation only. One publisher, finite trajectory and exit.
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
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using Clock=std::chrono::steady_clock;using State=unitree_hg::msg::dds_::LowState_;using Cmd=unitree_hg::msg::dds_::LowCmd_;
std::atomic<bool> stopped{false};void stop(int){stopped=true;}
constexpr std::array<float,29> KP={60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,80,60,60,60,40,40,40,80,60,60,60,40,40,40};
constexpr std::array<float,29> KD={1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1.5,1.5,1.5,1.5,1,1,1,1.5,1.5,1.5,1.5,1,1,1};
float smooth(float t){t=std::clamp(t,0.f,1.f);return t*t*(3-2*t);}
struct Sample{float t=0;bimanual::Arm q{},g{};};
struct CloseReaders{std::function<void()> f;~CloseReaders(){f();}};
int main(int argc,char**argv){
 if(argc!=3)return 1;std::string dir=std::string(argv[1])+"/";bool execute=std::string(argv[2])=="--execute-empty-profile";
 if(!execute&&std::string(argv[2])!="--dry-run")return 1;
 std::ifstream f(dir+"profile.txt");std::string id;int count=0;bimanual::Body anchor{};
 if(!(f>>id>>count)||id!="EMPTY_CAD_VALIDATION_V1"||count<2||count>6000)return 2;
 for(float &x:anchor)if(!(f>>x)||!std::isfinite(x))return 2;
 std::vector<Sample> plan(count);
 for(int k=0;k<count;++k){auto &a=plan[k];if(!(f>>a.t)||!std::isfinite(a.t)||a.t<0||a.t>180)return 2;
  for(int j=0;j<14;++j)if(!(f>>a.q[j])||!std::isfinite(a.q[j])||a.q[j]<bimanual::lo[j]||a.q[j]>bimanual::hi[j])return 2;
  for(float &x:a.g)if(!(f>>x)||!std::isfinite(x)||std::abs(x)>5)return 2;
  if(k){float dt=a.t-plan[k-1].t;if(dt<.009f||dt>.101f)return 2;for(int j=0;j<14;++j)if(std::abs(a.q[j]-plan[k-1].q[j])/dt>.081f||std::abs(a.g[j]-plan[k-1].g[j])/dt>4.f)return 2;}
 }
 std::string extra;if(f>>extra)return 2;if(plan.front().t!=0)return 2;
 for(int j=0;j<14;++j)if(std::abs(plan.front().q[j]-anchor[j+15])>.001f||std::abs(plan.back().q[j]-anchor[j+15])>.001f||std::abs(plan.front().g[j])>.001f||std::abs(plan.back().g[j])>.001f)return 2;
 int lock=open("/home/unitree/codex_g1_tasks/body_microtest.lock",O_CREAT|O_RDWR,0600);if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 2;
 signal(SIGINT,stop);signal(SIGTERM,stop);unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 std::mutex mu;State latest;Clock::time_point received{};int valid=0,invalid=0,other=0;std::deque<uint32_t> own;
 auto sub=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<State>("rt/lowstate",[&](const void*p){const auto&s=*(const State*)p;std::lock_guard<std::mutex>g(mu);if(s.crc()!=crc32_core((uint32_t*)&s,(sizeof(s)>>2)-1)){++invalid;return;}latest=s;received=Clock::now();++valid;},0);
 auto cmds=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Cmd>("rt/lowcmd",[&](const void*p){const auto&c=*(const Cmd*)p;std::lock_guard<std::mutex>g(mu);if(std::find(own.begin(),own.end(),c.crc())==own.end())++other;},0);
 CloseReaders cleanup{[&](){auto c=cmds->GetReader()->GetNative();auto s=sub->GetReader()->GetNative();c.close();s.close();cmds.reset();sub.reset();std::cout<<"DDS_READERS_CLOSED_ON_MAIN"<<std::endl;}};
 std::this_thread::sleep_for(std::chrono::seconds(3));State initial;{std::lock_guard<std::mutex>g(mu);initial=latest;if(valid<500||invalid||other||Clock::now()-received>std::chrono::milliseconds(100))return 2;}
 if(initial.mode_machine()!=5||initial.mode_pr()!=0)return 2;
 bimanual::Body desired{},ff{},baseff{},measured{};
 for(int i=0;i<29;++i){auto m=initial.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.tau_est())||std::abs(m.q()-anchor[i])>.025f||std::abs(m.dq())>.1f||std::abs(m.tau_est())>5||m.motorstate()||m.temperature()[0]>=65||m.temperature()[1]>=85)return 2;desired[i]=measured[i]=m.q();baseff[i]=ff[i]=m.tau_est();}
 std::cout<<"PROFILE_VALIDATED duration="<<plan.back().t<<" execute="<<execute<<std::endl;if(!execute)return 0;
 unitree::robot::ChannelPublisher<Cmd> publisher("rt/lowcmd");publisher.InitChannel();Cmd cmd;cmd.mode_machine()=5;cmd.mode_pr()=0;
 std::ofstream trace(dir+"feedback.csv");trace<<std::setprecision(10)<<"steady_s,t,path_t,reversing,releasing,j,q,dq,tau,qdes,tauff,scale,temp_case,temp_winding,qsent,estimated_tau_cmd\n";
 auto start=Clock::now(),next=start,previous=start;float phase=0,release_t=0;bool reverse=false,releasing=false,fault=false;int violations=0,cycle=0;std::string reason;
 for(;;){
  auto now=Clock::now();float t=std::chrono::duration<float>(now-start).count();float dt=std::clamp(std::chrono::duration<float>(now-previous).count(),0.f,.02f);previous=now;
  State s;Clock::time_point when;int competition;{std::lock_guard<std::mutex>g(mu);s=latest;when=received;competition=other;}
  if(now-when>std::chrono::milliseconds(150)||s.mode_machine()!=5||s.mode_pr()!=0||competition){std::cerr<<"FATAL_FEEDBACK_MODE_OR_COMPETITION"<<std::endl;publisher.CloseChannel();return 3;}
  std::string bad;
  for(int i=0;i<29;++i){auto m=s.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est())||m.motorstate()){std::cerr<<"FATAL_MOTOR_FEEDBACK j="<<i<<std::endl;publisher.CloseChannel();return 3;}
   if(std::abs(m.dq())>.5f)bad="speed";else if(m.temperature()[0]>=72||m.temperature()[1]>=95)bad="thermal_reserve";else if(i>=15&&std::abs(m.tau_est())>7.f)bad="empty_arm_torque";else if(!releasing&&std::abs(m.q()-desired[i])>(i<15?.035f:.065f))bad="tracking";
  }
  violations=bad.empty()?0:violations+1;
  if((violations>25||stopped||t>plan.back().t+30)&&!reverse&&!releasing){fault=true;reason=stopped?"signal":(bad.empty()?"timeout":bad);reverse=true;std::cerr<<"PROFILE_REVERSE reason="<<reason<<" phase="<<phase<<std::endl;}
  if(t>=8&&!releasing){phase=std::clamp(phase+(reverse?-dt:dt),0.f,plan.back().t);if((reverse&&phase<=0)||phase>=plan.back().t){releasing=true;release_t=t;std::cout<<"PROFILE_RETURNED max_goal_error=";float e=0;for(int i=15;i<29;++i)e=std::max(e,std::abs(s.motor_state()[i].q()-measured[i]));std::cout<<e<<std::endl;if(e>.045f){fault=true;reason="return_error";}}}
  auto it=std::lower_bound(plan.begin(),plan.end(),phase,[](const Sample&a,float b){return a.t<b;});size_t k=it==plan.end()?plan.size()-1:std::distance(plan.begin(),it);size_t h=k?k-1:0;float a=k==h?0:(phase-plan[h].t)/(plan[k].t-plan[h].t);
  for(int j=0;j<14;++j){int i=j+15;desired[i]=measured[i]+plan[h].q[j]+a*(plan[k].q[j]-plan[h].q[j])-anchor[i];ff[i]=baseff[i]+plan[h].g[j]+a*(plan[k].g[j]-plan[h].g[j]);}
  float scale=releasing?1-smooth((t-release_t)/5):smooth(t/4);
  std::array<float,29> estimated_command{};
  for(int i=0;i<29;++i){auto&m=cmd.motor_cmd()[i];float qsend=desired[i],damping=KD[i];
   if(i>=15){int j=i-15,k=j%7;damping*=3.f;float q=s.motor_state()[i].q(),dq=s.motor_state()[i].dq();float pterm=KP[i]*std::clamp(3.f*(desired[i]-q),-.065f,.065f);float limit=k<2?6.5f:(k<4?4.f:(k==6?1.f:1.5f));float total=std::clamp(ff[i]+pterm-damping*dq,-limit,limit);qsend=q+(total-ff[i]+damping*dq)/KP[i];qsend=std::clamp(qsend,bimanual::lo[j]+.01f,bimanual::hi[j]-.01f);}
   m.mode()=releasing&&scale<=0?0:1;m.q()=qsend;m.dq()=0;m.kp()=KP[i]*scale;m.kd()=damping*scale;m.tau()=ff[i]*scale;estimated_command[i]=(ff[i]+KP[i]*(qsend-s.motor_state()[i].q())-damping*s.motor_state()[i].dq())*scale;
  }
  cmd.crc()=crc32_core((uint32_t*)&cmd,(sizeof(cmd)>>2)-1);{std::lock_guard<std::mutex>g(mu);own.push_back(cmd.crc());if(own.size()>2048)own.pop_front();}if(!publisher.Write(cmd))return 3;
  if(cycle%10==0){double steady=std::chrono::duration<double>(now.time_since_epoch()).count();for(int i=0;i<29;++i){auto m=s.motor_state()[i];trace<<steady<<","<<t<<","<<phase<<","<<reverse<<","<<releasing<<","<<i<<","<<m.q()<<","<<m.dq()<<","<<m.tau_est()<<","<<desired[i]<<","<<ff[i]<<","<<scale<<","<<int(m.temperature()[0])<<","<<int(m.temperature()[1])<<","<<cmd.motor_cmd()[i].q()<<","<<estimated_command[i]<<"\n";}trace.flush();}
  if(cycle%50==0){std::ofstream st(dir+"status.tmp");st<<"{\"elapsed\":"<<t<<",\"phase\":"<<phase<<",\"reverse\":"<<reverse<<",\"release\":"<<releasing<<",\"fault\":"<<fault<<",\"reason\":\""<<reason<<"\"}";st.close();rename((dir+"status.tmp").c_str(),(dir+"status.json").c_str());}
  if(releasing&&t-release_t>5.25f){publisher.CloseChannel();std::cout<<"FINITE_EMPTY_RELEASED fault="<<fault<<" reason="<<reason<<std::endl;return fault?4:0;}
  ++cycle;next+=std::chrono::milliseconds(2);if(Clock::now()-next>std::chrono::milliseconds(20))next=Clock::now();std::this_thread::sleep_until(next);
 }
}
