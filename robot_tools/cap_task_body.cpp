// EXPERIMENTAL: not validated for object contact. Physical execution disabled below.
// Supported-table cap task: one 29-joint publisher; only arm targets change.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include "bimanual_motion.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using Clock=std::chrono::steady_clock;using State=unitree_hg::msg::dds_::LowState_;using Cmd=unitree_hg::msg::dds_::LowCmd_;using Bms=unitree_hg::msg::dds_::BmsState_;
std::atomic<bool> stop_requested{false};void stop_task(int){stop_requested=true;}
constexpr std::array<float,29> KP={60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,80,60,60,60,40,40,40,80,60,60,60,40,40,40};
constexpr std::array<float,29> KD={1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1.5,1.5,1.5,1.5,1,1,1,1.5,1.5,1.5,1.5,1,1,1};
float smooth(float v){v=std::clamp(v,0.f,1.f);return v*v*(3-2*v);}double now_s(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
struct Sample{float t=0;bimanual::Arm q{},g{};};using Path=std::vector<Sample>;
struct Bundle{long seq=0;std::string verb;Path path,start_return,end_return;};
bool load_path(const std::string&file,const std::string&run,long seq,Path&out){
 std::ifstream f(file);std::string format,id,extra;long nseq;int count;if(!(f>>format>>id>>nseq>>count)||format!="CAPSEG1"||id!=run||seq!=nseq||count<2||count>5000)return false;Path p(count);
 for(int k=0;k<count;++k){auto&a=p[k];if(!(f>>a.t)||!std::isfinite(a.t)||a.t<0||a.t>180)return false;
  for(int j=0;j<14;++j)if(!(f>>a.q[j])||!std::isfinite(a.q[j])||a.q[j]<bimanual::lo[j]+.005f||a.q[j]>bimanual::hi[j]-.005f)return false;
  for(float&v:a.g)if(!(f>>v)||!std::isfinite(v)||std::abs(v)>5)return false;
  if(k){float dt=a.t-p[k-1].t;if(dt<.009f||dt>.101f)return false;for(int j=0;j<14;++j)if(std::abs(a.q[j]-p[k-1].q[j])/dt>.081f||std::abs(a.g[j]-p[k-1].g[j])/dt>5.f)return false;}}
 if(p.front().t!=0||p.back().t<1||f>>extra)return false;out=std::move(p);return true;
}
Sample sample(const Path&p,float t){auto it=std::lower_bound(p.begin(),p.end(),t,[](const Sample&a,float b){return a.t<b;});size_t k=it==p.end()?p.size()-1:std::distance(p.begin(),it),h=k?k-1:0;float v=h==k?0:(t-p[h].t)/(p[k].t-p[h].t);Sample r;r.t=t;for(int j=0;j<14;++j){r.q[j]=p[h].q[j]+v*(p[k].q[j]-p[h].q[j]);r.g[j]=p[h].g[j]+v*(p[k].g[j]-p[h].g[j]);}return r;}
bool matches(const Sample&a,const Sample&b,float qtol=.003f,float gtol=.06f){for(int j=0;j<14;++j)if(std::abs(a.q[j]-b.q[j])>qtol||std::abs(a.g[j]-b.g[j])>gtol)return false;return true;}
struct Cleanup{std::function<void()>fn;~Cleanup(){fn();}};
int main(int argc,char**argv){
 if(argc!=4)return 1;const std::string dir=std::string(argv[1])+"/",run=argv[2];bool execute=std::string(argv[3])=="--execute";if(!execute&&std::string(argv[3])!="--dry-run")return 1;
 if(execute){std::cerr<<"EXPERIMENTAL_CONTACT_CONTROLLER_NOT_VALIDATED: no publisher started"<<std::endl;return 2;}
 if(dir.find("/home/unitree/codex_g1_tasks/cap_open_")!=0||run.size()>80)return 1;
 int lock=open("/home/unitree/codex_g1_tasks/body_microtest.lock",O_CREAT|O_RDWR,0600);if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 2;signal(SIGINT,stop_task);signal(SIGTERM,stop_task);
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");std::mutex mu;State latest;Clock::time_point received{},battery_received{};int valid=0,invalid=0,other=0,bcount=0,soc=-1;std::deque<uint32_t> own;
 auto states=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<State>("rt/lowstate",[&](const void*p){const auto&s=*(const State*)p;std::lock_guard<std::mutex>g(mu);if(s.crc()!=crc32_core((uint32_t*)&s,(sizeof(s)>>2)-1)){++invalid;return;}latest=s;received=Clock::now();++valid;},0);
 auto commands=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Cmd>("rt/lowcmd",[&](const void*p){const auto&c=*(const Cmd*)p;std::lock_guard<std::mutex>g(mu);if(std::find(own.begin(),own.end(),c.crc())==own.end())++other;},0);
 auto battery=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Bms>("rt/lf/bmsstate",[&](const void*p){const auto&b=*(const Bms*)p;std::lock_guard<std::mutex>g(mu);soc=b.soc();battery_received=Clock::now();++bcount;},0);
 Cleanup cleanup{[&](){auto a=commands->GetReader()->GetNative();auto b=states->GetReader()->GetNative();auto c=battery->GetReader()->GetNative();a.close();b.close();c.close();commands.reset();states.reset();battery.reset();std::cout<<"DDS_READERS_CLOSED_ON_MAIN"<<std::endl;}};
 std::this_thread::sleep_for(std::chrono::seconds(3));State initial;{std::lock_guard<std::mutex>g(mu);initial=latest;if(valid<500||invalid||other||bcount<2||soc<28||soc>100||Clock::now()-received>std::chrono::milliseconds(100))return 2;}
 if(initial.mode_machine()!=5||initial.mode_pr()!=0)return 2;bimanual::Body base{},baseff{},desired{},ff{};Sample current,home;
 for(int i=0;i<29;++i){auto m=initial.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est())||m.motorstate()||std::abs(m.dq())>.1f||std::abs(m.tau_est())>5||m.temperature()[0]>=65||m.temperature()[1]>=85)return 2;base[i]=desired[i]=m.q();baseff[i]=ff[i]=m.tau_est();if(i>=15)home.q[i-15]=current.q[i-15]=m.q();}
 std::cout<<"CAP_BODY_PREFLIGHT execute="<<execute<<" soc="<<soc<<std::endl;if(!execute)return 0;
 std::mutex cmd_mu;std::shared_ptr<Bundle> pending;std::string loader_error;std::atomic<bool> quitting{false};long loaded_seq=0;
 std::thread loader([&](){while(!quitting){std::ifstream f(dir+"request.txt");std::string id,verb;long seq=0;if(f>>id>>seq>>verb&&id==run&&seq>loaded_seq&&seq<10000){auto b=std::make_shared<Bundle>();b->seq=seq;b->verb=verb;bool ok=true;if(verb=="MOVE"||verb=="RECOVER"||verb=="SAFE_CAP")ok=load_path(dir+"trajectory_"+std::to_string(seq)+".txt",run,seq,b->path)&&load_path(dir+"return_start_"+std::to_string(seq)+".txt",run,seq,b->start_return)&&load_path(dir+"return_end_"+std::to_string(seq)+".txt",run,seq,b->end_return);else ok=verb=="FINISH"||verb=="STOP";{std::lock_guard<std::mutex>g(cmd_mu);if(ok)pending=b;else loader_error="invalid_segment_file";}loaded_seq=seq;}std::this_thread::sleep_for(std::chrono::milliseconds(60));}});
 Cleanup join_loader{[&](){quitting=true;if(loader.joinable())loader.join();}};
 unitree::robot::ChannelPublisher<Cmd> publisher("rt/lowcmd");publisher.InitChannel();Cmd cmd;cmd.mode_machine()=5;cmd.mode_pr()=0;
 std::ofstream trace(dir+"body_feedback.csv");trace<<std::setprecision(10)<<"steady_s,t,seq,moving,fault,j,q,dq,tau,qgoal,tauff,scale,temp_case,temp_winding,qsent,estimated_tau_cmd\n";
 auto started=Clock::now(),previous=started,next=started;long seq=0,done_seq=0,rejected_seq=0;bool moving=false,fault=false,recovering=false,releasing=false,emergency_reverse=false,emergency_return=false;std::string reason;float phase=0,release_time=0,fault_time=0;int failures=0,cycle=0;std::shared_ptr<Bundle> active;Path route;
 auto safe_grippers=[&](){bool all=true;for(const auto&side:{"left","right"}){std::ifstream f(dir+"gripper_"+side+"/safe.txt");std::string id;double stamp;float q;int safe,error;if(!(f>>id>>stamp>>q>>safe>>error)||id!=run||now_s()-stamp>.8||!safe||error)all=false;}return all;};
 for(;;){
  auto now=Clock::now();float t=std::chrono::duration<float>(now-started).count(),dt=std::clamp(std::chrono::duration<float>(now-previous).count(),0.f,.02f);previous=now;State s;Clock::time_point when,bwhen;int competition,level;{std::lock_guard<std::mutex>g(mu);s=latest;when=received;bwhen=battery_received;competition=other;level=soc;}
  if(now-when>std::chrono::milliseconds(150)||s.mode_machine()!=5||s.mode_pr()!=0||competition){std::ofstream(dir+"abort_requested").put('1');std::cerr<<"FATAL_FEEDBACK_MODE_OR_COMPETITION"<<std::endl;publisher.CloseChannel();return 3;}
  std::string bad;
  for(int i=0;i<29;++i){auto m=s.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est())||m.motorstate()){std::ofstream(dir+"abort_requested").put('1');std::cerr<<"FATAL_MOTOR j="<<i<<std::endl;publisher.CloseChannel();return 3;}if(std::abs(m.dq())>.5f)bad="speed";else if(m.temperature()[0]>=(recovering?80:72)||m.temperature()[1]>=(recovering?110:95))bad="thermal_reserve";else if(i>=15&&std::abs(m.tau_est())>7.f)bad="arm_torque";else if(!releasing&&std::abs(m.q()-desired[i])>(i<15?.035f:.065f))bad="tracking";}
  if(level<=23||now-bwhen>std::chrono::seconds(5))bad="battery_reserve";
  double heartbeat=0;{std::ifstream f(dir+"guardian_heartbeat.txt");f>>heartbeat;}if(t>6&&now_s()-heartbeat>2)bad="guardian_stale";
  failures=bad.empty()?0:failures+1;
  if((failures>25||stop_requested||t>1800)&&!fault&&!releasing){fault=true;reason=stop_requested?"signal":(bad.empty()?"task_timeout":bad);fault_time=t;moving=false;std::ofstream(dir+"abort_requested").put('1');std::cerr<<"CAP_STOP "<<reason<<std::endl;}
  std::shared_ptr<Bundle> request;{std::lock_guard<std::mutex>g(cmd_mu);request=pending;pending.reset();}
  if(request){
   if(request->verb=="STOP"){if(!fault){fault=true;reason="supervisor_stop";fault_time=t;moving=false;std::ofstream(dir+"abort_requested").put('1');}}
   else if(request->verb=="FINISH"){if(!moving&&matches(current,home,.012f,.15f)&&safe_grippers()){releasing=true;release_time=t;seq=request->seq;}else rejected_seq=request->seq;}
   else {bool allowed=!moving&&!releasing&&(!fault||((request->verb=="RECOVER"&&safe_grippers())||request->verb=="SAFE_CAP"));bool continuity=matches(request->path.front(),current)&&matches(request->start_return.front(),request->path.front())&&matches(request->end_return.front(),request->path.back())&&matches(request->start_return.back(),home,.008f,.05f)&&matches(request->end_return.back(),home,.008f,.05f);
    if(allowed&&continuity&&t>=5){active=request;route=request->path;phase=0;moving=true;recovering=fault;seq=request->seq;std::cout<<"ACCEPTED_SEGMENT "<<seq<<std::endl;}else rejected_seq=request->seq;}
  }
  if(fault&&!moving&&!releasing&&!emergency_return&&t-fault_time>2&&safe_grippers()&&now_s()-heartbeat>2){recovering=true;emergency_return=true;if(active&&phase>0&&phase<active->path.back().t){route=active->path;moving=true;emergency_reverse=true;}else if(active){route=active->end_return;phase=0;moving=true;}else {releasing=true;release_time=t;}}
  if(moving){phase=std::clamp(phase+(emergency_reverse?-dt:dt),0.f,route.back().t);current=sample(route,phase);if((emergency_reverse&&phase<=0)||(!emergency_reverse&&phase>=route.back().t)){moving=false;if(emergency_reverse){emergency_reverse=false;route=active->start_return;phase=0;moving=true;}else if(emergency_return){releasing=true;release_time=t;}else done_seq=seq;}}
  for(int j=0;j<14;++j){desired[j+15]=current.q[j];ff[j+15]=baseff[j+15]+current.g[j];}
  float scale=releasing?1-smooth((t-release_time)/5):smooth(t/4);std::array<float,29> estimated{};
  for(int i=0;i<29;++i){auto&m=cmd.motor_cmd()[i];float sent=desired[i],damping=KD[i];if(i>=15){int j=i-15,k=j%7;damping*=3;float q=s.motor_state()[i].q(),dq=s.motor_state()[i].dq(),limit=k<2?6.5f:(k<4?4.f:(k==6?1.f:1.5f));float total=std::clamp(ff[i]+KP[i]*std::clamp(3.f*(desired[i]-q),-.065f,.065f)-damping*dq,-limit,limit);sent=std::clamp(q+(total-ff[i]+damping*dq)/KP[i],bimanual::lo[j]+.01f,bimanual::hi[j]-.01f);}m.mode()=releasing&&scale<=0?0:1;m.q()=sent;m.dq()=0;m.kp()=KP[i]*scale;m.kd()=damping*scale;m.tau()=ff[i]*scale;estimated[i]=(ff[i]+KP[i]*(sent-s.motor_state()[i].q())-damping*s.motor_state()[i].dq())*scale;}
  cmd.crc()=crc32_core((uint32_t*)&cmd,(sizeof(cmd)>>2)-1);{std::lock_guard<std::mutex>g(mu);own.push_back(cmd.crc());if(own.size()>2048)own.pop_front();}if(!publisher.Write(cmd))return 3;
  if(cycle%10==0){double stamp=now_s();for(int i=0;i<29;++i){auto m=s.motor_state()[i];trace<<stamp<<","<<t<<","<<seq<<","<<moving<<","<<fault<<","<<i<<","<<m.q()<<","<<m.dq()<<","<<m.tau_est()<<","<<desired[i]<<","<<ff[i]<<","<<scale<<","<<int(m.temperature()[0])<<","<<int(m.temperature()[1])<<","<<cmd.motor_cmd()[i].q()<<","<<estimated[i]<<"\n";}trace.flush();}
  if(cycle%50==0){std::ofstream f(dir+"body_status.tmp");f<<std::setprecision(10)<<"{\"steady\":"<<now_s()<<",\"elapsed\":"<<t<<",\"soc\":"<<level<<",\"seq\":"<<seq<<",\"done_seq\":"<<done_seq<<",\"rejected_seq\":"<<rejected_seq<<",\"moving\":"<<moving<<",\"phase\":"<<phase<<",\"fault\":"<<fault<<",\"reason\":\""<<reason<<"\",\"recovering\":"<<recovering<<",\"releasing\":"<<releasing<<",\"q\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].q();}f<<"],\"dq\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].dq();}f<<"],\"qgoal\":[";for(int j=0;j<14;++j){if(j)f<<",";f<<current.q[j];}f<<"],\"g\":[";for(int j=0;j<14;++j){if(j)f<<",";f<<current.g[j];}f<<"],\"temperature\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<"["<<int(s.motor_state()[i].temperature()[0])<<","<<int(s.motor_state()[i].temperature()[1])<<"]";}f<<"]}";f.close();rename((dir+"body_status.tmp").c_str(),(dir+"body_status.json").c_str());}
  if(releasing&&t-release_time>5.25f){publisher.CloseChannel();std::ofstream(dir+"body_exited").put('1');std::cout<<"CAP_BODY_RELEASED fault="<<fault<<std::endl;return fault?4:0;}
  ++cycle;next+=std::chrono::milliseconds(2);if(Clock::now()-next>std::chrono::milliseconds(20))next=Clock::now();std::this_thread::sleep_until(next);
 }
}
