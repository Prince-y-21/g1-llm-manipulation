// Retry patch: explicit stop-hold and evidenced rest release; one 29-joint publisher.
// Supported-table cap task: one 29-joint publisher; only arm targets change.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include "bimanual_motion.hpp"
#include "retry_gate.hpp"
#include "retry_ack.hpp"
#include "retry_thermal_v4.hpp"
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
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
using Clock=std::chrono::steady_clock;using State=unitree_hg::msg::dds_::LowState_;using Cmd=unitree_hg::msg::dds_::LowCmd_;using Bms=unitree_hg::msg::dds_::BmsState_;
std::atomic<bool> stop_requested{false};void stop_task(int){stop_requested=true;}
constexpr std::array<float,29> KP={60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,80,60,60,60,40,40,40,80,60,60,60,40,40,40};
constexpr std::array<float,29> KD={1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1.5,1.5,1.5,1.5,1,1,1,1.5,1.5,1.5,1.5,1,1,1};
float smooth(float v){v=std::clamp(v,0.f,1.f);return v*v*(3-2*v);}double now_s(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
struct Sample{float t=0;bimanual::Arm q{},g{};};using Path=std::vector<Sample>;
struct Bundle{long seq=0;std::string verb;Path path,start_return,end_return;thermal_v4::Contract thermal;float normal_end=0;};
bool load_path(const std::string&file,const std::string&run,long seq,Path&out){
 std::ifstream f(file);std::string format,id,extra;long nseq;int count;if(!(f>>format>>id>>nseq>>count)||format!="CAPSEG1"||id!=run||seq!=nseq||count<2||count>5000)return false;Path p(count);
 for(int k=0;k<count;++k){auto&a=p[k];if(!(f>>a.t)||!std::isfinite(a.t)||a.t<0||a.t>180)return false;
  for(int j=0;j<14;++j)if(!(f>>a.q[j])||!std::isfinite(a.q[j])||a.q[j]<bimanual::lo[j]+.005f||a.q[j]>bimanual::hi[j]-.005f)return false;
  for(float&v:a.g)if(!(f>>v)||!std::isfinite(v)||std::abs(v)>25)return false;
  if(k){float dt=a.t-p[k-1].t;if(dt<.009f||dt>.101f)return false;for(int j=0;j<14;++j)if(std::abs(a.q[j]-p[k-1].q[j])/dt>37.f||std::abs(a.g[j]-p[k-1].g[j])/dt>250.f)return false;}}
 if(p.front().t!=0||p.back().t<1||f>>extra)return false;out=std::move(p);return true;
}
Sample sample(const Path&p,float t){auto it=std::lower_bound(p.begin(),p.end(),t,[](const Sample&a,float b){return a.t<b;});size_t k=it==p.end()?p.size()-1:std::distance(p.begin(),it),h=k?k-1:0;float v=h==k?0:(t-p[h].t)/(p[k].t-p[h].t);Sample r;r.t=t;for(int j=0;j<14;++j){r.q[j]=p[h].q[j]+v*(p[k].q[j]-p[h].q[j]);r.g[j]=p[h].g[j]+v*(p[k].g[j]-p[h].g[j]);}return r;}
bool matches(const Sample&a,const Sample&b,float qtol=.003f,float gtol=.06f){for(int j=0;j<14;++j)if(std::abs(a.q[j]-b.q[j])>qtol||std::abs(a.g[j]-b.g[j])>gtol)return false;return true;}
struct Cleanup{std::function<void()>fn;~Cleanup(){fn();}};
int main(int argc,char**argv){
 if(argc!=4)return 1;const std::string dir=std::string(argv[1])+"/",run=argv[2];if(std::string(argv[3])=="--validate-segment"){Path p;bool ok=load_path(dir+"trajectory_1.txt",run,1,p);std::cout<<"SEGMENT_VALID="<<ok<<std::endl;return ok?0:2;}bool execute=std::string(argv[3])=="--execute-paper-phase";if(!execute&&std::string(argv[3])!="--dry-run")return 1;
 if(dir.find("/home/unitree/codex_g1_tasks/paper_to_mousepad_")!=0||run.size()>80)return 1;
 int lock=open("/home/unitree/codex_g1_tasks/body_microtest.lock",O_CREAT|O_RDWR,0600);if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 2;signal(SIGINT,stop_task);signal(SIGTERM,stop_task);
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");std::mutex mu;State latest;Clock::time_point received{},battery_received{};int valid=0,invalid=0,other=0,bcount=0,soc=-1;std::deque<uint32_t> own;
 auto states=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<State>("rt/lowstate",[&](const void*p){const auto&s=*(const State*)p;std::lock_guard<std::mutex>g(mu);if(s.crc()!=crc32_core((uint32_t*)&s,(sizeof(s)>>2)-1)){++invalid;return;}latest=s;received=Clock::now();++valid;},0);
 auto commands=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Cmd>("rt/lowcmd",[&](const void*p){const auto&c=*(const Cmd*)p;std::lock_guard<std::mutex>g(mu);if(std::find(own.begin(),own.end(),c.crc())==own.end())++other;},0);
 auto battery=unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Bms>("rt/lf/bmsstate",[&](const void*p){const auto&b=*(const Bms*)p;std::lock_guard<std::mutex>g(mu);soc=b.soc();battery_received=Clock::now();++bcount;},0);
 Cleanup cleanup{[&](){auto a=commands->GetReader()->GetNative();auto b=states->GetReader()->GetNative();auto c=battery->GetReader()->GetNative();a.close();b.close();c.close();commands.reset();states.reset();battery.reset();std::cout<<"DDS_READERS_CLOSED_ON_MAIN"<<std::endl;}};
 std::this_thread::sleep_for(std::chrono::seconds(3));State initial;{std::lock_guard<std::mutex>g(mu);initial=latest;if(valid<500||invalid||other||bcount<2||soc<20||soc>100||Clock::now()-received>std::chrono::milliseconds(100))return 2;}
 if(initial.mode_machine()!=5||initial.mode_pr()!=0)return 2;bimanual::Body base{},baseff{},desired{},ff{};Sample current,home;
 for(int i=0;i<29;++i){auto m=initial.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est())||m.motorstate()||std::abs(m.dq())>bimanual::velocity[i]||std::abs(m.tau_est())>bimanual::effort[i]||m.temperature()[0]>85||m.temperature()[1]>120)return 2;base[i]=desired[i]=m.q();baseff[i]=ff[i]=m.tau_est();if(i>=15)home.q[i-15]=current.q[i-15]=m.q();}
 std::cout<<"CAP_BODY_PREFLIGHT execute="<<execute<<" soc="<<soc<<std::endl;if(!execute)return 0;
 {std::ofstream a(dir+"anchor.json");a<<std::setprecision(10)<<"{\"q\":[";for(int i=0;i<29;++i){if(i)a<<",";a<<base[i];}a<<"],\"tau\":[";for(int i=0;i<29;++i){if(i)a<<",";a<<baseff[i];}a<<"]}";}

 std::mutex cmd_mu;std::shared_ptr<Bundle> pending;std::string loader_error;std::atomic<bool> quitting{false};long loaded_seq=0;std::atomic<long> loader_rejected{0};
 std::thread loader([&](){while(!quitting){std::ifstream f(dir+"request.txt");std::string id,verb;long seq=0;if(f>>id>>seq>>verb&&id==run&&seq>loaded_seq&&seq<10000){auto b=std::make_shared<Bundle>();b->seq=seq;b->verb=verb;bool ok=true;if(verb=="MOVE"||verb=="RECOVER"||verb=="THERMAL_UNLOAD")ok=load_path(dir+"trajectory_"+std::to_string(seq)+".txt",run,seq,b->path)&&load_path(dir+"return_start_"+std::to_string(seq)+".txt",run,seq,b->start_return)&&load_path(dir+"return_end_"+std::to_string(seq)+".txt",run,seq,b->end_return);else ok=verb=="RELEASE_AT_VERIFIED_REST"||verb=="STOP_HOLD"||verb=="STOP";
 if(ok&&(verb=="MOVE"||verb=="RECOVER"||verb=="THERMAL_UNLOAD")){std::vector<float>times;for(const auto&a:b->path)times.push_back(a.t);ok=thermal_v4::load(dir+"thermal_"+std::to_string(seq)+".txt",run,seq,thermal_v4::fnv64_file(dir+"trajectory_"+std::to_string(seq)+".txt"),times,b->thermal)&&thermal_v4::normal_stop_time(b->thermal,times,b->normal_end);}
{std::lock_guard<std::mutex>g(cmd_mu);if(ok)pending=b;else {loader_error="invalid_segment_or_thermal_contract";loader_rejected=b->seq;}}loaded_seq=seq;}std::this_thread::sleep_for(std::chrono::milliseconds(60));}});
 Cleanup join_loader{[&](){quitting=true;if(loader.joinable())loader.join();}};
 unitree::robot::b2::MotionSwitcherClient switcher;switcher.SetTimeout(3.f);switcher.Init();std::string form,owner;int mode_rc=switcher.CheckMode(form,owner);if(mode_rc!=0)return 2;if(!owner.empty()){mode_rc=switcher.ReleaseMode();std::cout<<"OWNERSHIP_RELEASE previous="<<owner<<" rc="<<mode_rc<<std::endl;if(mode_rc!=0)return 2;}
 unitree::robot::ChannelPublisher<Cmd> publisher("rt/lowcmd");publisher.InitChannel();Cmd cmd;cmd.mode_machine()=5;cmd.mode_pr()=0;
 std::ofstream trace(dir+"body_feedback.csv");trace<<std::setprecision(10)<<"steady_s,t,seq,moving,fault,j,q,dq,tau,qgoal,tauff,scale,temp_case,temp_winding,qsent,estimated_tau_cmd\n";
 auto started=Clock::now(),previous=started,next=started;long seq=0,done_seq=0,rejected_seq=0;bool moving=false,fault=false,recovering=false,releasing=false,long_task_notice=false,thermal_unloading=false,thermal_relief_done=false,thermal_fault_seen=false,thermal_no_route=false;int thermal_direction=0;unsigned hot_mask=0,thermal_hot_mask_latched=0;paper_retry::Gate gate;std::string reason;float phase=0,release_time=0,fault_time=0;int failures=0,cycle=0;std::shared_ptr<Bundle> active;Path route;
 auto evidence_for=[&](long request_seq){paper_retry::Evidence e;std::ifstream f(dir+"verified_"+std::to_string(request_seq)+".txt");e.parse(f);return e;};
 auto released_now=[&](const std::string&side,long tx){std::ifstream f(dir+"gripper_"+side+"/control_ack.json");std::ostringstream contents;contents<<f.rdbuf();return paper_retry::fresh_released_ack(contents.str(),run,side,tx,now_s());};
 auto hold=[&](const std::string& why,float t,bool soft_fault){moving=false;recovering=false;thermal_unloading=false;gate.stop(why,t,soft_fault);fault=gate.fault;reason=gate.reason;fault_time=t;std::ofstream(dir+"recovery_required.txt")<<why<<"\n";std::cerr<<"STOP_HOLD "<<why<<" q_and_ff_preserved"<<std::endl;};
 auto hard_failure=[&](const std::string&why){gate.hard_failure(why);std::ofstream(dir+"control_unavailable.txt")<<why<<"\nHOLD_NOT_GUARANTEED\n";std::cerr<<"UNCONTROLLABLE "<<why<<" HOLD_NOT_GUARANTEED"<<std::endl;};
 for(;;){
  auto now=Clock::now();float t=std::chrono::duration<float>(now-started).count(),dt=std::clamp(std::chrono::duration<float>(now-previous).count(),0.f,.02f);previous=now;State s;Clock::time_point when,bwhen;int competition,level;{std::lock_guard<std::mutex>g(mu);s=latest;when=received;bwhen=battery_received;competition=other;level=soc;}
  if(now-when>std::chrono::milliseconds(150)||s.mode_machine()!=5||s.mode_pr()!=0||competition){hard_failure("feedback_mode_or_competition");publisher.CloseChannel();return 3;}
  std::string bad,nonthermal_bad;hot_mask=0;
  for(int i=0;i<29;++i){auto m=s.motor_state()[i];if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est())||m.motorstate()){hard_failure("motor_state_"+std::to_string(i));publisher.CloseChannel();return 3;}
   if(std::abs(m.dq())>bimanual::velocity[i])nonthermal_bad="official_model_velocity";
   if(std::abs(m.tau_est())>bimanual::effort[i])nonthermal_bad="official_model_effort";
   if(m.temperature()[0]>85||m.temperature()[1]>120){if(i>=15)hot_mask|=1u<<(i-15);else nonthermal_bad="temperature_outside_arm_contract";}
  }
  if(level<20||now-bwhen>std::chrono::seconds(5))nonthermal_bad="battery_reserve";
  double heartbeat=0;{std::ifstream f(dir+"guardian_heartbeat.txt");f>>heartbeat;}if(t>6&&now_s()-heartbeat>2)nonthermal_bad="guardian_stale";
  bad=nonthermal_bad.empty()?(hot_mask?"sdk_temperature":""):nonthermal_bad;failures=bad.empty()?0:failures+1;
  if(t>1200&&!long_task_notice){long_task_notice=true;std::ofstream(dir+"body_long_task_notice.txt")<<"elapsed="<<t<<" telemetry_only_no_control_transition\n";}
  bool explicit_stop_this_cycle=stop_requested||access((dir+"abort_requested").c_str(),F_OK)==0;
  // Explicit stop/abort wins over thermal recovery. Never silently restart after it.
  if(stop_requested&&!releasing&&(moving||!fault))hold("signal",t,true);
  if(access((dir+"abort_requested").c_str(),F_OK)==0&&!releasing)hold("abort_requested",t,true);
  {std::ifstream f(dir+"stop_hold_requested.txt");std::string why;if(f>>why){
   bool thermal_notice=thermal_v4::ignore_thermal_notice(why,!nonthermal_bad.empty(),hot_mask,thermal_unloading,thermal_relief_done);
   if(!thermal_notice){explicit_stop_this_cycle=true;hold(why,t,true);}else std::cout<<"THERMAL_NOTICE_NOT_A_SECOND_STOP "<<why<<std::endl;
   f.close();unlink((dir+"stop_hold_requested.txt").c_str());}}
  auto context_ok=[&](const thermal_v4::Contract&c){
   for(const auto&side:{std::string("left"),std::string("right")}){
    std::ifstream f(dir+"gripper_"+side+"/control_ack.json");std::ostringstream os;os<<f.rdbuf();std::string j=os.str();
    try{double stamp=std::stod(paper_retry::ack_field(j,"steady"));double target=std::stod(paper_retry::ack_field(j,"sent_target"));long gs=std::stol(paper_retry::ack_field(j,"seq"));
     if(paper_retry::ack_field(j,"run")!=run||paper_retry::ack_field(j,"side")!=side||!paper_retry::ack_bool(paper_retry::ack_field(j,"controllable"))||paper_retry::ack_bool(paper_retry::ack_field(j,"moving"))||paper_retry::ack_field(j,"error")!="0"||now_s()-stamp>1||stamp>now_s()||gs!=(side=="left"?c.left_seq:c.right_seq)||std::abs(target-(side=="left"?c.left_target:c.right_target))>1e-5)return false;
    }catch(...){return false;}
   }return true;
  };
  if(failures>25&&!fault&&!releasing){
   bool only_temperature=thermal_v4::soft_temperature_only(true,!nonthermal_bad.empty(),hot_mask);
   if(only_temperature){thermal_fault_seen=true;int d=active?thermal_v4::direction(active->thermal,phase,hot_mask):0;
    if(active&&thermal_v4::permitted(true,!nonthermal_bad.empty(),hot_mask,d,context_ok(active->thermal),explicit_stop_this_cycle)){
     fault=true;gate.stop("sdk_temperature_registered_unload",t,true);reason=gate.reason;fault_time=t;recovering=true;thermal_unloading=true;thermal_relief_done=false;thermal_direction=d;thermal_hot_mask_latched=hot_mask;moving=true;route=active->path;
     std::cout<<"THERMAL_UNLOAD_BEGIN direction="<<d<<" contract="<<active->thermal.id<<" phase="<<phase<<std::endl;
    }else {thermal_no_route=true;hold("thermal_no_registered_safe_direction_onsite_required",t,true);}
   }else hold(bad,t,true);
  }
  if(thermal_unloading)thermal_hot_mask_latched|=hot_mask;
  if(thermal_unloading&&(!nonthermal_bad.empty()||!active||!context_ok(active->thermal)||!thermal_v4::allowed_direction(active->thermal,phase,thermal_hot_mask_latched,thermal_direction))){thermal_no_route=true;hold("thermal_unload_health_or_context_lost_onsite_required",t,true);}
  if(recovering&&!thermal_unloading&&failures>25&&!releasing)hold("recovery_interrupted_"+bad,t,true);
  gate.tick(t);rejected_seq=std::max(rejected_seq,loader_rejected.load());
  std::shared_ptr<Bundle> request;{std::lock_guard<std::mutex>g(cmd_mu);request=pending;pending.reset();}
  if(request){
   if(request->verb=="STOP"||request->verb=="STOP_HOLD"){
    hold(request->verb=="STOP"?"supervisor_stop":"operator_stop_hold",t,request->verb=="STOP");seq=request->seq;done_seq=seq;
   }else if(request->verb=="RELEASE_AT_VERIFIED_REST"){
    auto e=evidence_for(request->seq);
    // home is a command-continuity condition; fresh visual/rest evidence is required separately.
    if(t>=5&&gate.can_release(moving,releasing,matches(current,home,.012f,.15f),e,run,request->seq,seq,now_s())&&released_now("left",e.release_side=="left"?e.release_tx:0)&&released_now("right",e.release_side=="right"?e.release_tx:0)){
     releasing=true;release_time=t;seq=request->seq;std::cout<<"VERIFIED_REST_RELEASE "<<e.evidence_id<<std::endl;
    }else rejected_seq=request->seq;
   }else{
    auto e=evidence_for(request->seq);
    bool thermal_request=request->verb=="THERMAL_UNLOAD";
    bool allowed=!explicit_stop_this_cycle&&(thermal_request?(!moving&&!releasing&&thermal_fault_seen&&thermal_v4::soft_temperature_only(true,!nonthermal_bad.empty(),hot_mask)&&context_ok(request->thermal)&&thermal_v4::allowed_direction(request->thermal,0,hot_mask,1)):(gate.can_move(moving,releasing,request->verb=="RECOVER",e,run,request->seq,seq,now_s(),bad.empty())&&(!fault||released_now(e.release_side,e.release_tx))&&context_ok(request->thermal)));
    bool continuity=matches(request->path.front(),current)&&matches(request->start_return.front(),request->path.front())&&matches(request->end_return.front(),sample(request->path,request->normal_end))&&matches(request->start_return.back(),home,.008f,.05f)&&matches(request->end_return.back(),home,.008f,.05f);
    if(allowed&&continuity&&t>=5){active=request;route=request->path;phase=0;moving=true;recovering=fault;seq=request->seq;thermal_unloading=thermal_request;thermal_direction=thermal_request?1:0;thermal_hot_mask_latched=thermal_request?hot_mask:0;thermal_relief_done=false;thermal_no_route=false;std::cout<<"ACCEPTED_SEGMENT "<<seq<<std::endl;}else rejected_seq=request->seq;
   }
  }
  // Normal motion stops at its reviewed place waypoint. Only registered thermal
  // recovery may use the attached low-load tail; neither branch infers release.
  if(moving){auto progress=thermal_v4::advance(phase,dt,active->normal_end,route.back().t,thermal_unloading,thermal_direction);phase=progress.phase;current=sample(route,phase);
   if(progress.complete){
    moving=false;recovering=false;
    if(thermal_unloading){thermal_unloading=false;thermal_relief_done=true;reason="thermal_registered_model_endpoint_reached";std::cout<<"THERMAL_MODEL_ENDPOINT_REACHED kind="<<(thermal_direction<0?active->thermal.start_kind:active->thermal.end_kind)<<" actual_support_not_a_sensor_proof"<<std::endl;}else done_seq=seq;
   }
  }
  for(int j=0;j<14;++j){desired[j+15]=current.q[j];ff[j+15]=baseff[j+15]+current.g[j];}
  float scale=releasing?1-smooth((t-release_time)/5):smooth(t/4);std::array<float,29> estimated{};
  for(int i=0;i<29;++i){auto&m=cmd.motor_cmd()[i];float gain=KP[i]*(i>=15?3.f:1.f),damping=KD[i]*(i>=15?3.f:1.f);float q=s.motor_state()[i].q(),dq=s.motor_state()[i].dq();float total=std::clamp(ff[i]+gain*(desired[i]-q)-damping*dq,-bimanual::effort[i],bimanual::effort[i]);float sent=std::clamp(q+(total-ff[i]+damping*dq)/gain,bimanual::lower[i],bimanual::upper[i]);m.mode()=releasing&&scale<=0?0:1;m.q()=sent;m.dq()=0;m.kp()=gain*scale;m.kd()=damping*scale;m.tau()=ff[i]*scale;estimated[i]=(ff[i]+gain*(sent-q)-damping*dq)*scale;}
  cmd.crc()=crc32_core((uint32_t*)&cmd,(sizeof(cmd)>>2)-1);{std::lock_guard<std::mutex>g(mu);own.push_back(cmd.crc());if(own.size()>2048)own.pop_front();}if(!publisher.Write(cmd)){hard_failure("publisher_write_failed");publisher.CloseChannel();return 3;}
  if(cycle%10==0){double stamp=now_s();for(int i=0;i<29;++i){auto m=s.motor_state()[i];trace<<stamp<<","<<t<<","<<seq<<","<<moving<<","<<fault<<","<<i<<","<<m.q()<<","<<m.dq()<<","<<m.tau_est()<<","<<desired[i]<<","<<ff[i]<<","<<scale<<","<<int(m.temperature()[0])<<","<<int(m.temperature()[1])<<","<<cmd.motor_cmd()[i].q()<<","<<estimated[i]<<"\n";}trace.flush();}
  if(cycle%50==0){std::ofstream f(dir+"body_status.tmp");f<<std::setprecision(10)<<"{\"steady\":"<<now_s()<<",\"elapsed\":"<<t<<",\"soc\":"<<level<<",\"seq\":"<<seq<<",\"done_seq\":"<<done_seq<<",\"rejected_seq\":"<<rejected_seq<<",\"moving\":"<<moving<<",\"phase\":"<<phase<<",\"fault\":"<<fault<<",\"reason\":\""<<reason<<"\",\"recovering\":"<<recovering<<",\"releasing\":"<<releasing<<",\"thermal_unloading\":"<<thermal_unloading<<",\"thermal_relief_done\":"<<thermal_relief_done<<",\"thermal_no_route\":"<<thermal_no_route<<",\"thermal_coverage_mask\":"<<(active?active->thermal.protected_mask:0)<<",\"thermal_direction\":"<<thermal_direction<<",\"hot_arm_mask\":"<<hot_mask<<",\"hold_state\":\""<<gate.state(moving,releasing)<<"\",\"requires_intervention\":"<<gate.latched<<",\"q\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].q();}f<<"],\"dq\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<s.motor_state()[i].dq();}f<<"],\"qgoal\":[";for(int j=0;j<14;++j){if(j)f<<",";f<<current.q[j];}f<<"],\"g\":[";for(int j=0;j<14;++j){if(j)f<<",";f<<current.g[j];}f<<"],\"temperature\":[";for(int i=0;i<29;++i){if(i)f<<",";f<<"["<<int(s.motor_state()[i].temperature()[0])<<","<<int(s.motor_state()[i].temperature()[1])<<"]";}f<<"]}";f.close();rename((dir+"body_status.tmp").c_str(),(dir+"body_status.json").c_str());}
  if(releasing&&t-release_time>5.25f){publisher.CloseChannel();std::ofstream(dir+"body_exited").put('1');std::cout<<"CAP_BODY_RELEASED fault="<<fault<<std::endl;return fault?4:0;}
  ++cycle;next+=std::chrono::milliseconds(2);if(Clock::now()-next>std::chrono::milliseconds(20))next=Clock::now();std::this_thread::sleep_until(next);
 }
}
