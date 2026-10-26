// Task-local revision. NOT hardware-tested. See GRIPPER_INTERFACE.md before use.
#include "serialPort/SerialPort.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using Clock=std::chrono::steady_clock;
volatile sig_atomic_t stop_request=0;
double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
void stop(int){stop_request=1;}
constexpr float KP=5.f,KD=.05f,OFFICIAL_PEAK_NM=5.f,OFFICIAL_SPEED_RAD_S=27.f;
void atomic(const std::string&p,const std::string&s){std::ofstream f(p+".tmp");f<<s;f.close();rename((p+".tmp").c_str(),p.c_str());}
struct Request{std::string run,verb,policy="HOLD";long seq=0;float amount=0,speed=0,duration=0,recovery_open=0;};
bool parse(const std::string&path,Request&r){
 std::ifstream f(path);std::string line,extra;if(!std::getline(f,line))return false;std::istringstream in(line);
 if(!(in>>r.run>>r.seq>>r.verb>>r.amount>>r.speed>>r.duration>>r.policy>>r.recovery_open)||(in>>extra))return false;
 return std::isfinite(r.amount)&&std::isfinite(r.speed)&&std::isfinite(r.duration)&&std::isfinite(r.recovery_open)&&(r.policy=="HOLD"||r.policy=="OPEN");
}
std::string number(float v){if(!std::isfinite(v))return "null";std::ostringstream o;o<<std::setprecision(12)<<v;return o.str();}
int main(int argc,char**argv){
 if(argc!=5)return 1;
 const std::string root=argv[1],run=argv[2],side=argv[3],mode=argv[4];
 if(root.find("/home/unitree/codex_g1_tasks/paper_to_mousepad_")!=0||run.empty()||run.size()>80||(side!="left"&&side!="right"))return 1;
 const bool query=mode=="--query-brake";if(!query&&mode!="--execute")return 1;
 const std::string dir=root+"/gripper_"+side+"/",port=side=="left"?"/dev/ttyUSB2":"/dev/ttyUSB1";
 int lock=open(("/home/unitree/codex_g1_tasks/paper_serial_"+side+".lock").c_str(),O_CREAT|O_RDWR,0600);
 if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 2;
 signal(SIGINT,stop);signal(SIGTERM,stop);SerialPort serial(port,16,6000000);
 std::this_thread::sleep_for(std::chrono::milliseconds(150));
 MotorCmd c{};c.motorType=MotorType::M4010;c.hex_len=0;c.id=side=="left"?1:0;c.res=0;c.timeout=1;
 c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.q=c.dq=c.tau=c.kp=c.kd=0;
 MotorData d{};d.motorType=c.motorType;d.correct=false;d.merror=0;d.temp=0;
 bool ok=false;for(int k=0;k<4&&!ok;++k){d.correct=false;ok=serial.sendRecv(&c,&d)&&d.correct;std::this_thread::sleep_for(std::chrono::milliseconds(50));}
 if(!ok||d.merror)return 3;
 const float gear=queryGearRatio(c.motorType);float q=d.q/gear,dq=d.dq/gear,tau=d.tau*gear;
 if(!(gear>0)||!std::isfinite(q)||!std::isfinite(dq)||!std::isfinite(tau))return 3;
 std::cout<<std::setprecision(12)<<"INITIAL_BRAKE_QUERY q="<<q<<" gear="<<gear<<" temp="<<d.temp<<std::endl;
 if(query)return 0;
 float requested=q,reference=q,sent=q,held=q,action_start_q=q,recovery_delta=0;
 double start=seconds(),last_feedback=start,action_start=start,deadline=start;
 long seq=0,done=0,rejected=0,release_transaction_seq=0;int errors=0,cycles=0;
 bool moving=false,closing=false,recovery_open=false,allow_recovery_open=false;
 bool emergency=false,fault_latched=false,release_confirmed=false,exit_requested=false,peak_seen=false,long_task_notice=false;
 bool rearm_required=false,comm_retry_pending=false;
 MotorCmd last_issued=c;
 std::string state="HOLD_LAST_COMMAND",event="NONE";
 std::ofstream trace(dir+"feedback.csv");
 trace<<"steady,feedback_steady,elapsed,seq,state,requested,reference,sent,held,q,dq,torque,temp,merror,communication_ok,controllable,release_confirmed,event\n";
 auto hold_last=[&](){moving=false;closing=false;reference=held=sent;state="HOLD_LAST_COMMAND";};
 auto begin=[&](float target,double duration,bool close,bool recover){
  action_start_q=sent;requested=target;action_start=seconds();deadline=action_start+duration;
  moving=true;closing=close;recovery_open=recover;release_confirmed=false;release_transaction_seq=0;
  state=recover?"RECOVERY_OPEN_RELEASE_MAY_OCCUR":(close?"STEP_CLOSE":"OPEN");
 };
 for(;;){
  double now=seconds(),hb=0;{std::ifstream f(root+"/guardian_heartbeat.txt");f>>hb;}
  if(now-start>1500&&!long_task_notice){long_task_notice=true;atomic(dir+"long_task_notice.txt","telemetry_only_no_control_transition");std::cout<<"LONG_TASK_NOTICE telemetry_only elapsed="<<now-start<<std::endl;}
  if(stop_request||(now-start>4&&now-hb>2)||access((root+"/abort_requested").c_str(),F_OK)==0){
   if(!emergency){emergency=true;hold_last();event="SUPERVISION_STOP_HOLD_PENDING_HANDOFF";}
  }
  if(cycles%5==0){Request r;if(parse(dir+"request.txt",r)&&r.run==run&&r.seq>std::max(seq,rejected)){
   bool accepted=false;
   if(r.verb=="HOLD_LAST_COMMAND"){hold_last();accepted=true;}
   else if(r.verb=="HOLD_MEASURED"&&!fault_latched&&!rearm_required&&ok){
    // Explicitly removes position-error preload. Never use implicitly after grasp.
    moving=false;closing=false;reference=held=sent=q;requested=q;state="HOLD_MEASURED_PRELOAD_REMOVED";release_confirmed=false;release_transaction_seq=0;accepted=true;
   }else if(r.verb=="CONFIRM_RELEASE"&&!moving&&!fault_latched&&!rearm_required&&ok){
    // This is an explicit upper-layer assertion after visual verification, not a sensor inference.
    release_confirmed=true;release_transaction_seq=r.seq;accepted=true;
   }else if(r.verb=="REARM"&&ok&&d.merror==0&&std::isfinite(q)&&std::isfinite(dq)&&std::isfinite(tau)){
    // Preserve a recovered FOC hold's preload; a BRAKE-latched state has none to preserve.
    if(fault_latched){reference=held=sent=q;requested=q;}else hold_last();
    fault_latched=false;rearm_required=false;comm_retry_pending=false;errors=0;peak_seen=false;moving=false;
    release_confirmed=false;release_transaction_seq=0;state="REARMED_HOLD";accepted=true;
   }else if(r.verb=="STOP_AFTER_BODY"&&release_confirmed&&access((root+"/body_exited").c_str(),F_OK)==0){exit_requested=true;accepted=true;}
   else if(r.verb=="FORCE_BRAKE_EXIT"){
    // Explicit hard-stop command may release an object; caller must account for it.
    event="EXPLICIT_FORCE_BRAKE_EXIT_RELEASE_MAY_OCCUR";exit_requested=true;accepted=true;
   }else if((r.verb=="STEP_CLOSE"||r.verb=="OPEN"||r.verb=="POSITION")&&!fault_latched&&!rearm_required&&ok){
    bool close=r.verb=="STEP_CLOSE";float target=close?sent-r.amount:r.amount;
    // POSITION retained only for opening. No old full-travel automatic closing.
    bool direction=close?(!emergency&&r.amount>0):(target>=sent);
    double distance=std::abs(double(target)-sent);
    // Cubic time profile max speed=1.5*distance/duration. This is action timing, not a new protection threshold.
    if(direction&&r.speed>0&&r.speed<=OFFICIAL_SPEED_RAD_S&&r.duration>0&&r.duration<=120&&
       1.5*distance/r.duration<=double(r.speed)+1e-7&&r.recovery_open>=0&&
       (r.policy!="OPEN"||(r.recovery_open>0&&std::max(1.,1.5*double(r.recovery_open)/.17)<=120))){
     allow_recovery_open=r.policy=="OPEN";recovery_delta=r.recovery_open;peak_seen=false;
     begin(target,r.duration,close,false);accepted=true;
    }
   }
   if(accepted){seq=r.seq;if(!moving)done=seq;}else rejected=r.seq;
  }}
  if(moving){double u=std::clamp((now-action_start)/(deadline-action_start),0.,1.);double h=u*u*(3.-2.*u);reference=action_start_q+(requested-action_start_q)*h;}
  c.motorType=MotorType::M4010;c.hex_len=0;c.id=side=="left"?1:0;c.res=0;c.timeout=0;
  c.q=c.dq=c.tau=c.kp=c.kd=0;
  if(comm_retry_pending){c=last_issued;}
  else if(fault_latched){c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.timeout=1;}
  else{
   c.mode=queryMotorMode(c.motorType,MotorMode::FOC);
   // Limit total estimated output PD effort, not only feed-forward. No integral term.
   float effort=std::clamp(KP*(reference-q)-KD*dq,-OFFICIAL_PEAK_NM,OFFICIAL_PEAK_NM);
   sent=q+(effort+KD*dq)/KP;c.q=sent*gear;c.kp=KP/(gear*gear);c.kd=KD/(gear*gear);
  }
  // Snapshot the fully initialized packet BEFORE SDK packing/sendRecv can mutate it.
  if(!comm_retry_pending)last_issued=c;
  d.correct=false;ok=serial.sendRecv(&c,&d)&&d.correct;
  if(!ok){
   ++errors;rearm_required=true;release_confirmed=false;release_transaction_seq=0;hold_last();event="COMMUNICATION_UNCONTROLLABLE";
   // Up to 3 retries after the first failed FOC exchange, with exactly the same packet.
   // No trajectory advance, no recomputation from stale feedback, no fresh measured ACK.
   comm_retry_pending=errors<4&&!fault_latched&&last_issued.mode==queryMotorMode(c.motorType,MotorMode::FOC);
   if(!comm_retry_pending){fault_latched=true;state="BRAKE_LATCHED_COMMUNICATION_FAULT";}
  }else{
   bool was_retry=comm_retry_pending;comm_retry_pending=false;
   last_feedback=now;q=d.q/gear;dq=d.dq/gear;tau=d.tau*gear;errors=0;
   if(was_retry){hold_last();state="COMM_RECOVERED_HOLD_REARM_REQUIRED";event="COMM_RECOVERED_NOT_AUTOMATICALLY_REARMED";}
   if(!std::isfinite(q)||!std::isfinite(dq)||!std::isfinite(tau)||d.merror){
    fault_latched=true;rearm_required=true;release_confirmed=false;release_transaction_seq=0;hold_last();state="BRAKE_LATCHED_DEVICE_FAULT";event=d.merror?"DEVICE_MERROR_UNCONTROLLABLE":"NONFINITE_FEEDBACK_UNCONTROLLABLE";
   }else if((std::abs(tau)>OFFICIAL_PEAK_NM||std::abs(dq)>OFFICIAL_SPEED_RAD_S)&&(!peak_seen||moving)&&!fault_latched){
    bool was_closing=closing;peak_seen=true;hold_last();release_confirmed=false;release_transaction_seq=0;
    event="SOFTWARE_OFFICIAL_LIMIT_OBSERVED_NOT_FIRMWARE_FAULT";
    // Freeze the last transmitted reference; do not silently discard grip preload.
    state="LIMIT_HOLD_LAST_COMMAND_RELEASE_PLAN_REQUIRED";
    if(was_closing&&allow_recovery_open&&std::abs(dq)<=OFFICIAL_SPEED_RAD_S){
     // Only the command's explicitly authorized finite recovery can release an object.
     reference=held=sent=q;requested=q;
     begin(q+recovery_delta,std::max(1.,1.5*double(recovery_delta)/.17),false,true);
    }
   }
   if(moving&&now>=deadline){
    // Freeze the final command actually transmitted; DO NOT set it to measured q.
    bool recovered=recovery_open;hold_last();done=seq;
    event=recovered?"RECOVERY_OPEN_DONE_RELEASE_UNVERIFIED":"ACTION_TIME_COMPLETE_RESULT_UNVERIFIED";
   }
  }
  bool controllable=ok&&!fault_latched&&!rearm_required&&d.merror==0&&now-last_feedback<.8;
  std::ostringstream s;s<<std::setprecision(12)<<"{\"protocol_version\":2,\"run\":\""<<run<<"\",\"side\":\""<<side<<"\",\"steady\":"<<last_feedback<<",\"written_steady\":"<<now<<",\"elapsed\":"<<now-start
   <<",\"seq\":"<<seq<<",\"done_seq\":"<<done<<",\"rejected_seq\":"<<rejected<<",\"q\":"<<number(q)<<",\"dq\":"<<number(dq)<<",\"torque\":"<<number(tau)
   <<",\"temperature\":"<<int(d.temp)<<",\"error\":"<<int(d.merror)<<",\"emergency\":"<<emergency<<",\"controllable\":"<<controllable<<",\"communication_ok\":"<<ok
   <<",\"release_confirmed\":"<<release_confirmed<<",\"release_transaction_seq\":"<<release_transaction_seq<<",\"moving\":"<<moving<<",\"fault_latched\":"<<fault_latched<<",\"rearm_required\":"<<rearm_required<<",\"communication_failures\":"<<errors<<",\"requested_target\":"<<number(requested)<<",\"reference_target\":"<<number(reference)<<",\"sent_target\":"<<number(sent)
   <<",\"held_target\":"<<number(held)<<",\"action_deadline\":"<<deadline<<",\"state\":\""<<state<<"\",\"event\":\""<<event<<"\"}";
  atomic(dir+"status.json",s.str());atomic(dir+"control_ack.json",s.str());
  std::ostringstream safe;safe<<std::setprecision(12)<<run<<" "<<last_feedback<<" "<<q<<" "<<(controllable&&release_confirmed&&!moving)<<" "<<int(d.merror);
  atomic(dir+"safe.txt",safe.str());
  trace<<std::setprecision(12)<<now<<","<<last_feedback<<","<<now-start<<","<<seq<<","<<state<<","<<requested<<","<<reference<<","<<sent<<","<<held<<","<<q<<","<<dq<<","<<tau<<","<<int(d.temp)<<","<<int(d.merror)<<","<<ok<<","<<controllable<<","<<release_confirmed<<","<<event<<"\n";trace.flush();
  if(exit_requested)break;
  ++cycles;std::this_thread::sleep_for(std::chrono::milliseconds(5));
 }
 c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.timeout=1;c.q=c.dq=c.tau=c.kp=c.kd=0;
 d.correct=false;bool final_ok=serial.sendRecv(&c,&d)&&d.correct;
 std::ostringstream ended;ended<<"{\"steady\":"<<seconds()<<",\"final_brake_reply\":"<<final_ok<<",\"merror\":"<<int(d.merror)<<",\"release_confirmed\":"<<release_confirmed<<"}";
 atomic(dir+"exited",ended.str());std::cout<<"EXPLICIT_BRAKE_EXIT "<<ended.str()<<std::endl;return final_ok&&d.merror==0?0:4;
}
