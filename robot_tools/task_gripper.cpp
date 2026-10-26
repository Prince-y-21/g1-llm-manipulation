// Right Dex1-1 only. Holds the observed opening until a command file arrives.
// Input: seq target_q_rad speed_rad_per_second. Contact stops closure progress.
#include "serialPort/SerialPort.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <limits.h>
volatile sig_atomic_t stopping=0;
void signal_handler(int){stopping=1;}
int main(){
 const std::string dir="/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/";
 int fd=open((dir+"gripper_guard.lock").c_str(),O_CREAT|O_RDWR,0600);
 if(fd<0||flock(fd,LOCK_EX|LOCK_NB)!=0)return 2;
 if(access((dir+"gripper_target.txt").c_str(),F_OK)==0)return 2;
 signal(SIGINT,signal_handler);signal(SIGTERM,signal_handler);
 // The SDK enumerator accepts canonical tty names, not by-id symlinks.
 char port[PATH_MAX];
 if(!realpath("/dev/serial/by-id/DEVICE_TO_CONFIGURE",port))return 2;
 SerialPort serial(port,16,6000000);
 MotorCmd c;c.motorType=MotorType::M4010;c.hex_len=0;c.id=0;c.res=0;
 c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);
 c.q=c.dq=c.tau=c.kp=c.kd=0;c.timeout=1;
 MotorData d;d.motorType=c.motorType;
 bool ok=false;
 for(int attempt=0;attempt<3&&!ok;++attempt){
  ok=serial.sendRecv(&c,&d);
  if(!ok)std::this_thread::sleep_for(std::chrono::milliseconds(5));
 }
 if(!ok||!d.correct||d.merror||d.temp>55){std::cerr<<"INITIAL_GRIPPER_READ_FAILED ok="<<ok<<" correct="<<d.correct<<" error="<<d.merror<<" temp="<<d.temp<<std::endl;return 2;}
 float gear=queryGearRatio(c.motorType),q=d.q/gear;
 if(q<0||q>5.62)return 2;
 const float opening=q;float target=q,ramp=q,speed=.15f;
 bool contact=false;int contact_samples=0,errors=0;long seq=0;int cycles=0;
 auto last=std::chrono::steady_clock::now();
 std::cout<<"RIGHT_GRIPPER_HOLD opening="<<opening<<" gear="<<gear<<std::endl;
 while(!stopping){
  auto now=std::chrono::steady_clock::now();
  float dt=std::clamp(std::chrono::duration<float>(now-last).count(),.001f,.02f);last=now;
  if(cycles%20==0){std::ifstream f(dir+"gripper_target.txt");long n;float t,v;
   if(f>>n>>t>>v&&n>seq){
    // Vendor calibration span is 322 degrees (~5.62 rad). Keep .22 rad
    // from that endpoint while permitting opening beyond the startup angle.
    if(!std::isfinite(t)||!std::isfinite(v)||t<opening-2.5f||t>5.40f||v<=0||v>.25f){std::cerr<<"REJECTED_GRIPPER_COMMAND"<<std::endl;return 3;}
    seq=n;target=t;speed=v;contact=false;contact_samples=0;ramp=q;
    std::cout<<"ACCEPTED_GRIPPER seq="<<seq<<" target="<<target<<std::endl;
   }
  }
  if(!contact)ramp+=std::clamp(target-ramp,-speed*dt,speed*dt);
  // Vendor example uses output-side kp=5, kd=.05. Limit position error
  // separately so the requested PD effort remains at most .4 Nm.
  float command_q=std::clamp(ramp,q-.08f,q+.08f);
  c.mode=queryMotorMode(c.motorType,MotorMode::FOC);c.timeout=0;
  c.q=command_q*gear;c.dq=0;c.tau=0;c.kp=5.f/(gear*gear);c.kd=.05f/(gear*gear);
  ok=serial.sendRecv(&c,&d);
  if(!ok||!d.correct){if(++errors>3){std::cerr<<"GRIPPER_COMM_FAULT"<<std::endl;break;}continue;}
  errors=0;q=d.q/gear;float velocity=d.dq/gear,torque=d.tau*gear;
  if(d.merror||d.temp>55||std::abs(torque)>.60f){std::cerr<<"GRIPPER_PROTECTION error="<<d.merror<<" torque="<<torque<<" temperature="<<d.temp<<std::endl;break;}
  if(!contact&&ramp<q-.05f&&std::abs(velocity)<.025f&&torque<-.30f){
   if(++contact_samples>60){contact=true;ramp=q-.06f;std::cout<<"CONTACT q="<<q<<" torque="<<torque<<std::endl;}
  }else contact_samples=0;
  if(cycles%20==0){std::ofstream f(dir+"gripper_status.tmp");
   f<<"{\"seq\":"<<seq<<",\"q\":"<<q<<",\"dq\":"<<velocity<<",\"torque\":"<<torque<<",\"temperature\":"<<d.temp<<",\"error\":"<<d.merror<<",\"contact\":"<<(contact?"true":"false")<<",\"target\":"<<target<<",\"ramp\":"<<ramp<<",\"initial_opening\":"<<opening<<"}";
   f.close();rename((dir+"gripper_status.tmp").c_str(),(dir+"gripper_status.json").c_str());
  }
  ++cycles;std::this_thread::sleep_for(std::chrono::milliseconds(5));
 }
 c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.kp=c.kd=c.q=c.dq=c.tau=0;c.timeout=1;
 serial.sendRecv(&c,&d);std::cout<<"GRIPPER_STOPPED_BRAKE"<<std::endl;return 0;
}

