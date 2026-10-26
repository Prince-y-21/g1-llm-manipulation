// Bounded EMPTY left-gripper direction test; no body publisher or object contact.
#include "serialPort/SerialPort.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
volatile sig_atomic_t stop_cycle=0;
void on_cycle_signal(int){stop_cycle=1;}
int main(int argc,char**argv){
 if(argc!=2)return 1;std::string root=argv[1];
 int lock=open((root+"/left_serial_test.lock").c_str(),O_CREAT|O_RDWR,0600);
 if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 2;
 signal(SIGTERM,on_cycle_signal);signal(SIGINT,on_cycle_signal);
 SerialPort serial("/dev/ttyUSB2",16,6000000);std::this_thread::sleep_for(std::chrono::milliseconds(150));
 MotorCmd c;c.motorType=MotorType::M4010;c.hex_len=0;c.id=1;c.res=0;c.timeout=1;c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.q=c.dq=c.tau=c.kp=c.kd=0;
 MotorData d;d.motorType=c.motorType;d.correct=false;d.merror=0;d.temp=0;
 bool ok=false;for(int k=0;k<3&&!ok;++k){ok=serial.sendRecv(&c,&d)&&d.correct;std::this_thread::sleep_for(std::chrono::milliseconds(50));}
 if(!ok||d.merror||d.temp>45)return 3;
 const float g=queryGearRatio(c.motorType),initial=d.q/g;
 if(initial<-.05||initial>.25)return 3;
 std::ofstream csv(root+"/left_cycle.csv");csv<<"elapsed_s,desired_q,actual_q,dq,torque,temp\n";
 auto start=std::chrono::steady_clock::now();int result=0;float minimum=initial,maximum=initial;
 for(;;){
  float elapsed=std::chrono::duration<float>(std::chrono::steady_clock::now()-start).count();
  if(stop_cycle||elapsed>8.3f)break;
  float phase=elapsed<.5f?0.f:(elapsed<3.5f?(elapsed-.5f)/3.f:(elapsed<4.3f?1.f:(elapsed<7.3f?1.f-(elapsed-4.3f)/3.f:0.f)));
  float target=initial-.15f*phase;float q=d.q/g;
  c.mode=queryMotorMode(c.motorType,MotorMode::FOC);c.timeout=0;c.q=std::clamp(target,q-.04f,q+.04f)*g;c.dq=c.tau=0;c.kp=5.f/(g*g);c.kd=.05f/(g*g);
  ok=serial.sendRecv(&c,&d)&&d.correct;
  if(!ok){result=4;break;}
  q=d.q/g;float velocity=d.dq/g,torque=d.tau*g;
  csv<<elapsed<<","<<target<<","<<q<<","<<velocity<<","<<torque<<","<<d.temp<<"\n";
  minimum=std::min(minimum,q);maximum=std::max(maximum,q);
  if(d.merror||d.temp>45||std::abs(torque)>.30f||std::abs(velocity)>.25f||q<initial-.19f||q>initial+.04f){result=5;break;}
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
 }
 c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.q=c.dq=c.tau=c.kp=c.kd=0;c.timeout=1;serial.sendRecv(&c,&d);
 std::cout<<"EMPTY_LEFT_CYCLE initial="<<initial<<" min="<<minimum<<" max="<<maximum<<" final="<<d.q/g<<" result="<<result<<" BRAKE_EXIT\n";
 if(result==0&&(initial-minimum<.08f||std::abs(d.q/g-initial)>.035f))result=6;
 return result;
}
