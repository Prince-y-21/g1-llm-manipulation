// Bounded serial identity probe. Sends BRAKE, never FOC or calibration.
#include "serialPort/SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdlib>
int main(int argc,char**argv){
 if(argc!=3)return 1;
 int id=std::atoi(argv[2]);if(id<0||id>3)return 1;
 SerialPort serial(argv[1],16,6000000);
 std::this_thread::sleep_for(std::chrono::milliseconds(100));
 MotorCmd c;c.motorType=MotorType::M4010;c.hex_len=0;c.id=id;c.res=0;
 c.mode=queryMotorMode(c.motorType,MotorMode::BRAKE);c.q=c.dq=c.tau=c.kp=c.kd=0;c.timeout=1;
 MotorData d;d.motorType=c.motorType;d.correct=false;d.merror=0;d.temp=0;
 for(int k=0;k<3;++k){
  bool ok=serial.sendRecv(&c,&d);
  if(ok&&d.correct){float gear=queryGearRatio(c.motorType);std::cout<<"VALID port="<<argv[1]<<" id="<<id<<" q="<<d.q/gear<<" temp="<<d.temp<<" error="<<d.merror<<" tau="<<d.tau*gear<<std::endl;return 0;}
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
 }
 std::cout<<"NO_VALID_REPLY port="<<argv[1]<<" id="<<id<<std::endl;return 2;
}
