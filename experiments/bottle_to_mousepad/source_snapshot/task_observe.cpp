// Passive DDS subscriptions only. Does not publish motor or mode commands.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
int main(int argc,char** argv){
 if(argc!=2)return 1;
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 std::mutex mutex;int valid=0,invalid=0,commands=0,bms_count=0;
 unitree_hg::msg::dds_::LowState_ last;
 unitree_hg::msg::dds_::BmsState_ battery;
 std::array<float,29> minimum{},maximum{};
 unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_> sub("rt/lowstate");
 sub.InitChannel([&](const void* p){auto s=*(const unitree_hg::msg::dds_::LowState_*)p;
  std::lock_guard<std::mutex> lock(mutex);
  if(s.crc()!=crc32_core((uint32_t*)&s,(sizeof(s)>>2)-1)){++invalid;return;}
  for(int i=0;i<29;++i){float q=s.motor_state()[i].q();if(!valid)minimum[i]=maximum[i]=q;minimum[i]=std::min(minimum[i],q);maximum[i]=std::max(maximum[i],q);}
  last=s;++valid;
 },1);
 unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_> cmd("rt/lowcmd");
 cmd.InitChannel([&](const void*){std::lock_guard<std::mutex> l(mutex);++commands;},1);
 unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_> bms("rt/lf/bmsstate");
 bms.InitChannel([&](const void* p){std::lock_guard<std::mutex> l(mutex);battery=*(const unitree_hg::msg::dds_::BmsState_*)p;++bms_count;},1);
 std::this_thread::sleep_for(std::chrono::seconds(5));
 std::lock_guard<std::mutex> l(mutex);
 std::ofstream f(argv[1]);
 f<<"{\"valid_states\":"<<valid<<",\"invalid_states\":"<<invalid<<",\"command_count\":"<<commands<<",\"mode_machine\":"<<(int)last.mode_machine()<<",\"mode_pr\":"<<(int)last.mode_pr()<<",\"tick\":"<<last.tick()<<",\"imu_quaternion\":[";
 for(int i=0;i<4;++i){if(i)f<<",";f<<last.imu_state().quaternion()[i];}
 f<<"],\"bms_messages\":"<<bms_count<<",\"soc\":"<<(int)battery.soc()<<",\"soh\":"<<(int)battery.soh()<<",\"bms_voltage_raw\":[";
 for(int i=0;i<3;++i){if(i)f<<",";f<<battery.bmsvoltage()[i];}
 f<<"],\"bms_current_raw\":"<<battery.current()<<",\"joints\":[";
 for(int i=0;i<29;++i){auto m=last.motor_state()[i];if(i)f<<",";
 f<<"{\"index\":"<<i<<",\"q\":"<<m.q()<<",\"dq\":"<<m.dq()<<",\"tau\":"<<m.tau_est()<<",\"mode\":"<<(int)m.mode()<<",\"temperature\":["<<m.temperature()[0]<<","<<m.temperature()[1]<<"],\"motorstate\":"<<m.motorstate()<<",\"q_range\":"<<maximum[i]-minimum[i]<<"}";
 }
 f<<"]}";f.close();
 std::cout<<"PASSIVE_SNAPSHOT valid="<<valid<<" invalid="<<invalid<<" lowcmd="<<commands<<" battery_messages="<<bms_count<<" soc="<<(int)battery.soc()<<std::endl;
 return valid>100&&invalid==0?0:2;
}
