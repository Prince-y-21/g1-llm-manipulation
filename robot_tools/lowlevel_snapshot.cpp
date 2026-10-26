#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <atomic>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
int main(){
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 std::mutex mutex;std::atomic<int> valid{0},invalid{0},commands{0};
 unitree_hg::msg::dds_::LowState_ last;
 unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_> s("rt/lowstate");
 s.InitChannel([&](const void* p){auto state=*(const unitree_hg::msg::dds_::LowState_*)p;
  if(state.crc()!=crc32_core((uint32_t*)&state,(sizeof(state)>>2)-1)){++invalid;return;}
  std::lock_guard<std::mutex> lock(mutex);last=state;++valid;
 },1);
 unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_> c("rt/lowcmd");
 c.InitChannel([&](const void*){++commands;},1);
 std::this_thread::sleep_for(std::chrono::seconds(4));
 std::lock_guard<std::mutex> lock(mutex);
 std::ofstream f("/tmp/codex-g1-observe/baseline.json");
 f<<"{\"valid_states\":"<<valid.load()<<",\"invalid_states\":"<<invalid.load()<<",\"command_count\":"<<commands.load()<<",\"mode_machine\":"<<(int)last.mode_machine()<<",\"mode_pr\":"<<(int)last.mode_pr()<<",\"joints\":[";
 std::cout<<"VALID "<<valid.load()<<" INVALID "<<invalid.load()<<" LOWCMD_COUNT "<<commands.load()<<" MACHINE "<<(int)last.mode_machine()<<" PR "<<(int)last.mode_pr()<<std::endl;
 for(int i=0;i<29;++i){auto m=last.motor_state()[i];
  if(i)f<<",";
  f<<"{\"index\":"<<i<<",\"q\":"<<m.q()<<",\"dq\":"<<m.dq()<<",\"tau\":"<<m.tau_est()<<",\"mode\":"<<(int)m.mode()<<",\"temperature\":["<<m.temperature()[0]<<","<<m.temperature()[1]<<"],\"motorstate\":"<<m.motorstate()<<"}";
  std::cout<<i<<" q="<<m.q()<<" dq="<<m.dq()<<" tau="<<m.tau_est()<<" mode="<<(int)m.mode()<<" temp="<<m.temperature()[0]<<","<<m.temperature()[1]<<" status="<<m.motorstate()<<std::endl;
 }
 f<<"]}";return valid.load()>100&&invalid.load()==0&&commands.load()==0?0:2;
}
