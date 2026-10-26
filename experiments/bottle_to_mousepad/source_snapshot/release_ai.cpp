#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <iostream>
#include <thread>
int main(){
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 unitree::robot::b2::MotionSwitcherClient c;c.Init();c.SetTimeout(3.f);
 std::string form,name;int rc=c.CheckMode(form,name);
 std::cout<<"BEFORE rc="<<rc<<" form="<<form<<" name="<<name<<std::endl;
 if(rc||name!="ai")return 2;
 rc=c.ReleaseMode();std::cout<<"RELEASE rc="<<rc<<std::endl;
 std::this_thread::sleep_for(std::chrono::seconds(1));
 form.clear();name.clear();rc=c.CheckMode(form,name);
 std::cout<<"AFTER rc="<<rc<<" form="<<form<<" name="<<name<<std::endl;
 return rc==0&&name.empty()?0:3;
}
