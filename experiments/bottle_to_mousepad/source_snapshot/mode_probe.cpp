#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <iostream>
int main(){
 unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
 unitree::robot::b2::MotionSwitcherClient m;m.Init();m.SetTimeout(2.f);
 std::string form,name;int rc=m.CheckMode(form,name);
 std::cout<<"CHECK_MODE rc="<<rc<<" form="<<form<<" name="<<name<<std::endl;
 unitree::robot::g1::LocoClient l;l.Init();l.SetTimeout(2.f);
 int fsm=-1;rc=l.GetFsmId(fsm);
 std::cout<<"GET_FSM rc="<<rc<<" fsm="<<fsm<<std::endl;
 unitree::robot::g1::G1ArmActionClient a;a.Init();a.SetTimeout(2.f);
 std::string actions;rc=a.GetActionList(actions);
 std::cout<<"GET_ARM_ACTION_LIST rc="<<rc<<" data="<<actions<<std::endl;
}
