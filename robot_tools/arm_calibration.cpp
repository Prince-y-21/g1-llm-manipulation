// Bounded out-and-back calibration. Dry run is the default.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

using Clock = std::chrono::steady_clock;
std::atomic<bool> interrupted{false};
void interrupt(int) { interrupted.store(true); }

int main(int argc, char** argv) {
    const bool execute = argc == 2 && std::string(argv[1]) == "--execute";
    if (argc > 1 && !execute) return 1;
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    std::mutex mutex;
    unitree_hg::msg::dds_::LowState_ latest;
    Clock::time_point last{};
    int count = 0;
    unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_> sub("rt/lowstate");
    sub.InitChannel([&](const void* p) {
        std::lock_guard<std::mutex> lock(mutex);
        latest = *static_cast<const unitree_hg::msg::dds_::LowState_*>(p);
        last = Clock::now();
        ++count;
    }, 1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    unitree_hg::msg::dds_::LowState_ start;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (count < 100 || Clock::now() - last > std::chrono::milliseconds(100)) {
            std::cerr << "ABORT: no fresh joint state" << std::endl;
            return 2;
        }
        start = latest;
    }
    if (start.mode_machine() != 5) {
        std::cerr << "ABORT: unexpected hardware model" << std::endl;
        return 2;
    }
    for (int i=12; i<29; ++i) {
        const auto& s=start.motor_state()[i];
        if (!std::isfinite(s.q()) || !std::isfinite(s.tau_est()) ||
            std::abs(s.dq()) > .1 || std::abs(s.tau_est()) > 10 ||
            std::abs(s.q()) > .15 || s.temperature()[0] > 65 || s.temperature()[1] > 65) {
            std::cerr << "ABORT: initial state outside calibration envelope at " << i << std::endl;
            return 2;
        }
    }
    // The only motion target is right shoulder pitch -0.015 rad, then return.
    // All other upper-body targets remain at their observed initial positions.
    std::cout << "PLAN right_shoulder_pitch_delta=-0.015rad duration=4s "
                 "takeover=4s hold=15s return=4s release=4s" << std::endl;
    if (!execute) {
        std::cout << "DRY_RUN: no command publisher was created" << std::endl;
        return 0;
    }
    unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_> pub("rt/arm_sdk");
    pub.InitChannel();
    unitree_hg::msg::dds_::LowCmd_ command;
    command.mode_machine()=start.mode_machine();
    command.mode_pr()=start.mode_pr();
    std::array<float,17> current_target{};
    for(int i=12;i<29;++i) current_target[i-12]=start.motor_state()[i].q();
    bool fault=false;
    float weight=0;
    auto sample = [&]() {
        std::lock_guard<std::mutex> lock(mutex);
        return latest;
    };
    auto send = [&](float delta, float coefficient, bool monitor) {
        auto s=sample();
        bool fresh;
        {
            std::lock_guard<std::mutex> lock(mutex);
            fresh=Clock::now()-last < std::chrono::milliseconds(150);
        }
        if (monitor && (!fresh || interrupted.load())) fault=true;
        for(int i=12;i<29;++i) {
            float q=start.motor_state()[i].q()+(i==22?delta:0.f);
            auto& mc=command.motor_cmd()[i];
            mc.mode()=1; mc.q()=q; mc.dq()=0; mc.kp()=60; mc.kd()=1.5;
            mc.tau()=start.motor_state()[i].tau_est();
            if(monitor && (std::abs(s.motor_state()[i].q()-q)>.04 ||
                           std::abs(s.motor_state()[i].dq())>.5 ||
                           s.motor_state()[i].temperature()[0]>65 ||
                           s.motor_state()[i].temperature()[1]>65)) fault=true;
            current_target[i-12]=q;
        }
        weight=coefficient;
        command.motor_cmd()[29].q()=weight;
        pub.Write(command);
    };
    auto stage = [&](const char* name, float seconds, float d0, float d1, float w0, float w1) {
        std::cout << "STAGE " << name << std::endl;
        const int steps=static_cast<int>(seconds*100);
        auto next=Clock::now();
        for(int i=0;i<=steps;++i) {
            float phase=static_cast<float>(i)/steps;
            float blend=phase*phase*(3-2*phase);
            send(d0+(d1-d0)*blend,w0+(w1-w0)*blend,true);
            if(fault) {
                std::cerr << "FAULT stage=" << name << " step=" << i << std::endl;
                return false;
            }
            next+=std::chrono::milliseconds(10);
            std::this_thread::sleep_until(next);
        }
        auto s=sample();
        std::cout<<"STATE "<<name;
        for(int i=12;i<29;++i)std::cout<<" "<<i<<":"<<s.motor_state()[i].q();
        std::cout<<std::endl;
        if(std::string(name)=="CALIBRATION_MOVE" &&
           std::abs((s.motor_state()[22].q()-start.motor_state()[22].q())-d1)>.005) {
            std::cerr<<"NO_CONFIRMED_RESPONSE: trajectory publication is not successful motion"<<std::endl;
            fault=true;
            return false;
        }
        return true;
    };
    bool ok=stage("TAKEOVER",4,0,0,0,1)
         && stage("CALIBRATION_MOVE",4,0,-.015,1,1)
         && stage("CALIBRATION_HOLD",15,-.015,-.015,1,1)
         && stage("RETURN",4,-.015,0,1,1)
         && stage("RELEASE",4,0,0,1,0);
    if(!ok) {
        // Stop trajectory progress, hold the last target and smoothly hand back.
        std::cout<<"FAULT_HAND_BACK"<<std::endl;
        float initial_weight=weight;
        for(int k=0;k<=200;++k){
            command.motor_cmd()[29].q()=initial_weight*(1.f-k/200.f);
            pub.Write(command);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    command.motor_cmd()[29].q()=0;
    for(int i=0;i<5;++i){pub.Write(command);std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    std::cout<<(ok?"DONE_RETURNED_AND_RELEASED":"ABORTED_RELEASED")<<std::endl;
    return ok?0:3;
}
