// Camera recorder only: no DDS publisher, no serial commands.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
std::atomic<bool> done{false};
const std::string root="/home/unitree/codex_g1_tasks/pour_water_20261019/";
void save(const std::string&name,const cv::Mat&frame){cv::imwrite(root+name+".tmp.jpg",frame);rename((root+name+".tmp.jpg").c_str(),(root+name+".jpg").c_str());}
void wrist(int index,std::string folder){try{cv::VideoCapture cam(index,cv::CAP_V4L2);cam.set(cv::CAP_PROP_FRAME_WIDTH,640);cam.set(cv::CAP_PROP_FRAME_HEIGHT,480);cam.set(cv::CAP_PROP_BUFFERSIZE,1);cv::VideoWriter vid(folder+"/wrist"+std::to_string(index)+".avi",cv::VideoWriter::fourcc('M','J','P','G'),8,cv::Size(640,480));int n=0;auto last=std::chrono::steady_clock::now();while(!done){cv::Mat im;if(!cam.read(im)||im.empty())continue;auto now=std::chrono::steady_clock::now();if(now-last<std::chrono::milliseconds(120))continue;last=now;vid.write(im);save("latest_wrist"+std::to_string(index),im);++n;}}catch(const std::exception&e){std::cerr<<"WRIST "<<index<<" "<<e.what()<<std::endl;}}
int main(int argc,char**argv){if(argc!=2)return 1;std::string label=argv[1];if(label.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_")!=std::string::npos)return 1;
 int fd=open((root+"watch.lock").c_str(),O_CREAT|O_RDWR,0600);if(fd<0||flock(fd,LOCK_EX|LOCK_NB))return 2;
 std::string folder=root+"video_"+label;if(mkdir(folder.c_str(),0700))return 2;
 try{rs2::pipeline pipe;rs2::config cfg;cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);pipe.start(cfg);rs2::align align(RS2_STREAM_COLOR);
 std::thread a(wrist,0,folder),b(wrist,8,folder);cv::VideoWriter vid(folder+"/head.avi",cv::VideoWriter::fourcc('M','J','P','G'),8,cv::Size(640,480));std::ofstream timeline(folder+"/frames.csv"),states(folder+"/states.jsonl");timeline<<"frame,unix_ms\n";auto started=std::chrono::steady_clock::now(),last=started;int n=0;
 while(std::chrono::steady_clock::now()-started<std::chrono::seconds(180)&&access((root+"watch_stop").c_str(),F_OK)!=0){auto fs=align.process(pipe.wait_for_frames(3000));auto now=std::chrono::steady_clock::now();if(now-last<std::chrono::milliseconds(120))continue;last=now;auto cf=fs.get_color_frame();auto df=fs.get_depth_frame();cv::Mat im(cv::Size(640,480),CV_8UC3,(void*)cf.get_data());vid.write(im);save("latest_head",im);auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();timeline<<n<<","<<ms<<"\n";timeline.flush();if(n%8==0){cv::Mat dep(cv::Size(640,480),CV_16UC1,(void*)df.get_data());cv::imwrite(root+"latest_depth.tmp.png",dep);rename((root+"latest_depth.tmp.png").c_str(),(root+"latest_depth.png").c_str());}
 cv::imwrite(folder+"/head_"+std::to_string(n)+".jpg",im);std::ifstream guard(root+"guard_status.json"),grip(root+"gripper_status.json");std::string sg((std::istreambuf_iterator<char>(guard)),{}),ss((std::istreambuf_iterator<char>(grip)),{});if(!sg.empty()&&!ss.empty()){states<<"{\"frame\":"<<n<<",\"unix_ms\":"<<ms<<",\"guard\":"<<sg<<",\"gripper\":"<<ss<<"}\n";states.flush();}
 std::ofstream status(root+"watch_status.tmp");status<<"{\"frame\":"<<n<<",\"unix_ms\":"<<ms<<",\"folder\":\""<<folder<<"\"}";status.close();rename((root+"watch_status.tmp").c_str(),(root+"watch_status.json").c_str());++n;}
 done=true;a.join();b.join();pipe.stop();std::cout<<"CAMERA_WATCH_EXIT frames="<<n<<std::endl;
 }catch(const std::exception&e){done=true;std::cerr<<e.what()<<std::endl;return 1;}return 0;}
