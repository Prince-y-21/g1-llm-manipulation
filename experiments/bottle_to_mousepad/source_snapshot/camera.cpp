#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
int main(){try{
 rs2::pipeline pipe;rs2::config config;
 config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
 config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
 auto profile=pipe.start(config);
 rs2::align align(RS2_STREAM_COLOR);
 rs2::frameset fs;
 for(int i=0;i<20;++i)fs=pipe.wait_for_frames(3000);
 fs=align.process(fs);
 auto color=fs.get_color_frame();auto depth=fs.get_depth_frame();
 auto in=color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
 cv::Mat c(cv::Size(640,480),CV_8UC3,(void*)color.get_data());
 cv::Mat d(cv::Size(640,480),CV_16UC1,(void*)depth.get_data());
 cv::imwrite("/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/head_color.jpg",c);
 cv::imwrite("/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/head_depth.png",d);
 std::ofstream out("/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/head_intrinsics.txt");
 out<<"fx "<<in.fx<<" fy "<<in.fy<<" cx "<<in.ppx<<" cy "<<in.ppy<<" scale "<<depth.get_units()<<std::endl;
 std::cout<<"SAVED_HEAD_COLOR_DEPTH "<<in.fx<<" "<<in.fy<<" "<<in.ppx<<" "<<in.ppy<<" units "<<depth.get_units()<<std::endl;
 pipe.stop();
 for(int i:{0,8}){cv::VideoCapture cam(i,cv::CAP_V4L2);cam.set(cv::CAP_PROP_FRAME_WIDTH,640);cam.set(cv::CAP_PROP_FRAME_HEIGHT,480);cv::Mat frame;for(int n=0;n<6;++n)cam.read(frame);if(!frame.empty())cv::imwrite("/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016/wrist"+std::to_string(i)+".jpg",frame);}
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}

