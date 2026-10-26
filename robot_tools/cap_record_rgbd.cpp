// Read-only image recorder; has no DDS or motor interface.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <unistd.h>
using Clock=std::chrono::steady_clock;
double stamp(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
int main(int argc,char**argv){if(argc!=4)return 1;std::string dir=std::string(argv[1])+"/";double max_s=std::stod(argv[3]);if(max_s<1||max_s>240)return 1;
 try{
  rs2::pipeline pipe;rs2::config cfg;cfg.enable_device(argv[2]);cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);auto profile=pipe.start(cfg);rs2::align align(RS2_STREAM_COLOR);
  for(int k=0;k<25;++k)pipe.wait_for_frames(2000);auto in=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
  std::ofstream meta(dir+"camera_metadata.json");meta<<std::setprecision(17)<<"{\"sdk_serial\":\""<<argv[2]<<"\",\"fx\":"<<in.fx<<",\"fy\":"<<in.fy<<",\"cx\":"<<in.ppx<<",\"cy\":"<<in.ppy<<",\"depth_scale\":"<<profile.get_device().first<rs2::depth_sensor>().get_depth_scale()<<"}";meta.close();
  std::atomic<bool> done{false};
  auto wrist=[&](int device){cv::VideoCapture cam(device,cv::CAP_V4L2);cam.set(cv::CAP_PROP_FRAME_WIDTH,640);cam.set(cv::CAP_PROP_FRAME_HEIGHT,480);cam.set(cv::CAP_PROP_BUFFERSIZE,1);std::ofstream index(dir+"wrist"+std::to_string(device)+"_frames.csv");index<<std::setprecision(17)<<"seq,steady_start,steady_end\n";int k=0;while(!done){auto next=Clock::now()+std::chrono::milliseconds(450);cv::Mat frame;double a=stamp();cam.read(frame);double b=stamp();if(!frame.empty()){std::string name=dir+"wrist"+std::to_string(device)+"_"+std::to_string(k)+".jpg";cv::imwrite(name,frame);index<<k++<<","<<a<<","<<b<<"\n";index.flush();}std::this_thread::sleep_until(next);}cam.release();};
  std::thread left(wrist,0),right(wrist,8);std::ofstream index(dir+"head_frames.csv");index<<std::setprecision(17)<<"seq,steady_start,steady_end,color_frame,depth_frame,color_device_ms,depth_device_ms\n";
  std::ofstream(dir+"ready").put('1');double start=stamp(),last=0;int k=0;
  while(stamp()-start<max_s&&access((dir+"stop").c_str(),F_OK)!=0){double a=stamp();auto frames=pipe.wait_for_frames(1500);if(stamp()-last<.19)continue;frames=align.process(frames);auto color=frames.get_color_frame();auto depth=frames.get_depth_frame();double b=stamp();last=b;
   cv::Mat rgb(cv::Size(640,480),CV_8UC3,(void*)color.get_data());cv::Mat z(cv::Size(640,480),CV_16UC1,(void*)depth.get_data());cv::imwrite(dir+"head_"+std::to_string(k)+".jpg",rgb);cv::imwrite(dir+"depth_"+std::to_string(k)+".png",z);
   index<<k++<<","<<a<<","<<b<<","<<color.get_frame_number()<<","<<depth.get_frame_number()<<","<<color.get_timestamp()<<","<<depth.get_timestamp()<<"\n";index.flush();
  }
  done=true;left.join();right.join();pipe.stop();std::cout<<"PASSIVE_RECORDING_COMPLETE head_frames="<<k<<std::endl;return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}
}
