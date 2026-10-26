#pragma once
#include <array>
#include <cmath>
#include <sstream>
#include <string>
namespace bimanual {
using Arm = std::array<float,14>;
using Body = std::array<float,29>;
constexpr Arm lo={-2.31f,-.80f,-2.40f,-.90f,-1.89f,-1.50f,-1.50f,-2.31f,-1.91f,-2.40f,-.90f,-1.89f,-1.50f,-1.50f};
constexpr Arm hi={.70f,1.91f,2.40f,1.80f,1.89f,1.50f,1.50f,.70f,.80f,2.40f,1.80f,1.89f,1.50f,1.50f};
struct Request {long seq=0;float duration=0;Arm q{};};
inline bool parse(const std::string& text,const std::string& run,Request& out){
 std::istringstream in(text);std::string id,extra;Request x;
 if(!(in>>id>>x.seq>>x.duration)||id!=run||x.seq<=0)return false;
 for(float& q:x.q)if(!(in>>q)||!std::isfinite(q))return false;
 if(in>>extra)return false;
 if(!std::isfinite(x.duration))return false;
 out=x;return true;
}
inline bool valid(const Request& x,const Body& desired){
 if(x.duration<3.f||x.duration>30.f)return false;
 for(int j=0;j<14;++j){float d=std::abs(x.q[j]-desired[j+15]);
  if(!std::isfinite(x.q[j])||x.q[j]<lo[j]||x.q[j]>hi[j]||d>.25f||d*1.5f/x.duration>.08f)return false;
 }
 return true;
}
inline void apply(Body& body,const Arm& from,const Arm& to,float phase){
 phase=std::fmax(0.f,std::fmin(1.f,phase));float blend=phase*phase*(3.f-2.f*phase);
 for(int j=0;j<14;++j)body[j+15]=from[j]+(to[j]-from[j])*blend;
}
}
