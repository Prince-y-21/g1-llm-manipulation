#pragma once
#include <string>
#include <cmath>
namespace paper_retry {
// Flat generated control_ack.json only; no general JSON parsing / no q-based inference.
inline std::string ack_field(const std::string&s,const std::string&key){
 std::size_t k=s.find("\""+key+"\"");if(k==std::string::npos)return "";
 k=s.find(':',k+key.size()+2);if(k==std::string::npos)return "";
 k=s.find_first_not_of(" \r\n\t",k+1);if(k==std::string::npos)return "";
 if(s[k]=='\"'){std::size_t e=s.find('"',k+1);if(e==std::string::npos)return "";return s.substr(k+1,e-k-1);}
 std::size_t e=s.find_first_of(",}\r\n",k);std::string out=s.substr(k,e==std::string::npos?e:e-k);
 while(!out.empty()&&(out.back()==' '||out.back()=='\t'))out.pop_back();return out;
}
inline bool ack_bool(const std::string&s){return s=="1"||s=="true";}
inline bool fresh_released_ack(const std::string&json,const std::string&run,const std::string&side,long tx,double now){
 try{
  std::string moving=ack_field(json,"moving");std::size_t n=0;
  std::string text=ack_field(json,"steady");double stamp=std::stod(text,&n);if(n!=text.size())return false;
  std::string rel=ack_field(json,"release_transaction_seq");long release=std::stol(rel,&n);if(n!=rel.size())return false;
  std::string version=ack_field(json,"protocol_version");long ver=std::stol(version,&n);if(n!=version.size())return false;
  return ver>=2&&ack_field(json,"run")==run&&ack_field(json,"side")==side
   &&std::isfinite(stamp)&&std::isfinite(now)&&stamp<=now&&now-stamp<=1
   &&release>0&&(!tx||release==tx)&&ack_bool(ack_field(json,"controllable"))
   &&(moving=="0"||moving=="false")&&ack_bool(ack_field(json,"release_confirmed"));
 }catch(...){return false;}
}
}
