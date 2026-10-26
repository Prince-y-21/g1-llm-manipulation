#pragma once
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace thermal_v4 {
static const unsigned ARM_MASK=(1u<<14)-1;
static const float NUMERIC_EPS=1e-6f; // serialization/comparison tolerance, not a hardware ceiling
struct Row {float t=0;unsigned reverse=0,forward=0;std::array<float,14> load{};};
struct Contract {
 std::string id,scene,payload,proof,role,start_kind,end_kind,digest;
 unsigned protected_mask=0;long left_seq=0,right_seq=0;double left_target=0,right_target=0;
 std::vector<Row> rows;
};
inline std::string fnv64_file(const std::string&path){
 std::ifstream f(path,std::ios::binary);if(!f)return "";uint64_t h=14695981039346656037ull;char c;
 while(f.get(c)){h^=static_cast<unsigned char>(c);h*=1099511628211ull;}
 static const char*hex="0123456789abcdef";std::string out(16,'0');for(int i=15;i>=0;--i){out[i]=hex[h&15];h>>=4;}return out;
}
inline bool support_kind(const std::string&s){return s=="NONE"||s=="EMPTY_LOW_LOAD_MODEL"||s=="PAYLOAD_SUPPORT_TARGET_MODEL"||s=="PAYLOAD_HELD_LOW_LOAD_MODEL"||s=="ARMS_SUPPORT_MODEL";}
inline bool normal_stop_time(const Contract&c,const std::vector<float>&times,float&out){
 if(times.empty())return false;
 const std::string prefix="NORMAL_END_ROW_";
 if(c.role.rfind(prefix,0)!=0){out=times.back();return true;}
 try{std::size_t used=0;std::string value=c.role.substr(prefix.size());unsigned long index=std::stoul(value,&used);
  if(used!=value.size()||index==0||index>=times.size()-1)return false;
  out=times[index];return std::isfinite(out)&&out>0;
 }catch(...){return false;}
}
struct Advance{float phase;bool complete;};
inline Advance advance(float phase,float dt,float normal_end,float full_end,bool unloading,int direction){
 float limit=unloading?full_end:normal_end;
 float next=std::clamp(phase+(unloading?direction:1)*dt,0.f,limit);
 return {next,unloading&&direction<0?next<=0:next>=limit};
}
inline bool validate(Contract&c,const std::vector<float>&times){
 if(c.rows.size()!=times.size()||times.size()<2||(c.protected_mask&~ARM_MASK)||!support_kind(c.start_kind)||!support_kind(c.end_kind)||c.id.empty()||c.scene.empty()||c.payload.empty()||c.proof.size()!=64)return false;
 bool no_auto=c.role=="NO_AUTOMATIC_RELIEF";
 if(no_auto){if(c.protected_mask||c.start_kind!="NONE"||c.end_kind!="NONE")return false;}
 else if(!c.protected_mask||(c.start_kind=="NONE"&&c.end_kind=="NONE"))return false;
 unsigned r=c.start_kind=="NONE"?0:ARM_MASK;
 for(std::size_t k=0;k<c.rows.size();++k){auto&a=c.rows[k];if(std::abs(a.t-times[k])>NUMERIC_EPS)return false;
  for(int j=0;j<14;++j){if(!std::isfinite(a.load[j])||a.load[j]<0)return false;if(k&&a.load[j]+NUMERIC_EPS<c.rows[k-1].load[j])r&=~(1u<<j);}
  if(a.reverse!=r)return false;
 }
 unsigned f=c.end_kind=="NONE"?0:ARM_MASK;
 for(std::size_t kk=c.rows.size();kk>0;--kk){std::size_t k=kk-1;auto&a=c.rows[k];if(k+1<c.rows.size())for(int j=0;j<14;++j)if(a.load[j]+NUMERIC_EPS<c.rows[k+1].load[j])f&=~(1u<<j);if(a.forward!=f)return false;}
 // Every interpolation interval needs one common direction for ALL protected joints.
 for(std::size_t k=1;k<c.rows.size();++k)if((c.rows[k].reverse&c.protected_mask)!=c.protected_mask&&(c.rows[k-1].forward&c.protected_mask)!=c.protected_mask)return false;
 return true;
}
inline bool load(const std::string&file,const std::string&run,long seq,const std::string&digest,const std::vector<float>&times,Contract&out){
 std::ifstream f(file);std::string version,rid,extra;long nseq;int n;Contract c;
 if(!(f>>version>>rid>>nseq>>c.digest>>c.id>>c.scene>>c.payload>>c.proof>>c.role>>c.start_kind>>c.end_kind>>c.protected_mask>>c.left_seq>>c.right_seq>>c.left_target>>c.right_target>>n))return false;
 if(version!="THERMAL_ROUTE_V1"||rid!=run||nseq!=seq||c.digest!=digest||n<2||n>5000||!std::isfinite(c.left_target)||!std::isfinite(c.right_target))return false;
 c.rows.resize(n);for(auto&a:c.rows){if(!(f>>a.t>>a.reverse>>a.forward))return false;for(float&v:a.load)if(!(f>>v))return false;}
 if(f>>extra||!validate(c,times))return false;out=c;return true;
}
inline bool allowed_direction(const Contract&c,float phase,unsigned mask,int direction){
 if(!mask||(mask&~c.protected_mask)||c.rows.empty())return false;
 auto it=std::lower_bound(c.rows.begin(),c.rows.end(),phase,[](const Row&a,float t){return a.t<t;});
 std::size_t hi=it==c.rows.end()?c.rows.size()-1:static_cast<std::size_t>(it-c.rows.begin());std::size_t lo=hi?hi-1:0;
 if(std::abs(c.rows[hi].t-phase)<=NUMERIC_EPS)lo=hi;
 return direction>0?(c.rows[lo].forward&mask)==mask:(c.rows[hi].reverse&mask)==mask;
}
// +1 follows the prevalidated remaining path, -1 reverses its prevalidated prefix.
// This is a model-load condition, never a promise about actual torque/thermal recovery.
inline int direction(const Contract&c,float phase,unsigned hot_mask){
 if(!hot_mask||(hot_mask&~c.protected_mask)||c.rows.empty())return 0;
 auto it=std::lower_bound(c.rows.begin(),c.rows.end(),phase,[](const Row&a,float t){return a.t<t;});
 std::size_t hi=it==c.rows.end()?c.rows.size()-1:static_cast<std::size_t>(it-c.rows.begin());
 std::size_t lo=hi?hi-1:0;if(std::abs(c.rows[hi].t-phase)<=NUMERIC_EPS)lo=hi;
 bool forward=(c.rows[lo].forward&hot_mask)==hot_mask;
 bool reverse=(c.rows[hi].reverse&hot_mask)==hot_mask;
 // Prefer the carry remainder to support when both are valid; do not return home blindly.
 if(forward)return 1;if(reverse)return -1;return 0;
}
inline bool ignore_thermal_notice(const std::string&why,bool nonthermal_bad,unsigned hot_mask,bool active,bool complete){
 return (why=="sdk_temperature"||why=="official_power_or_temperature_recovery_required")&&!nonthermal_bad&&(hot_mask||active||complete);
}
inline bool soft_temperature_only(bool hardware_healthy,bool other_bad,unsigned hot_mask){return hardware_healthy&&!other_bad&&hot_mask!=0;}
inline bool permitted(bool hardware_healthy,bool other_bad,unsigned hot_mask,int direction,bool context,bool explicit_stop){
 return !explicit_stop&&soft_temperature_only(hardware_healthy,other_bad,hot_mask)&&context&&(direction==1||direction==-1);
}
}
