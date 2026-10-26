#include "retry_thermal_v4.hpp"
#include <iostream>
#include <stdexcept>
#include <fstream>
int checks=0;void expect(bool ok,const char*s){++checks;if(!ok)throw std::runtime_error(s);}
thermal_v4::Contract make(const std::vector<float>&a,const std::vector<float>&b,unsigned mask,bool begin=true,bool end=true){
 thermal_v4::Contract c;c.id="test";c.scene="scene";c.payload="paper";c.proof=std::string(64,'a');c.role="CARRY";c.protected_mask=mask;c.start_kind=begin?"PAYLOAD_SUPPORT_TARGET_MODEL":"NONE";c.end_kind=end?"PAYLOAD_SUPPORT_TARGET_MODEL":"NONE";
 for(std::size_t k=0;k<a.size();++k){thermal_v4::Row r;r.t=float(k);r.load.fill(0);r.load[0]=a[k];r.load[1]=b[k];c.rows.push_back(r);}
 unsigned r=begin?thermal_v4::ARM_MASK:0;for(std::size_t k=0;k<a.size();++k){if(k)for(int j=0;j<14;++j)if(c.rows[k].load[j]+1e-6<c.rows[k-1].load[j])r&=~(1u<<j);c.rows[k].reverse=r;}
 unsigned f=end?thermal_v4::ARM_MASK:0;for(std::size_t kk=a.size();kk>0;--kk){auto k=kk-1;if(k+1<a.size())for(int j=0;j<14;++j)if(c.rows[k].load[j]+1e-6<c.rows[k+1].load[j])f&=~(1u<<j);c.rows[k].forward=f;}return c;
}
int main(){using namespace thermal_v4;std::vector<float> times={0,1,2};auto c=make({1,3,1},{0,0,0},1);
 expect(validate(c,times),"single-peak load path has coverage");
 expect(direction(c,.5f,1)==-1,"before peak returns to modeled support");
 expect(direction(c,1.5f,1)==1,"after peak proceeds to modeled support");
 expect(direction(c,1.f,1)==1,"at peak prefer carry remainder");
 expect(allowed_direction(c,.5f,1,-1),"reverse prefix permitted");
 expect(!allowed_direction(c,.5f,1,1),"forward load increase barred");
 expect(!allowed_direction(c,1.5f,1,-1),"reverse load increase barred");
 expect(direction(c,.5f,2)==0,"unprotected hot joint not claimed covered");
 expect(direction(c,.5f,3)==0,"new simultaneously hot joint invalidates coverage");
 auto valley=make({3,1,3},{0,0,0},1);expect(!validate(valley,times),"load valley has no monotone escape");
 auto mixed=make({1,2,3},{3,2,1},3);expect(!validate(mixed,times),"per-joint conflict rejected despite possible global max");
 auto decrease=make({3,2,1},{0,0,0},1,false,true);expect(validate(decrease,times)&&direction(decrease,.5f,1)==1,"remaining path unloads");
 auto increase=make({1,2,3},{0,0,0},1,true,false);expect(validate(increase,times)&&direction(increase,1.5f,1)==-1,"empty/payload prefix unloads");
 auto forged=c;forged.rows[1].reverse=0;expect(!validate(forged,times),"forged masks rejected");
 auto negative=c;negative.rows[1].load[0]=-1;expect(!validate(negative,times),"negative load rejected");
 auto wrong=c;wrong.rows[1].t=.9;expect(!validate(wrong,times),"wrong phase binding rejected");
 auto actual_claim=c;actual_claim.end_kind="PHYSICALLY_VERIFIED_SUPPORT";expect(!validate(actual_claim,times),"no fake sensor support kind");
 auto none=make({3,1,3},{0,0,0},0,false,false);none.role="NO_AUTOMATIC_RELIEF";expect(validate(none,times),"explicit uncovered normal mode accepted");
 expect(direction(none,.5,1)==0,"uncovered mode never starts thermal route");
 expect(soft_temperature_only(true,false,1),"thermal-only hardware healthy exception");
 expect(!soft_temperature_only(false,false,1),"hard fault cannot be bypassed");
 expect(!soft_temperature_only(true,true,1),"thermal plus nonthermal cannot be bypassed");
 expect(!soft_temperature_only(true,false,0),"no thermal predicate fabrication");
 expect(ignore_thermal_notice("sdk_temperature",false,1,false,false),"thermal notice before activation not second stop");
 expect(ignore_thermal_notice("official_power_or_temperature_recovery_required",false,1,true,false),"thermal active keeps registered unload");
 expect(!ignore_thermal_notice("sdk_temperature",true,1,true,false),"real nonthermal fault still stops");
 expect(!ignore_thermal_notice("external_abort",false,1,true,false),"explicit stop not mistaken for thermal notice");
 expect(!ignore_thermal_notice("sdk_temperature",false,0,false,false),"cold unexpected notification is not silently ignored");
 auto held=c;held.start_kind="PAYLOAD_HELD_LOW_LOAD_MODEL";expect(validate(held,times),"held low-load endpoint need not claim external support");
 expect(permitted(true,false,1,1,true,false),"registered thermal remainder eligible");
 expect(!permitted(true,false,1,1,true,true),"explicit STOP wins even with valid thermal path");
 expect(!permitted(false,false,1,-1,true,false),"hardware fault outranks registered reverse");
 expect(!permitted(true,true,1,1,true,false),"other live fault cannot use thermal exception");
 expect(!permitted(true,false,1,0,true,false),"missing direction never auto runs");
 expect(!permitted(true,false,1,1,false,false),"changed context never auto runs");
 auto stopped=c;stopped.role="NORMAL_END_ROW_1";float normal_end=0;
 expect(normal_stop_time(stopped,times,normal_end)&&normal_end==times[1],"normal place stop is bound to trajectory row");
 stopped.role="NORMAL_END_ROW_999";expect(!normal_stop_time(stopped,times,normal_end),"out-of-range normal stop rejected");
 stopped.role="NORMAL_END_ROW_0";expect(!normal_stop_time(stopped,times,normal_end),"zero-length normal motion rejected");
 stopped.role="NORMAL_END_ROW_1junk";expect(!normal_stop_time(stopped,times,normal_end),"malformed normal stop rejected");
 auto normal=advance(1.5f,1.f,2.f,4.f,false,0);expect(normal.complete&&normal.phase==2,"normal move cannot enter recovery tail");
 auto hot=advance(normal.phase,1.f,2.f,4.f,true,1);expect(!hot.complete&&hot.phase==3,"thermal recovery leaves hot place instead of holding there");
 auto low=advance(hot.phase,2.f,2.f,4.f,true,1);expect(low.complete&&low.phase==4,"thermal tail terminates at low-load endpoint");
 auto back=advance(1.f,2.f,2.f,4.f,true,-1);expect(back.complete&&back.phase==0,"thermal prefix terminates at original low-load endpoint");
 std::cout<<"PASS "<<checks<<" shared thermal contract/direction assertions"<<std::endl;
}
