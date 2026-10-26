#pragma once
#include <string>
#include <istream>
#include <cmath>
namespace paper_retry {
// Software evidence freshness / sequencing checks, not new motor limits.
struct Evidence {
 std::string format,run,evidence_id,release_side; long request_seq=0,body_seq=0,release_tx=0;
 double issued=0,visual=0; int transaction_done=0,visually_released=0,verified_rest=0,path_checked=0;
 bool parse(std::istream&f){return bool(f>>format>>run>>request_seq>>body_seq>>issued>>visual>>release_tx>>release_side>>transaction_done>>visually_released>>verified_rest>>path_checked>>evidence_id);}
 bool valid(const std::string&r,long req,long current,double now,bool rest)const{
  return format=="PAPER_EVIDENCE_V1"&&run==r&&request_seq==req&&body_seq==current
   &&std::isfinite(issued)&&std::isfinite(visual)&&issued<=now&&visual<=issued
   &&now-issued<=5&&now-visual<=60&&release_tx>0&&(release_side=="left"||release_side=="right")&&transaction_done==1
   &&visually_released==1&&(!rest||verified_rest==1)&&(rest||path_checked==1)
   &&!evidence_id.empty()&&evidence_id!="UNVERIFIED";
 }
};
struct Gate {
 bool fault=false,latched=false,uncontrollable=false; double fault_at=0; std::string reason;
 void stop(const std::string&why,double now,bool soft_fault){
  if(soft_fault&&!fault){fault=true;fault_at=now;}
  reason=why;
 }
 void tick(double now){if(fault&&now-fault_at>=20)latched=true;}
 void hard_failure(const std::string&why){uncontrollable=true;reason=why;}
 bool can_move(bool moving,bool releasing,bool recover,const Evidence&e,const std::string&run,long req,long seq,double now,bool healthy)const{
  return !uncontrollable&&!moving&&!releasing&&healthy&&(!fault||(recover&&e.valid(run,req,seq,now,false)));
 }
 bool can_release(bool moving,bool releasing,bool command_home,const Evidence&e,const std::string&run,long req,long seq,double now)const{
  return !uncontrollable&&!moving&&!releasing&&command_home&&e.valid(run,req,seq,now,true);
 }
 const char*state(bool moving,bool releasing)const{
  if(uncontrollable)return "UNCONTROLLABLE";if(releasing)return "VERIFIED_REST_RELEASING";
  if(moving)return fault?"EXPLICIT_CHECKED_RECOVERY":"MOVING";
  if(latched)return "LATCHED_HOLD_NEEDS_OPERATOR";if(fault)return "RECOVERY_REQUIRED_HOLD";
  return "COMMAND_HOLD";
 }
};
}
