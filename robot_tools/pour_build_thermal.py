from pathlib import Path
import subprocess
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');s=(r/'pour_guard_waist.cpp').read_text()
s=s.replace('#include <deque>','#include <deque>\n#include <vector>')
s=s.replace(' bool rest_left=',' bool thermal_recovery=argc==2&&std::string(argv[1])=="--thermal-unload-only";\n bool rest_left=',1)
s=s.replace('bool recenter=argc==2&&','bool recenter=thermal_recovery||(argc==2&&',1)
s=s.replace('std::string(argv[1])=="--handoff-suspended-recenter";','std::string(argv[1])=="--handoff-suspended-recenter");',1)
a='||m.temperature()[0]>casing_cutoff||m.temperature()[1]>winding_cutoff){';assert a in s;s=s.replace(a,'||(!thermal_recovery&&(m.temperature()[0]>casing_cutoff||m.temperature()[1]>winding_cutoff))){',1)
a='if(m.temperature()[0]>casing_cutoff||m.temperature()[1]>winding_cutoff){';assert a in s;s=s.replace(a,'if(!thermal_recovery&&(m.temperature()[0]>casing_cutoff||m.temperature()[1]>winding_cutoff)){',1)
# A known temperature fault may only be handed to a bounded whitelist of
# geometry-checked EMPTY-ARM unloading targets. All other guards remain active.
anchor=' Cmd transferred;';assert anchor in s
text='''
 struct RecoveryTarget{long seq;float duration;std::array<float,7> q;};std::vector<RecoveryTarget> recovery_targets;
 if(thermal_recovery){std::ifstream list(dir+"thermal_whitelist.txt");RecoveryTarget v;
  while(list>>v.seq>>v.duration){bool ok=true;for(float& x:v.q)if(!(list>>x)||!std::isfinite(x))ok=false;if(!ok)return 2;recovery_targets.push_back(v);}
  if(recovery_targets.empty()||recovery_targets.size()>40)return 2;
  std::cout<<"THERMAL_UNLOAD_ONLY temperature fault acknowledged; no task or pouring commands accepted; whitelist="<<recovery_targets.size()<<std::endl;
 }
'''
s=s.replace(anchor,text+anchor)
s=s.replace('if(cycle%25==0&&!fault&&!moving&&!releasing&&','if(!thermal_recovery&&cycle%25==0&&!fault&&!moving&&!releasing&&')
anchor='     seq=newseq;\n     if(valid)';assert anchor in s
text='''     if(thermal_recovery){bool authorized=false;for(auto& v:recovery_targets){if(v.seq!=newseq||std::abs(v.duration-newduration)>.001f)continue;bool same=true;for(int j=0;j<7;++j)if(std::abs(v.q[j]-q[j])>1e-5f)same=false;if(same)authorized=true;}valid=valid&&authorized;}
'''
s=s.replace(anchor,text+anchor)
# This is only an explicitly requested release after the arm has cleared desk.
s=s.replace('if(!moving&&!fault&&access((dir+"release_request")','if(!moving&&(!fault||thermal_recovery)&&access((dir+"release_request")')
(r/'pour_guard_thermal.cpp').write_text(s)
subprocess.check_call(['g++','-std=c++17','-O2','-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/'pour_guard_thermal.cpp'),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/'pour_guard_thermal')])
print('THERMAL_WHITELIST_CONTROLLER_BUILT')
