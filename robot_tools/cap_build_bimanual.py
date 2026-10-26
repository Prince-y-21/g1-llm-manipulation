"""Generate a candidate dual-arm publisher from audited legacy source, then test/dry-run only."""
from pathlib import Path
import json,subprocess,time,hashlib,os
parent=Path('/home/unitree/codex_g1_tasks/cap_open_20261021T064124Z');r=parent/'bimanual_development';r.mkdir(exist_ok=False)
s=(Path('/home/unitree/codex_g1_tasks/pour_water_20261019')/'pour_guard_waist.cpp').read_text()
s=s.replace('#include <unistd.h>','#include <unistd.h>\n#include "bimanual_motion.hpp"\n#include <sstream>')
s=s.replace('const std::string dir="/home/unitree/codex_g1_tasks/pour_water_20261019/";','std::string dir;std::string run_id;')
start=s.index(' bool rest_left=');end=s.index(' int fd=open',start)
s=s[:start]+''' if(argc!=3)return 1;
 dir=std::string(argv[1])+"/";
 if(dir.find("/home/unitree/codex_g1_tasks/cap_open_")!=0)return 1;
 run_id="bimanual_development";
 bool execute=std::string(argv[2])=="--execute-suspended";
 if(!execute&&std::string(argv[2])!="--dry-run")return 1;
 const bool rest_left=false,recenter=false,handoff=false;
'''+s[end:]
# Candidate publisher remains disabled without a separately reviewed execution permit.
needle=' if(!execute){std::cout<<"DRY_RUN_NO_PUBLISHER"<<std::endl;return 0;}'
assert needle in s
s=s.replace(needle,needle+'''\n std::ifstream permit(dir+"execution_permit.txt");std::string p;
 if(!(permit>>p)||p!="validated_empty_arm_geometry_and_exit"){std::cerr<<"NO_EXECUTION_PERMIT"<<std::endl;return 2;}''')
# All trajectory/rebase arrays cover both arms; initial body15 remain unchanged.
s=s.replace('std::array<float,7>','std::array<float,14>').replace('j<7','j<14').replace('j+22','j+15').replace('i=22;i<29','i=15;i<29')
s=s.replace('i==22?', '(i==15||i==22)?')
# Pulse API and optional left-rest motion are deliberately absent from this candidate.
start=s.index('  if(cycle%25==0&&!fault&&!moving&&!releasing&&');end=s.index('  if(cycle%50==0&&!fault&&!moving&&!releasing&&',start);s=s[:start]+s[end:]
start=s.index('  if(rest_left&&!fault){');end=s.index('  if(cycle%50==0',start);s=s[:start]+s[end:]
# Replace legacy file parser/limits, keeping original accepted-sequence and response supervision.
start=s.index('   std::ifstream f(dir+"arm_target.txt");');end=s.index('     seq=newseq;',start)
s=s[:start]+'''   std::ifstream f(dir+"arm_target.txt");std::string line;long newseq=0;float newduration=0;std::array<float,14> q{};
   if(std::getline(f,line)){bimanual::Request request;
    bool parsed=bimanual::parse(line,run_id,request);
    if(!parsed){std::cerr<<"INVALID_TARGET_FORMAT_OR_RUN_ID"<<std::endl;}
    newseq=request.seq;newduration=request.duration;q=request.q;
    if(parsed&&newseq>seq){
     bool valid=bimanual::valid(request,desired);
'''+s[end:]
s=s.replace('for(int j=0;j<14;++j)desired[j+15]=from[j]+(target[j]-from[j])*blend;','bimanual::apply(desired,from,target,phase);')
s=s.replace('if(cycle%100==0){','if(cycle%100==0){')
(r/'bimanual_guard.cpp').write_text(s)
for name in ['bimanual_motion.hpp','bimanual_motion_test.cpp']:
 assert (parent/name).exists();(r/name).write_bytes((parent/name).read_bytes())
subprocess.check_call(['g++','-std=c++17','-O2',str(r/'bimanual_motion_test.cpp'),'-o',str(r/'bimanual_motion_test')],timeout=25)
tests=subprocess.check_output([str(r/'bimanual_motion_test')],universal_newlines=True,timeout=5);print(tests,flush=True)
subprocess.check_call(['g++','-std=c++17','-O2','-I/usr/local/include','-I/usr/local/include/ddscxx',str(r/'bimanual_guard.cpp'),'/usr/local/lib/libunitree_sdk2.a','-lddscxx','-lddsc','-lpthread','-o',str(r/'bimanual_guard')],timeout=55)
dry=subprocess.check_output([str(r/'bimanual_guard'),str(r),'--dry-run'],universal_newlines=True,timeout=10);print(dry,flush=True)
record=dict(stage='software_candidate_tested_no_motor_publisher',tests=tests,dry_run=dry,executed=False,permit_created=False,source_sha256=hashlib.sha256(s.encode()).hexdigest(),required_next=['code review of generated controller','source/binary binding for execution permit','fresh full geometry and empty-hand validation','validated payload/thermal recovery supervisor before contact'],utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
(r/'software_validation.json').write_text(json.dumps(record,indent=2));print('BIMANUAL_CANDIDATE_BUILT_NOT_STARTED',r,flush=True)
