"""Build scoped waist-pitch + both-arm profile version; no actuation."""
from pathlib import Path
p=Path(__file__).resolve().parent;s=(p/'bimanual_finite_profile.cpp').read_text()
s=s.replace('EMPTY_CAD_VALIDATION_V1','EMPTY_WAIST_ARMS_VALIDATION_V2').replace('bimanual::Arm q{},g{};','bimanual::Body q{},g{};')
start=s.index(' for(int k=0;k<count;++k)');end=s.index(' int lock=open(',start)
s=s[:start]+''' for(int k=0;k<count;++k){auto &a=plan[k];if(!(f>>a.t)||!std::isfinite(a.t)||a.t<0||a.t>180)return 2;
  for(int i=0;i<29;++i){if(!(f>>a.q[i])||!std::isfinite(a.q[i]))return 2;if(i<14&&std::abs(a.q[i]-anchor[i])>.0001f)return 2;if(i==14&&std::abs(a.q[i]-anchor[i])>.201f)return 2;if(i>=15&&(a.q[i]<bimanual::lo[i-15]||a.q[i]>bimanual::hi[i-15]))return 2;}
  for(int i=0;i<29;++i){if(!(f>>a.g[i])||!std::isfinite(a.g[i]))return 2;if(std::abs(a.g[i])>(i<14?.0001f:(i==14?8.f:5.f)))return 2;}
  if(k){float dt=a.t-plan[k-1].t;if(dt<.009f||dt>.101f)return 2;for(int i=14;i<29;++i)if(std::abs(a.q[i]-plan[k-1].q[i])/dt>(i==14?.031f:.081f)||std::abs(a.g[i]-plan[k-1].g[i])/dt>4.f)return 2;}
 }
 std::string extra;if(f>>extra)return 2;if(plan.front().t!=0)return 2;
 for(int i=0;i<29;++i)if(std::abs(plan.front().q[i]-anchor[i])>.001f||std::abs(plan.back().q[i]-anchor[i])>.001f||std::abs(plan.front().g[i])>.001f||std::abs(plan.back().g[i])>.001f)return 2;
'''+s[end:]
old='for(int j=0;j<14;++j){int i=j+15;desired[i]=measured[i]+plan[h].q[j]+a*(plan[k].q[j]-plan[h].q[j])-anchor[i];ff[i]=baseff[i]+plan[h].g[j]+a*(plan[k].g[j]-plan[h].g[j]);}'
new='for(int i=14;i<29;++i){desired[i]=measured[i]+plan[h].q[i]+a*(plan[k].q[i]-plan[h].q[i])-anchor[i];ff[i]=baseff[i]+plan[h].g[i]+a*(plan[k].g[i]-plan[h].g[i]);}'
assert old in s;s=s.replace(old,new)
s=s.replace('else if(i>=15&&std::abs(m.tau_est())>7.f)','else if(i==14&&std::abs(m.tau_est())>10.f)bad="waist_torque";else if(i>=15&&std::abs(m.tau_est())>7.f)')
s=s.replace('  std::string bad;','''  std::string bad;double dot=0,n0=0,n1=0;for(int j=0;j<4;++j){double a=initial.imu_state().quaternion()[j],b=s.imu_state().quaternion()[j];dot+=a*b;n0+=a*a;n1+=b*b;}
  double root_angle=2*std::acos(std::clamp(std::abs(dot)/std::sqrt(n0*n1),0.,1.));if(!std::isfinite(root_angle)||root_angle>.035)bad="pelvis_rotation";
''')
s=s.replace(' auto start=Clock::now()', ' std::ofstream imu(dir+"imu.csv");imu<<std::setprecision(10)<<"steady_s,t,path_t,root_angle,qw,qx,qy,qz\\n";\n auto start=Clock::now()')
s=s.replace('  if(cycle%10==0){double steady=', '  if(cycle%10==0){imu<<std::chrono::duration<double>(now.time_since_epoch()).count()<<","<<t<<","<<phase<<","<<root_angle;for(int j=0;j<4;++j)imu<<","<<s.imu_state().quaternion()[j];imu<<"\\n";imu.flush();double steady=')
(p/'bimanual_finite_waist.cpp').write_text(s);print('SCOPED_WAIST_CONTROLLER_SOURCE_CREATED')
