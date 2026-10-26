"""Scoped empty-arm accuracy test; proportional correction, no integrator."""
from pathlib import Path
p=Path(__file__).resolve().parent;s=(p/'bimanual_finite_profile.cpp').read_text()
old='for(int i=0;i<29;++i){auto&m=cmd.motor_cmd()[i];m.mode()=releasing&&scale<=0?0:1;m.q()=desired[i];m.dq()=0;m.kp()=KP[i]*scale;m.kd()=KD[i]*scale;m.tau()=ff[i]*scale;}'
new='''std::array<float,29> estimated_command{};
  for(int i=0;i<29;++i){auto&m=cmd.motor_cmd()[i];float qsend=desired[i],damping=KD[i];
   if(i>=15){int j=i-15,k=j%7;damping*=3.f;float q=s.motor_state()[i].q(),dq=s.motor_state()[i].dq();float pterm=KP[i]*std::clamp(3.f*(desired[i]-q),-.065f,.065f);float limit=k<2?6.5f:(k<4?4.f:(k==6?1.f:1.5f));float total=std::clamp(ff[i]+pterm-damping*dq,-limit,limit);qsend=q+(total-ff[i]+damping*dq)/KP[i];qsend=std::clamp(qsend,bimanual::lo[j]+.01f,bimanual::hi[j]-.01f);}
   m.mode()=releasing&&scale<=0?0:1;m.q()=qsend;m.dq()=0;m.kp()=KP[i]*scale;m.kd()=damping*scale;m.tau()=ff[i]*scale;estimated_command[i]=(ff[i]+KP[i]*(qsend-s.motor_state()[i].q())-damping*s.motor_state()[i].dq())*scale;
  }'''
assert old in s;s=s.replace(old,new)
s=s.replace('temp_case,temp_winding\\n','temp_case,temp_winding,qsent,estimated_tau_cmd\\n')
old='<<int(m.temperature()[1])<<"\\n";';new='<<int(m.temperature()[1])<<","<<cmd.motor_cmd()[i].q()<<","<<estimated_command[i]<<"\\n";';assert old in s;s=s.replace(old,new)
(p/'bimanual_finite_servo.cpp').write_text(s);print('EMPTY_PROPORTIONAL_SERVO_SOURCE_CREATED')
