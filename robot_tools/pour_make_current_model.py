from pathlib import Path
import subprocess,os,json,time,signal,shutil
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019')
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:args=(proc/'cmdline').read_bytes().split(b'\0')
 except OSError:continue
 if len(args)>1 and args[1].decode(errors='replace')==str(r/'pour_current_model.py'):
  os.kill(int(proc.name),signal.SIGTERM)
  time.sleep(.1)
history=r/'planning_history'/str(time.time_ns());history.mkdir(parents=True)
for name in ['pour_current_model.py','planning_current.log','route_right_candidate.json','route_right_ready.json']:
 if (r/name).exists():shutil.copy2(str(r/name),str(history/name))
s=(r/'pour_model.py').read_text().split("if __name__=='__main__':")[0]
s=s.replace('cup=np.array([.370,.183,.032]);cup_radius=.050','cup=np.array([.387,.071,.018]);cup_radius=.049')
s=s.replace('bottle_xy=np.array([.399,-.008]);bottle_height=.190;grip_height=.095','bottle_xy=np.array([.429,-.112]);bottle_height=.192;grip_height=.105')
s+='''
if __name__=='__main__':
 q0=np.array([j['q'] for j in state['joints'][22:29]]);points=[]
 def add(label,q,carried=False,grasp=False):
  flags,c=segment(points[-1]['q'],q,carried,grasp) if points else ([],10.)
  pos,rr=tcp_pose(q);f,_=collision(q,carried,grasp)
  points.append(dict(label=label,q=np.array(q).tolist(),tcp=pos.tolist(),rotation=rr.tolist(),carried=carried,grasp=grasp,flags=flags,endpoint_flag=f,clearance=c))
  print(label,flags,'end',f,'tcp',np.round(pos,4).tolist(),flush=True)
 def pose(label,pos,rr,carried=False,grasp=False):add(label,ik(np.array(pos),rr,np.array(points[-1]['q'])),carried,grasp)
 add('initial',q0)
 q=q0.copy();q[1]=-.30;add('outward',q)
 q[0]=.50;add('retract',q)
 q[3]=0;add('unfold',q)
 q[5]=-.40;add('level',q)
 rot=np.eye(3);pose('outboard',[.30,-.29,.05],rot)
 rot=Rotation.from_euler('z',0.).as_dcm();pose('bridge',[.32,-.22,.08],rot)
 pose('pregrasp',[.374,-.127,table_z(*bottle_xy)+.115],rot,False,True)
 pose('grasp',[bottle_xy[0],bottle_xy[1],table_z(*bottle_xy)+grip_height],rot,False,True)
 for z in [.015,.065,.13,.20]:pose('lift_'+str(z),[bottle_xy[0],bottle_xy[1],z],rot,True,True)
 rot=np.eye(3);pose('pour_rotate',[.429,-.112,.20],rot,True,True)
 pose('pour_approach',[.387,.048,.170],rot,True,True)
 for deg in range(5,71,5):
  rr=Rotation.from_euler('x',-np.deg2rad(deg)).as_dcm()
  mouth=np.array([.387,.048,.255 if deg<=45 else .255-(deg-45)*.0017])
  pos=mouth-rr.dot([0,0,bottle_height-grip_height]);pose('tilt_'+str(deg),pos,rr,True,True)
 (ROOT/'route_right_candidate.json').write_text(json.dumps(points,indent=2))
 assert all(not p['flags'] and not p['endpoint_flag'] for p in points),'Candidate contains geometry flags'
 (ROOT/'route_right_ready.json').write_text(json.dumps(points,indent=2))
 print('RIGHT_READY',len(points),flush=True)
'''
(r/'pour_current_model.py').write_text(s)
p=subprocess.Popen(['python3',str(r/'pour_current_model.py')],stdout=(r/'planning_current.log').open('w'),stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),start_new_session=True)
print('CURRENT_OFFLINE_PID',p.pid)
