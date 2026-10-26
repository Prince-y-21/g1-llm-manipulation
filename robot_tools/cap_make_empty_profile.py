from cap_cad_geometry import *
import sys
source=Path(sys.argv[1]);dest=Path(sys.argv[2]);scope=sys.argv[3];state=json.loads(source.read_text());q0=np.array([j['q'] for j in state['joints']]);sc=Scene();sc.setq(q0);ids=[sc.m.joint(n).dofadr[0] for n in joint_names()];g0=sc.d.qfrc_bias[ids].copy()
if scope=='outward':
    out=q0.copy();out[16]+=.30;out[23]-=.30;way=[q0,out,out,q0,q0];dur=[11,1,11,2]
elif scope=='combined_prepare':
    out=q0.copy();out[16]+=.40;out[23]-=.40;way=[q0,out,out,q0,q0];dur=[10.5,75,10.5,2]
elif scope in ['left_free_opening','left_force_step','left_fullstroke','left_fullstroke90']:
    out=q0.copy();out[16]+=.40;way=[q0,out,out,q0,q0];dur=[10.5,115 if scope=='left_fullstroke90' else (75 if scope=='left_fullstroke' else (24 if scope=='left_force_step' else 18)),10.5,2]
elif scope=='calibration':
    candidate=json.loads((OUT/'calibration_fullbody_candidate.json').read_text())
    assert np.max(np.abs(np.array(candidate['waypoints'][0])-q0))<1e-8
    for row in candidate['checks']:
        assert min(x[0] for x in row['env'].values())>.03
        if row['leg']>0:assert row['self_nearest'][0]>.03
    forward=[np.array(x) for x in candidate['waypoints']];way=[q0.copy()];dur=[]
    for end in forward[1:]+list(reversed(forward[:-1])):
        travel=max(3.05,float(np.max(np.abs(end[15:]-way[-1][15:])))*1.5/.08+.2)
        travel=np.ceil(travel/.05)*.05;way.append(end.copy());dur.append(travel)
        way.append(end.copy());dur.append(.8 if np.max(abs(end-q0))>.01 else 2.)
    assert sum(dur)<180
else:raise ValueError(scope)
samples=[];t0=0
for a,b,dt in zip(way[:-1],way[1:],dur):
    count=int(round(dt/.05))
    for k in range(count):
        phase=k/count;q=a+(b-a)*(phase*phase*(3-2*phase));sc.setq(q);g=sc.d.qfrc_bias[ids]-g0
        samples.append((t0+k*.05,q[15:].copy(),g[15:].copy()))
    t0+=dt
samples.append((t0,q0[15:].copy(),np.zeros(14)))
with dest.open('w') as f:
    f.write('EMPTY_CAD_VALIDATION_V1 '+str(len(samples))+'\n'+' '.join(map(str,q0))+'\n')
    for t,q,g in samples:f.write(' '.join(map(str,np.r_[t,q,g]))+'\n')
dest.with_suffix('.json').write_text(json.dumps(dict(scope=scope,pose_anchor_source=str(source),gravity='Task reference CAD delta g(q_des)-g(q_start), no integral and no carry-over stacking',duration=t0,peak_gravity_delta=float(np.max(np.abs([x[2] for x in samples]))),waypoints=[q.tolist() for q in way],durations=dur),indent=2))
print('FINITE_PROFILE_CREATED',scope,t0,len(samples))
