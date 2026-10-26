"""Offline only. Assemble independently checked outbound, carry and return paths."""
from pathlib import Path
import json
ROOT=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
exec((ROOT/'task_plan.py').read_text().split('q0=np.array')[0])
lower=np.maximum(lower,[-2.3,-1.9,-2.39,-.89,-1.88,-1.49,-1.49])
upper=np.minimum(upper,[.69,.79,2.39,1.79,1.88,1.49,1.49])
base_collision=collision
def check(q,carried=False,upright=False):
    flag,clearance=base_collision(q,False)
    # Closed tool is narrower than open; retain original shape selection but
    # replace the original planar-only carried bottle test with height checks.
    if carried:
        flag,clearance=base_collision(q,True)
        if flag=='CARRIED_BOTTLE_NEAR_WHITE':flag=''
    t,pts=fk(q);tcp=t[:3,3]+t[:3,:3].dot([.19,0,0])
    for a,b,rad in [(pts[1],pts[3],.045),(pts[3],pts[5],.04),(pts[5],pts[6],.04)]:
        for w in np.linspace(0,1,21):
            p=a*(1-w)+b*w
            if -.08-rad<p[0]<.12+rad and abs(p[1])<.09+rad and .28-rad<p[2]<.58+rad:
                return 'HEAD',clearance
    if carried:
        tilt=float(np.arccos(np.clip(t[2,2],-1,1)))
        bottom=tcp[2]-.105-.034*np.sin(tilt)
        if np.linalg.norm(tcp[:2]-[.39,-.04])<.091 and bottom<.155:
            return 'CARRIED_BOTTLE_NEAR_WHITE',clearance
        if bottom<table_z(tcp[0],tcp[1])-.003:return 'BOTTLE_TABLE',clearance
    if upright and t[2,2]<np.cos(np.deg2rad(2)):
        return 'BOTTLE_TILT',clearance
    return flag,clearance

def segment(a,b,carried=False):
    flags=set();minimum=10.
    for w in np.linspace(0,1,101):
        f,c=check(a*(1-w)+b*w,carried,carried)
        if f:flags.add(f)
        minimum=min(minimum,c)
    return sorted(flags),minimum

q0=np.array([j['q'] for j in state['joints'][22:29]])
wp=[('initial',q0,False)]
q=q0.copy();q[1]=-.25;wp.append(('outward',q.copy(),False))
q[0]=.5;wp.append(('retract_below_edge',q.copy(),False))
q[3]=0;wp.append(('raise_forearm_behind_edge',q.copy(),False))
q[5]=-.4;wp.append(('level_tool_behind_edge',q.copy(),False))
reverse=json.loads((ROOT/'reverse_full_test.json').read_text())
assert len(reverse)==52
grasp=np.array(reverse[-1]['q'])
pre,_=ik(np.array([.345,-.166,table_z(.4,-.166)+.13]),0.,grasp)
bridge=np.array(json.loads((ROOT/'approach_bridge.json').read_text())['q'])
wp.append(('approach_outboard',bridge,False))
wp.extend([('pregrasp',pre,False),('grasp',grasp,False)])

def add_phase(label,poses,carried):
    # Keep Cartesian paths short enough that joint interpolation preserves
    # upright orientation and clearance, pruning only after dense checks.
    seq=[wp[-1][1]]+[np.array(p['q']) for p in poses]
    current=0;part=0
    while current<len(seq)-1:
        valid=[]
        for j in range(len(seq)-1,current,-1):
            flags,_=segment(seq[current],seq[j],carried)
            if not flags:
                valid.append(j)
                break
        if not valid:raise RuntimeError('UNSAFE_PHASE '+label+' '+str(current))
        current=max(valid);part+=1
        wp.append((label if current==len(seq)-1 else label+'__part'+str(part),seq[current],carried))
        print('PLANNED',label,current,'/',len(seq)-1,flush=True)

add_phase('lift',list(reversed(reverse[34:51])),True)
add_phase('transfer',list(reversed(reverse[17:34])),True)
add_phase('place',list(reversed(reverse[:17])),True)
add_phase('empty_lift',reverse[1:18],False)
add_phase('empty_transfer',reverse[18:35],False)
add_phase('empty_lower',reverse[35:],False)
wp.append(('empty_pregrasp',pre,False))
wp.append(('return_approach_outboard',bridge,False))
for label,q,carried in reversed(wp[:5]):wp.append(('return_'+label,q.copy(),False))
report=[]
for i,(label,q,carried) in enumerate(wp):
    t,_=fk(q)
    flags,clearance=segment(wp[i-1][1],q,carried) if i else ([],10.)
    report.append(dict(label=label,q=q.tolist(),tcp=(t[:3,3]+t[:3,:3].dot([.19,0,0])).tolist(),
                       collision_flags=flags,minimum_table_clearance_m=clearance))
    print(label,flags,round(clearance,4),flush=True)
(ROOT/'offline_route_candidate.json').write_text(json.dumps(report,indent=2))
assert all(not r['collision_flags'] for r in report),'Unresolved full-route collisions'
(ROOT/'offline_route.json').write_text(json.dumps(report,indent=2))
print('ALL_MODEL_CHECKS_PASSED',len(report),flush=True)
