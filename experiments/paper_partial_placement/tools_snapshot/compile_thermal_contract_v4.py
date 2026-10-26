"""OFFLINE FILE COMPILER. No robot connection, DDS, serial or motor action.
Reads externally verified geometry/load evidence; it does NOT perform that verification.
Produces thermal_SEQ.txt automatically included with the normal MOVE stage files.
"""
from pathlib import Path
import argparse,hashlib,json,math
ALL=(1<<14)-1
EPS=1e-6
KINDS={'NONE','EMPTY_LOW_LOAD_MODEL','PAYLOAD_SUPPORT_TARGET_MODEL','PAYLOAD_HELD_LOW_LOAD_MODEL','ARMS_SUPPORT_MODEL'}
def fnv64(data):
 h=14695981039346656037
 for b in data:h=((h^b)*1099511628211)&((1<<64)-1)
 return f'{h:016x}'
def compile_contract(trajectory,proof_path,output):
 raw=trajectory.read_bytes();proof_raw=proof_path.read_bytes();proof=json.loads(proof_raw)
 words=raw.decode().split();assert words[0]=='CAPSEG1';run=words[1];seq=int(words[2]);count=int(words[3]);assert len(words)==4+count*29
 times=[float(words[4+29*k]) for k in range(count)]
 assert proof['run_id']==run and proof['sequence']==seq
 assert proof['trajectory_sha256']==hashlib.sha256(raw).hexdigest()
 for flag in ['full_geometry_checked','payload_envelope_checked','model_uncertainty_envelope_checked']:
  assert proof.get(flag) is True, 'Missing external verification: '+flag
 no_auto=proof.get('automatic_relief',True) is False
 start,end=proof['start_kind'],proof['end_kind'];assert start in KINDS and end in KINDS
 if not no_auto:assert start!='NONE' or end!='NONE';assert proof.get('interval_load_monotonicity_checked') is True
 else:assert start==end=='NONE' and proof['role']=='NO_AUTOMATIC_RELIEF' and proof.get('uncovered_reason')
 if start!='NONE':assert proof.get('reverse_prefixes_geometry_checked') is True and proof.get('start_endpoint_model_checked') is True
 if end!='NONE':assert proof.get('forward_suffixes_geometry_checked') is True and proof.get('end_endpoint_model_checked') is True
 load=proof['per_arm_joint_model_load_Nm'];assert len(load)==count and all(len(row)==14 and all(math.isfinite(v) and v>=0 for v in row) for row in load)
 protected=proof['protected_body_joints'];assert all(15<=j<=28 for j in protected);assert bool(protected)!=no_auto;mask=sum(1<<(j-15) for j in set(protected))
 reverse=[];r=ALL if start!='NONE' else 0
 for k,row in enumerate(load):
  if k:
   for j in range(14):
    if row[j]+EPS<load[k-1][j]:r&=~(1<<j)
  reverse.append(r)
 forward=[0]*count;f=ALL if end!='NONE' else 0
 for k in range(count-1,-1,-1):
  if k+1<count:
   for j in range(14):
    if load[k][j]+EPS<load[k+1][j]:f&=~(1<<j)
  forward[k]=f
 for k in range(1,count):
  assert reverse[k]&mask==mask or forward[k-1]&mask==mask, f'No nonincreasing-load direction covering interval {k-1}->{k} for all protected joints'
 binding=proof['gripper_binding']
 assert binding['left_moving'] is False and binding['right_moving'] is False
 for key in ['contract_id','scene_id','payload_id','role']:
  assert proof[key] and not any(ch.isspace() for ch in proof[key])
 fields=['THERMAL_ROUTE_V1',run,str(seq),fnv64(raw),proof['contract_id'],proof['scene_id'],proof['payload_id'],hashlib.sha256(proof_raw).hexdigest(),proof['role'],start,end,str(mask),str(binding['left_seq']),str(binding['right_seq']),repr(binding['left_sent_target']),repr(binding['right_sent_target']),str(count)]
 lines=[' '.join(fields)]
 for k in range(count):lines.append(' '.join([repr(times[k]),str(reverse[k]),str(forward[k])]+[repr(float(x)) for x in load[k]]))
 output.write_bytes(('\n'.join(lines)+'\n').encode('utf-8'))
 return dict(run=run,seq=seq,count=count,protected_mask=mask,trajectory_fnv64=fields[3],output=str(output),proof_sha256=fields[7])
def main():
 a=argparse.ArgumentParser();a.add_argument('trajectory',type=Path);a.add_argument('proof',type=Path);a.add_argument('output',type=Path);x=a.parse_args();print(json.dumps(compile_contract(x.trajectory,x.proof,x.output),indent=2))
if __name__=='__main__':main()
