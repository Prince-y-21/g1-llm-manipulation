"""Approximate hydrostatic bottle model, not a calibrated volume sensor."""
import numpy as np,json
from pathlib import Path
z=np.linspace(0,.192,3000)
radius=np.where(z<.138,.033,np.where(z<.170,.033-(z-.138)*(.020/.032),.013))
total=float(np.trapz(np.pi*radius**2,z))
def retained(degrees):
 a=np.deg2rad(degrees)
 if degrees<.0001:return total
 level=.192*np.cos(a)-.013*np.sin(a)
 cut=(z*np.cos(a)-level)/np.sin(a);ratio=np.clip(cut/radius,-1,1)
 area=radius**2*np.arccos(ratio)-cut*np.sqrt(np.maximum(0,radius**2-cut**2))
 area=np.where(cut<=-radius,np.pi*radius**2,np.where(cut>=radius,0,area))
 return float(np.trapz(area,z))
if __name__=='__main__':
 rows=[]
 for onset in range(40,99,2):
  f=retained(onset)/total;angles=np.linspace(onset,110,3001)
  target=float(angles[np.argmin([abs(retained(a)/total-.75*f) for a in angles])])
  rows.append(dict(onset_degrees=onset,inferred_initial_ml=retained(onset)*1e6,estimated_quarter_ml=retained(onset)*.25e6,target_equilibrium_degrees=target))
 out=dict(assumptions='Axisymmetric piecewise cylinder/shoulder/neck; quasi-static free surface; open mouth; inferred amount from onset. Not measured liquid volume. Glugging, meniscus, plastic shape, transient motion and uncertain geometry cause error.',shape_capacity_ml=total*1e6,rows=rows)
 p=Path('/home/unitree/codex_g1_tasks/pour_water_20261019/volume_model.json');p.write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
