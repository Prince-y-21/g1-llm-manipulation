from pathlib import Path
import os, glob, json, time, subprocess
r=Path('/home/unitree/codex_g1_tasks/bottle_to_mousepad_20261016')
processes=[]
for p in Path('/proc').iterdir():
 if p.name.isdigit():
  try:
   args=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
   if any(k in args for k in ['guard','gripper','teleop','lowlevel','dex1','sport_client']):
    if 'python3' not in args: processes.append([int(p.name),args[:350]])
  except OSError:pass
print(json.dumps(dict(utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uptime=Path('/proc/uptime').read_text().strip(),processes=processes,serials={p:dict(real=os.path.realpath(p),rw=os.access(p,os.R_OK|os.W_OK)) for p in glob.glob('/dev/serial/by-id/DEVICE_TO_CONFIGURE')},cameras={p:os.path.realpath(p) for p in glob.glob('/dev/v4l/by-id/*')},old_files=[p.name for p in r.iterdir()] if r.exists() else []),indent=2))
