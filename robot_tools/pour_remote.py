"""SSH transport for this experiment. Does not itself publish robot commands."""
import base64, subprocess, sys
from pathlib import Path
def run(code, timeout=50):
    b=base64.b64encode(code.encode()).decode()
    return subprocess.run(['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=8','robot-g1','printf %s '+b+' | base64 -d | python3'],capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=timeout)
if __name__=='__main__':
    p=Path(sys.argv[1]); code='import sys\nsys.argv='+repr([p.name]+sys.argv[3:])+'\n'+p.read_text(encoding='utf-8');r=run(code,int(sys.argv[2]) if len(sys.argv)>2 else 50)
    print(r.stdout,end='');print(r.stderr,end='',file=sys.stderr);sys.exit(r.returncode)
