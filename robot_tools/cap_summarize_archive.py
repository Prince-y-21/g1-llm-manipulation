"""Local evidence verification and accounting; never connects to robot."""
from pathlib import Path
import tarfile,json,hashlib,csv

root=Path(__file__).resolve().parent.parent/'observations/cap_open_20261021/resume'
archive=root/'resume_remaining_evidence.tgz';out=root/'server_evidence'
expected='0a515f138b6f38356131c29667eeaeb3bc9d22a39e4096ab80652da4112b2aea'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
out.mkdir(exist_ok=False)
with tarfile.open(archive) as tar:
    for m in tar.getmembers():
        assert (out/m.name).resolve().is_relative_to(out.resolve())
        assert m.isfile() or m.isdir()
    tar.extractall(out,filter='data')
manifest=json.loads((root/'resume_remaining_manifest.json').read_text())
assert all((out/m['path']).is_file() and (out/m['path']).stat().st_size==m['bytes'] for m in manifest)
runs=[]
for folder in sorted(out.iterdir()):
    if not folder.is_dir():continue
    row={'name':folder.name}
    for name in ['result.json','status.json','after.json']:
        p=folder/name
        if p.exists():
            try:row[name]=json.loads(p.read_text())
            except Exception as e:row[name]={'read_error':str(e)}
    for name in ['run.log','cycle.log']:
        p=folder/name
        if p.exists():row[name]=p.read_text(errors='replace')[-3500:]
    p=folder/'feedback.csv'
    if p.exists():
        with p.open() as f:
            reader=csv.DictReader(f);n=0;first=None;last=None
            for a in reader:
                if first is None:first=a
                last=a;n+=1
        row['feedback_csv']={'rows':n,'first':first,'last':last}
    if len(row)>1:runs.append(row)
(root/'evidence_index.json').write_text(json.dumps({'archive_sha256':expected,'verified_files':len(manifest),'runs':runs},indent=2),encoding='utf-8')
for r in runs:
    result=r.get('result.json',{});status=r.get('status.json',{});fb=r.get('feedback_csv',{});last=fb.get('last') or {}
    print(r['name'],'exit',result.get('exit_code'),'fault',status.get('fault'),'t',last.get('t'),'log',r.get('run.log','').splitlines()[-3:])
print('Verified files',len(manifest))
