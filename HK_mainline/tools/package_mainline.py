#!/usr/bin/env python3
"""Package this project, verify every ZIP file, and emit SHA-256 checksums.
Python standard library only. No MATLAB process is started by this script.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET
import zipfile
from datetime import datetime, timezone, timedelta

ROOT = Path(__file__).resolve().parents[1]
CORE_MODELS = {'flappy_hk_quadrotor_position_hover.slx', 'flappy_hk_15hz_slow_fast_hover.slx', 'flappy_hk_15hz_trajectory_tracking.slx'}

def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def include(p, include_results=False):
    rel=p.relative_to(ROOT)
    if any(x in {'.git','__pycache__','slprj','cache','backups'} for x in rel.parts):return False
    if rel.parts[0]=='results' and not include_results:return False
    if rel.name=='PACKAGE_MANIFEST.json' or p.suffix in {'.pyc','.slxc','.tmp','.zip','.log'}:return False
    return True

def role(rel):
    if rel.startswith('reference_results/'):return 'historical_evidence'
    if rel.startswith('validation/'):return 'current_validation'
    if rel.startswith('models/'):
        return 'core_model' if PurePosixPath(rel).name in CORE_MODELS else 'construction_ancestor_or_extension'
    if rel.startswith(('CAD/','assets/')):return 'required_geometry_or_urdf'
    if rel.startswith('third_party/'):return 'upstream_source_and_runtime_json'
    if rel.startswith(('src/','scripts/')):return 'matlab_source'
    return 'documentation_or_entry_utility'

def inspect_models():
    models=[]
    for p in sorted((ROOT/'models').glob('*.slx')):
        paths=[];functions=set()
        with zipfile.ZipFile(p) as z:
            for name in z.namelist():
                if not name.endswith('.xml'):continue
                try: tree=ET.fromstring(z.read(name))
                except ET.ParseError:continue
                for e in tree.iter('P'):
                    if e.get('Name')=='ExtGeomFileName' and e.text:
                        value=e.text.replace('\\','/')
                        assert value.startswith('assets/'),f'External SLX geometry: {p.name}: {value}'
                        target=(ROOT/value).resolve()
                        assert target.is_relative_to(ROOT) and target.is_file(),f'Missing geometry: {value}'
                        paths.append(value)
                    if e.get('Name')=='MATLABFcn' and e.text:functions.add(e.text)
        models.append({'model':p.name,'geometry_paths':sorted(set(paths)),'interpreted_functions':sorted(functions)})
    return models

def verify_zip(archive,manifest):
    expected={ROOT.name+'/'+r['path']:r for r in manifest['files']}
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None,'ZIP CRC verification failed'
        actual=set(z.namelist());assert actual==set(expected)|{ROOT.name+'/PACKAGE_MANIFEST.json'},'ZIP file list mismatch'
        for n,r in expected.items():
            with z.open(n) as f:
                h=hashlib.sha256()
                for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
            assert h.hexdigest()==r['sha256'],f'ZIP hash mismatch: {n}'
    return len(expected)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path);ap.add_argument('--include-results',action='store_true')
    args=ap.parse_args()
    output=(args.output or ROOT.parent/'HK_mainline_20261010.zip').resolve()
    output.parent.mkdir(parents=True,exist_ok=True)
    assert not output.is_relative_to(ROOT),'Place output outside the project to avoid recursive packaging'
    models=inspect_models()
    files=[]
    for p in sorted(ROOT.rglob('*')):
        if not p.is_file() or not include(p,args.include_results):continue
        rel=p.relative_to(ROOT).as_posix()
        files.append({'path':rel,'bytes':p.stat().st_size,'sha256':digest(p),'role':role(rel)})
    manifest={'package':'HK_mainline','release_date':'2026-10-10','created_at':datetime.now(timezone(timedelta(hours=8))).isoformat(),
      'entry':'run_mainline.m','baseline':'HK CAD rigid prescribed-wing + quadrotor, 15 Hz, 0.2032 kg, 2x-wing mass/inertia',
      'source':'dynamic_simulation/Flappy_simscape','include_new_results':args.include_results,'models':models,'files':files}
    (ROOT/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temp=output.with_suffix(output.suffix+'.tmp')
    with zipfile.ZipFile(temp,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for r in files:z.write(ROOT/r['path'],ROOT.name+'/'+r['path'])
        z.write(ROOT/'PACKAGE_MANIFEST.json',ROOT.name+'/PACKAGE_MANIFEST.json')
    count=verify_zip(temp,manifest);temp.replace(output)
    sha=digest(output);output.with_suffix('.sha256').write_text(sha+'  '+output.name+'\n',encoding='ascii')
    print(json.dumps({'archive':str(output),'verified_files':count,'bytes':output.stat().st_size,'sha256':sha,'model_count':len(models)},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
