#!/usr/bin/env python3
"""Build a hover-only, relocatable release. Standard library; verify every ZIP member."""
import argparse,hashlib,json,re,zipfile
from pathlib import Path
from datetime import datetime,timezone,timedelta
import package_mainline as common
ROOT=common.ROOT
MODELS={n+'.slx' for n in ['flappy_hk_gui_hover','flappy_hk_15hz_slow_fast_hover',
 'flappy_hk_quadrotor_position_hover','flappy_hk_disturbed_position_hover',
 'flappy_hk_geometry_sil_hover','flappy_mechanics']}
DOCS={'README.md','docs/GUI_GUIDE.md','docs/GUI_VALIDATION.md','docs/PARAMETERS.md'}
VALIDATION={'gui_component_tests.json','gui_simulation_tests.json','hover_gui_preview.png',
 'gui_parameters_roundtrip.json','gui_relocation_tests.json','gui_姿态_preview.png','gui_翼误差_preview.png','gui_电机_preview.png','gui_传感_preview.png','model_consistency.csv','validation_report.json','smoke_report.json'}

def source_closure():
    available={p.stem:p for folder in ['.','src','scripts','tests'] for p in (ROOT/folder).glob('*.m')}
    roots=['hk_hover_gui','run_hover','run_mainline','startup_mainline','validate_mainline','smoke_mainline',
     'build_mainline','build_hover_gui_model','mainline_relocate_models','test_hover_gui','test_hover_gui_simulation','test_quadrotor_core',
     'flappy_quad_controller','flappy_quad_actuator','flappy_sil_wing_loads','flappy_sil_sensor_model',
     'flappy_sil_wing_kinematics','flappy_ground_contact','flappy_sil_motion_kinematics']
    # Include actual interpreted callback functions from each selected SLX.
    models=[m for m in common.inspect_models() if m['model'] in MODELS]
    for m in models:
        for expr in m['interpreted_functions']:
            roots.extend(re.findall(r'\b[A-Za-z]\w*\b',expr))
    pending=roots[:];visited=set()
    while pending:
        name=pending.pop()
        if name in visited or name not in available:continue
        visited.add(name)
        content=available[name].read_text(encoding='utf-8-sig')
        content=re.sub(r'(?m)^\s*%.*$','',content)
        pending.extend(n for n in set(re.findall(r'\b[A-Za-z]\w*\b',content)) if n in available and n not in visited)
    return {available[n].relative_to(ROOT).as_posix() for n in visited},models

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path);args=ap.parse_args()
    output=(args.output or ROOT.parent/'HK_hover_GUI_20261011.zip').resolve()
    assert not output.is_relative_to(ROOT)
    sources,models=source_closure()
    assert len(models)==len(MODELS),'Missing packaged mechanical ancestor or GUI model'
    files=[]
    for p in sorted(ROOT.rglob('*')):
        if not p.is_file() or not common.include(p):continue
        rel=p.relative_to(ROOT).as_posix();parts=p.relative_to(ROOT).parts
        keep=rel in sources or rel in DOCS or (parts[0]=='models' and p.name in MODELS)
        keep=keep or parts[0] in {'CAD','assets','third_party'}
        keep=keep or (parts[0]=='validation' and p.name in VALIDATION)
        keep=keep or rel in {'tools/package_hover_gui.py','tools/package_mainline.py'}
        if not keep:continue
        files.append({'path':rel,'bytes':p.stat().st_size,'sha256':common.digest(p),'role':('primary_hover_model' if p.name=='flappy_hk_gui_hover.slx' else ('mechanical_ancestor' if parts[0]=='models' else common.role(rel)))})
    manifest={'package':'HK_hover_GUI','release_date':'2026-10-11','entry':'hk_hover_gui.m',
      'batch_entry':'run_hover.m','scope':'takeoff and fixed-position hover only',
      'created_at':datetime.now(timezone(timedelta(hours=8))).isoformat(),
      'excluded':'history, previous results, trajectory/pulse/filter models and their scenario entrypoints',
      'models':models,'files':files}
    text=json.dumps(manifest,ensure_ascii=False,indent=2)+'\n'
    (ROOT/'HOVER_PACKAGE_MANIFEST.json').write_text(text,encoding='utf-8')
    temp=output.with_suffix('.zip.tmp');output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(temp,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for f in files:z.write(ROOT/f['path'],'HK_mainline/'+f['path'])
        z.writestr('HK_mainline/PACKAGE_MANIFEST.json',text)
    count=common.verify_zip(temp,manifest);temp.replace(output)
    sha=common.digest(output);output.with_suffix('.sha256').write_text(sha+'  '+output.name+'\n',encoding='ascii')
    print(json.dumps({'archive':str(output),'bytes':output.stat().st_size,'verified_files':count,
     'matlab_files':len(sources),'model_count':len(models),'sha256':sha},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
