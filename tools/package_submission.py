"""JHY: package sources, verified executable, data, and final documents."""
from pathlib import Path
import hashlib
import json
import re
from zipfile import ZipFile, ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT.parent/'JHY_麦轮底盘课程设计.zip'


def included(path):
    relative=path.relative_to(ROOT)
    if any(x in {'.git','.idea','__pycache__','build-cmake','build-cmake-verified'} for x in relative.parts): return False
    if relative.parts[0]=='build': return False
    if relative.parts[:2]==('Simulation','build'):
        return relative.as_posix()=='Simulation/build/mecanum_sim.exe'
    if path.suffix in {'.pyc','.tmp'} or path.name.startswith('~$'): return False
    if path.name=='文件清单.sha256': return False
    return True


def main():
    files=sorted(p for p in ROOT.rglob('*') if p.is_file() and included(p))
    old_name=re.compile(r'\bHLY\b|侯[淋林]元|hou[ _-]?lin[g]?[ _-]?yuan|Created by 34285',re.I)
    bad=[]
    for p in files:
        if old_name.search(p.relative_to(ROOT).as_posix()): bad.append(str(p))
        if p.name != Path(__file__).name and 'Drivers' not in p.relative_to(ROOT).parts and p.suffix.lower() in {'.c','.h','.md','.txt','.json','.py','.ps1','.cmake','.ioc','.xml'}:
            if old_name.search(p.read_text(encoding='utf-8-sig',errors='replace')): bad.append(str(p))
    if bad: raise RuntimeError('Unreplaced project names: '+str(bad))
    manifest='\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(ROOT).as_posix() for p in files)+'\n'
    manifest_path=ROOT/'docs/文件清单.sha256'
    manifest_path.write_text(manifest,encoding='utf-8')
    files.append(manifest_path)
    with ZipFile(OUT,'w',ZIP_DEFLATED,compresslevel=7) as z:
        for p in files: z.write(p,'JHY_麦轮底盘课程设计/'+p.relative_to(ROOT).as_posix())
    with ZipFile(OUT) as z:
        error=z.testzip()
        if error: raise RuntimeError('Zip CRC failed: '+error)
        required=['README.md','docs/设计文档.docx','docs/实验工作报告.docx','Simulation/build/mecanum_sim.exe','JHY/Build-Firmware.ps1','cmake/gcc-arm-none-eabi.cmake']
        for name in required:
            if 'JHY_麦轮底盘课程设计/'+name not in z.namelist(): raise RuntimeError('Missing '+name)
    print(json.dumps({'archive':str(OUT),'files':len(files),'bytes':OUT.stat().st_size,'legacy_name_scan':'PASS','archive_crc':'PASS'},ensure_ascii=False))


if __name__=='__main__': main()
