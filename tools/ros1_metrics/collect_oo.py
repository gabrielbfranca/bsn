#!/usr/bin/env python3
"""Invoke existing cpplcom and CCCC; preserve native outputs, then consolidate."""
import csv
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'reports/ros1_static'
RUNTIME=ROOT/'tools/.metrics-runtime'
COMPILER=RUNTIME/'clang18-root/usr/lib/llvm-18/bin/clang++'
CCCC=RUNTIME/'cccc-root/usr/bin/cccc'
PYTHON=ROOT/'tools/.metrics-venv/bin/python'
ENV=os.environ.copy()
ENV['LD_LIBRARY_PATH']=':'.join([str(RUNTIME/'clang18-root/usr/lib/x86_64-linux-gnu'),str(RUNTIME/'clang18-root/usr/lib/llvm-18/lib')])
ENV['PATH']=str(COMPILER.parent)+os.pathsep+ENV.get('PATH','')
COMMANDS=[]


def write_json(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n')


def run(args,path,ok=(0,),cwd=ROOT,env=ENV):
    path.parent.mkdir(parents=True,exist_ok=True)
    p=subprocess.run([str(x) for x in args],cwd=cwd,env=env,capture_output=True)
    path.write_bytes(p.stdout);path.with_suffix(path.suffix+'.stderr').write_bytes(p.stderr)
    COMMANDS.append({'argv':list(map(str,args)),'cwd':str(cwd),'returncode':p.returncode,
                     'stdout':str(path.relative_to(ROOT)),'stderr':str(path.with_suffix(path.suffix+'.stderr').relative_to(ROOT))})
    write_json(OUT/'oo-commands.json',COMMANDS)
    if p.returncode not in ok:raise RuntimeError(f'{args} failed with {p.returncode}; see {path}.stderr')
    return p


def compile_db(files,path,includes=()):
    data=[{'directory':str(ROOT),'file':str((ROOT/f).resolve()),
           'arguments':[str(COMPILER),'-x','c++','-std=c++11',*[a for inc in includes for a in ['-I',str(inc)]],'-c',str((ROOT/f).resolve())]}
          for f in files]
    write_json(path,data)


def headers():
    """Generate prerequisite ROS interfaces with ROS's own generator, not stubs."""
    dest=RUNTIME/'generated/include'
    env=ENV.copy();env['PYTHONPATH']='/opt/ros/melodic/lib/python2.7/dist-packages'
    for pkg,base in [('archlib',ROOT/'src/archlib'),('messages',ROOT/'src/sa-bsn/messages'),('services',ROOT/'src/sa-bsn/services')]:
        target=dest/pkg;target.mkdir(parents=True,exist_ok=True)
        for f in sorted([*base.rglob('*.msg'),*base.rglob('*.srv')]):
            run(['python2','/opt/ros/melodic/lib/gencpp/gen_cpp.py',f,'-p',pkg,'-o',target,
                 '-I','std_msgs:/opt/ros/melodic/share/std_msgs/msg','-e','/opt/ros/melodic/share/gencpp'],
                OUT/'validation/header-generation'/(f.stem+'.txt'),env=env)
    return dest


def main():
    os.chdir(ROOT)
    version=run([COMPILER,'--version'],OUT/'lcom4/clang-version.txt').stdout.decode()
    fix=OUT/'validation/cpplcom';fix.mkdir(parents=True,exist_ok=True)
    compile_db(['tools/ros1_metrics/fixtures/classes.cpp'],fix/'compile_commands.json')
    run([PYTHON,'-m','cpplcom','--compile-commands',fix/'compile_commands.json','--root',HERE/'fixtures',
         '--json',fix/'native.json','--threshold','0','--jobs','2'],fix/'native.txt',ok=(0,1))
    assert not (fix/'native.txt.stderr').read_text().strip(),(fix/'native.txt.stderr').read_text()
    rows=json.loads((fix/'native.json').read_text());by={r['class']:r for r in rows}
    expected={'fixtures::SharedState':1,'fixtures::SplitState':2,'fixtures::CallBridge':1,
              'fixtures::SeparateGetters':2,'fixtures::Stateless':2}
    checks={n:by[n]['lcom4']==v for n,v in expected.items()}
    checks['only_constructor_omitted']='fixtures::OnlyConstructor' not in by
    checks['declaration_and_implementation_merged']=by['fixtures::Branch']['methods']==2
    write_json(fix/'checks.json',checks);assert all(checks.values()),checks
    upstream=next(RUNTIME.glob('ulfhammarqvist-st-cpplcom-*'))
    # Preserve upstream audit separately: Clang 18 has a known member-template
    # fixture discrepancy. Production eligibility is reviewed against this limit.
    upstream_test=run([PYTHON,'-m','unittest','-v'],fix/'upstream-tests.txt',cwd=upstream,ok=(0,1))
    write_json(fix/'upstream-status.json',{'returncode':upstream_test.returncode,
        'limitation':'member template call in Tmpl fixture not joined with Clang 18' if upstream_test.returncode else None})
    cfix=OUT/'validation/cccc';cfix.mkdir(exist_ok=True)
    run([CCCC,'--outdir='+str(cfix),HERE/'fixtures/cbo.cpp'],cfix/'native.txt')
    cb={m.findtext('name'):int(m.find('coupling_between_objects').get('value'))
        for m in ET.parse(cfix/'cccc.xml').getroot().findall('./oo_design/module')}
    cc_checks={n:cb.get(n)==v for n,v in {'Consumer':1,'SignatureUser':1,'Supplier':2,'Standalone':0}.items()}
    write_json(cfix/'checks.json',cc_checks);assert all(cc_checks.values()),cc_checks
    selected=(OUT/'files.txt').read_text().splitlines()
    generated=headers()
    includes=[generated,Path('/opt/ros/melodic/include')]+sorted(p for p in (ROOT/'src').rglob('include') if p.is_dir())
    legacy='src/sa-bsn/system_manager/enactor/apps/enactor.cpp'
    cpp=[f for f in selected if Path(f).suffix=='.cpp' and f!=legacy]
    write_json(OUT/'lcom4/translation-unit-exclusions.json',[{'file':legacy,
        'reason':'not a CMake target; instantiates abstract Enactor; deployed enactor uses apps/controller.cpp',
        'class_definitions_retained_in':'src/sa-bsn/system_manager/enactor/src/Enactor.cpp'}])
    prior=OUT/'native/cpplcom-native.txt.stderr'
    if prior.exists() and prior.read_text().strip():
        attempt=OUT/'lcom4/attempt-with-legacy-entrypoint';attempt.mkdir(exist_ok=True)
        for ext in ['json','txt','txt.stderr']:
            shutil.copy2(OUT/('native/cpplcom-native.'+ext),attempt/('cpplcom-native.'+ext))
    db=OUT/'lcom4/compile_commands.json';compile_db(cpp,db,includes)
    run([PYTHON,'-m','cpplcom','--compile-commands',db,'--root',ROOT/'src','--exclude','*/test/*','--exclude','*/tests/*',
         '--exclude','*/lepton/*','--exclude','*/bindings/*','--json',OUT/'native/cpplcom-native.json',
         '--threshold','0','--jobs','2'],OUT/'native/cpplcom-native.txt',ok=(0,1))
    errors=(OUT/'native/cpplcom-native.txt.stderr').read_text()
    write_json(OUT/'lcom4/parse-status.json',{'parse_errors':errors.splitlines(),'success':not errors.strip()})
    native=OUT/'native/cccc';native.mkdir(exist_ok=True)
    run([CCCC,'--outdir='+str(native),*selected],native/'native.txt')
    write_json(OUT/'oo-metadata.json',{'tools':{'cpplcom':importlib.metadata.version('cpplcom'),
        'libclang_bindings':importlib.metadata.version('libclang'),'clang':version,'cccc_debian':'1:3.1.4-9'},
        'cpplcom_commit':'3b8ce6150d142da0e58bf7968144f26a5854b647',
        'cpplcom_archive_sha256':hashlib.sha256((RUNTIME/'cpplcom.tar.gz').read_bytes()).hexdigest(),
        'cccc_packages':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (RUNTIME/'debs').glob('cccc_*.deb')},
        'analysis_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in str(p)},
        'source_sha256':{f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in selected},
        'generated_header_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in generated.rglob('*.h')},
        'environment':{k:ENV[k] for k in ['PATH','LD_LIBRARY_PATH']},
        'clang_packages':json.loads((RUNTIME/'clang18-packages.json').read_text())})
    if errors.strip():raise RuntimeError('C++ parse errors; native output preserved; resolve before interpretation')
    from summarize_oo import main as summarize
    summarize()
    print('Native OO reports generated')


if __name__=='__main__':main()
