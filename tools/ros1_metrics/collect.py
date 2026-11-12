#!/usr/bin/env python3
"""Collect native reports, then join their outputs. No cohesion/CC implementation."""
import argparse
import csv
import copy
import hashlib
import importlib.metadata
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from datetime import datetime, timezone

import networkx as nx

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OUT = ROOT / 'reports/ros1_static'
COMMANDS = []
ENV = os.environ.copy()
ENV.update(PATH=str(HERE)+os.pathsep+str(ROOT/'tools/.metrics-venv/bin')+os.pathsep+ENV.get('PATH',''),
           SEMGREP_SEND_METRICS='off', SEMGREP_ENABLE_VERSION_CHECK='0',
           SEMGREP_SETTINGS_FILE=str(ROOT/'tools/.metrics-runtime/semgrep-settings.yml'))
DOXYGEN = shutil.which('doxygen') or str(ROOT/'tools/.metrics-runtime/doxygen-root/usr/bin/doxygen')
if Path(DOXYGEN).is_relative_to(ROOT):
    ENV['LD_LIBRARY_PATH'] = str(ROOT/'tools/.metrics-runtime/doxygen-root/usr/lib/x86_64-linux-gnu')


def dump(p, obj):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(obj, indent=2, ensure_ascii=False)+'\n')


def run(args, target, ok=(0,)):
    target.parent.mkdir(parents=True, exist_ok=True)
    p = subprocess.run(list(map(str,args)), cwd=ROOT, env=ENV, capture_output=True)
    target.write_bytes(p.stdout)
    target.with_suffix(target.suffix+'.stderr').write_bytes(p.stderr)
    COMMANDS.append({'argv':list(map(str,args)), 'cwd':str(ROOT), 'returncode':p.returncode,
                     'stdout':str(target.relative_to(ROOT)), 'stderr':str(target.with_suffix(target.suffix+'.stderr').relative_to(ROOT))})
    if p.returncode not in ok:
        dump(OUT/'commands.json', COMMANDS)
        raise RuntimeError(f'Command failed ({p.returncode}): {args}; see {target}.stderr')
    return p.stdout.decode(errors='replace')


def table(path, rows, fields=None):
    rows=list(rows)
    if fields is None:
        fields=list(rows[0]) if rows else []
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fields); w.writeheader()
        for r in rows:
            w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in r.items()})


def rel(p):
    return str(Path(p).relative_to(ROOT)) if Path(p).is_absolute() else p


def scope():
    selected=[]; excluded=[]
    for p in sorted((ROOT/'src').rglob('*')):
        if p.suffix not in {'.cpp','.cc','.cxx','.h','.hpp','.hh'} or not p.is_file():
            continue
        r=p.relative_to(ROOT)
        reason=None
        if r.parts[1] not in {'archlib','libbsn','sa-bsn'}: reason='third_party_or_outside_own_production_roots'
        elif {'test','tests','test_bdd','bdd','features'} & set(r.parts): reason='tests_and_helpers'
        elif 'bindings' in r.parts: reason='binding_demo_not_built_by_CMake'
        (excluded if reason else selected).append({'file':str(r),'reason':reason} if reason else str(r))
    table(OUT/'excluded.csv',excluded)
    (OUT/'files.txt').write_text('\n'.join(selected)+'\n')
    return selected


def doxygen(files, dest):
    config=dest/'Doxyfile'
    config.parent.mkdir(parents=True,exist_ok=True)
    config.write_text((HERE/'Doxyfile.base').read_text()+f'\nOUTPUT_DIRECTORY = {dest.relative_to(ROOT)}/doxygen\nINPUT = '+' '.join('"'+f+'"' for f in files)+'\n')
    run([DOXYGEN,config.relative_to(ROOT)],dest/'doxygen.stdout')
    return dest/'doxygen/xml'


def inventory(xml):
    classes={}; members=[]; class_ids={}
    idx=ET.parse(xml/'index.xml').getroot()
    for c in idx.findall('compound'):
        if c.get('kind') not in {'class','struct'}: continue
        node=ET.parse(xml/(c.get('refid')+'.xml')).getroot().find('compounddef')
        name=node.findtext('compoundname'); loc=node.find('location')
        classes[name]={'class':name,'file':rel(loc.get('file')),'line':int(loc.get('line','0')),
                       'id':c.get('refid'),'bases':[], 'node':node}
        class_ids[c.get('refid')]=name
    # Doxygen merges identically named structs defined in distinct .cpp files.
    # Preserve native XML, split derived identities by declaration file.
    for name,c in list(classes.items()):
        files={rel(m.find('location').get('file')) for m in c['node'].findall('./sectiondef/memberdef') if m.find('location') is not None}
        if Path(c['file']).suffix=='.cpp' and len(files)>1:
            del classes[name]
            for f in sorted(files):
                node=copy.deepcopy(c['node'])
                for section in node.findall('sectiondef'):
                    for m in list(section):
                        if m.find('location') is not None and rel(m.find('location').get('file'))!=f: section.remove(m)
                node.find('location').set('file',f)
                identity=f+'::'+name
                classes[identity]={**c,'class':identity,'file':f,'node':node}
    for name,c in classes.items():
        node=c['node']
        c['bases']=[class_ids.get(b.get('refid'),b.text) for b in node.findall('basecompoundref')]
        for m in node.findall('./sectiondef/memberdef'):
            if m.get('kind')!='function': continue  # friends are free functions
            loc=m.find('location'); mn=m.findtext('name')
            members.append({'class':name,'name':mn,'signature':m.findtext('definition','')+m.findtext('argsstring',''),
                'declaration_file':rel(loc.get('file')),'declaration_line':int(loc.get('line','0')),
                'body_file':rel(loc.get('bodyfile','')),'body_start':int(loc.get('bodystart','0')),
                'body_end':int(loc.get('bodyend','0')),'virtual':m.get('virt'), 'static':m.get('static'),
                'constructor':mn==name.split('::')[-1], 'node':m})
    return classes,members,class_ids


def lizard(files,dest):
    exe=ROOT/'tools/.metrics-venv/bin/lizard'
    for fmt,flag in [('csv','--csv'),('html','--html'),('txt',None)]:
        run([exe]+([flag] if flag else [])+files,dest/f'lizard-native.{fmt}',ok=(0,1))
    rows=[]
    for r in csv.reader((dest/'lizard-native.csv').open()):
        rows.append({'file':r[6],'name':r[7],'signature':r[8],'start':int(r[9]),'end':int(r[10]),'cc':int(r[1]),'parameter_count':int(r[3])})
    return rows


def join_methods(classes,members,functions):
    matched=[]; unmatched=[]; used=set()
    norm=lambda n: ''.join(n.split())
    for f in functions:
        candidates=[(i,m) for i,m in enumerate(members)
                    if m['body_file']==f['file'] and m['body_end']==f['end']
                    and m['body_start']>=f['start'] and norm(m['name'])==norm(f['name'].split('::')[-1])]
        if len(candidates)!=1:
            # Doxygen may place body start on the signature line or opening brace.
            candidates=[(i,m) for i,m in enumerate(members) if m['body_file']==f['file']
                        and f['start']<=m['body_start']<=f['end'] and norm(m['name'])==norm(f['name'].split('::')[-1])]
        mapping='native_body_span'
        if not candidates and '::' in f['name']:
            owner=f['name'].rsplit('::',1)[0]
            candidates=[(i,m) for i,m in enumerate(members) if i not in used and m['class']==owner
                        and not m['body_start'] and norm(m['name'])==norm(f['name'].split('::')[-1])
                        and len(m['node'].findall('param'))==f['parameter_count']]
            mapping='qualified_owner_name_and_unique_arity_Lizard_fallback'
            if len(candidates)==1:
                m=candidates[0][1]
                m.update(body_file=f['file'],body_start=f['start'],body_end=f['end'])
        if len(candidates)==1:
            i,m=candidates[0]
            if i in used: raise RuntimeError('Duplicate mapping '+str(f))
            used.add(i); matched.append({**f,'class':m['class'],'constructor':m['constructor'],'mapping':mapping})
        else:
            unmatched.append({**f,'candidate_count':len(candidates)})
    missing=[{k:v for k,v in m.items() if k!='node'} for i,m in enumerate(members) if m['body_start'] and i not in used]
    return matched,unmatched,missing


def graph(classes,ids):
    edges=set(); broad=set()
    for name,c in classes.items():
        for b in c['bases']:
            if b in classes and b!=name: edges.add((name,b,'inheritance')); broad.add((name,b))
        for m in c['node'].findall('./sectiondef/memberdef'):
            # Only type signatures; body references deliberately excluded from type-dependency proxy.
            for typ in [m.find('type')]+m.findall('./param/type'):
                if typ is None: continue
                for ref in typ.findall('.//ref'):
                    dest=ids.get(ref.get('refid'))
                    if dest and dest!=name:
                        broad.add((name,dest))
                        if m.get('kind')=='variable': edges.add((name,dest,'attribute_type'))
    g=nx.DiGraph();g.add_nodes_from(classes); g.add_edges_from((a,b) for a,b,_ in edges)
    components=[sorted(c) for c in nx.strongly_connected_components(g) if len(c)>1]
    return edges,broad,components


def semgrep(files,dest):
    exe=ROOT/'tools/.metrics-venv/bin/pysemgrep'
    flags=['--metrics=off','--disable-version-check','--no-git-ignore','--max-target-bytes','0','--jobs','2']
    for rule in ['isolation-cpp','isolation-extra']:
        run([exe,'--test','--config',HERE/(rule+'.yml'),HERE/'fixtures'/(rule+'.cpp'),'--metrics=off','--disable-version-check'],OUT/'validation'/(rule+'-tests.txt'))
    base=[exe,'scan',*flags,'--config',HERE/'isolation-cpp.yml','--config',HERE/'isolation-extra.yml',*files]
    for fmt in ['json','sarif','text']:
        run(base+(['--'+fmt] if fmt!='text' else []),dest/('semgrep-native.'+fmt))
    data=json.loads((dest/'semgrep-native.json').read_text())
    if data.get('errors'): raise RuntimeError('Semgrep errors: '+str(data['errors']))
    return data


def isolation(data,methods,classes,members):
    findings=[]
    for r in data['results']:
        path=rel(r['path']); line=r['start']['line']; end=r['end']['line']
        # Native Doxygen spans include definitions split from headers; range containment joins findings.
        ms=[m for m in methods if m['file']==path and m['start']<=line<=m['end']]
        owner=ms[0]['class'] if len(ms)==1 else ''
        meth=ms[0]['name'] if len(ms)==1 else ''
        if not owner:
            owners=[c for c in classes.values() if c['file']==path and int(c['node'].find('location').get('bodystart','0'))<=line<=int(c['node'].find('location').get('bodyend','0'))]
            if len(owners)==1: owner=owners[0]['class']
        phase='outside_class' if not owner else ('constructor_direct' if ms and ms[0]['constructor'] else 'setup' if meth.endswith('::setUp') else 'method' if meth else 'class_declaration')
        findings.append({'file':path,'line':line,'end_line':end,'class':owner,'method':meth,'phase':phase,
            'category':r['extra']['metadata'].get('category'),'rule':r['check_id'],
            'source':'\n'.join((ROOT/path).read_text().splitlines()[line-1:end])})
    ctor_candidates=[]
    for m in members:
        if not m['constructor']: continue
        for p in m['node'].findall('param'):
            t=p.find('type')
            if t is None: continue
            names=sorted({x.text for x in t.findall('.//ref')})
            if names:
                ctor_candidates.append({'class':m['class'],'signature':m['signature'],
                    'parameter':p.findtext('declname',''),'type':''.join(t.itertext()),'referenced_types':names,
                    'interpretation':'tipo externo recebido; verificar armazenamento e possibilidade de substituição'})
    return findings,ctor_candidates


def main():
    os.chdir(ROOT); OUT.mkdir(parents=True,exist_ok=True)
    selected=scope()
    val=OUT/'validation'; val.mkdir(exist_ok=True)
    vf=['tools/ros1_metrics/fixtures/classes.hpp','tools/ros1_metrics/fixtures/classes.cpp']
    vx=doxygen(vf,val); vc,vm,vi=inventory(vx); vl=lizard(vf,val)
    vmatched,vfree,vmissing=join_methods(vc,vm,vl)
    ve,vb,vcycles=graph(vc,vi)
    py=lizard(['tools/ros1_metrics/fixtures/complexity.py'],val/'python')
    checks={'cpp_cc':[f['cc'] for f in vl if f['name'] in {'fixtures::Branch::linear','fixtures::Branch::branch'}],
            'python_cc':[f['cc'] for f in py], 'distinct_homonymous_classes':all(n in vc for n in ['fixtures::Branch','second::Branch']),
            'cycle_detected':['fixtures::A','fixtures::B'] in vcycles,
            'inheritance_detected':('fixtures::Derived','fixtures::Base','inheritance') in ve,
            'all_defined_methods_joined':len(vmatched)==len(vl) and not vmissing,
            'abstract_excluded_from_defined':not any(f['name'].endswith('::abstract') for f in vmatched)}
    dump(val/'checks.json',checks)
    assert checks['cpp_cc']==[1,2] and checks['python_cc']==[1,2] and all(v for k,v in checks.items() if k not in {'cpp_cc','python_cc'}),checks
    native=OUT/'native'; xml=doxygen(selected,native); classes,members,ids=inventory(xml)
    functions=lizard(selected,native)
    methods,free,missing=join_methods(classes,members,functions)
    table(OUT/'methods.csv',methods)
    table(OUT/'unassigned-functions.csv',free)
    table(OUT/'defined-methods-unmeasured.csv',missing,fields=['class','name','signature','declaration_file','declaration_line','body_file','body_start','body_end','virtual','static','constructor'])
    table(OUT/'declared-methods.csv',({k:v for k,v in m.items() if k!='node'} for m in members))
    if missing: raise RuntimeError('Defined methods missing Lizard measurements; inspect report')
    edges,broad,cycles=graph(classes,ids)
    table(OUT/'class-relations.csv',({'source':a,'target':b,'relation':r} for a,b,r in sorted(edges)))
    table(OUT/'internal-type-dependencies.csv',({'source':a,'target':b} for a,b in sorted(broad)))
    dump(OUT/'class-cycles.json',{'algorithm':'networkx.strongly_connected_components','nodes':len(classes),
        'distinct_directed_edges':len({(a,b) for a,b,_ in edges}),'relations':'inheritance and own attribute types from Doxygen XML; self relations excluded',
        'cyclic_components':cycles,'self_relations_excluded':True})
    data=semgrep(selected,native)
    findings,candidates=isolation(data,methods,classes,members)
    table(OUT/'isolation-findings.csv',findings)
    table(OUT/'constructor-parameter-candidates.csv',candidates)
    rows=[]
    inheritance=nx.DiGraph();inheritance.add_nodes_from(classes)
    inheritance.add_edges_from((a,b) for a,c in classes.items() for b in c['bases'] if b in classes)
    ctor_effect_classes={f['class'] for f in findings if f['phase']=='constructor_direct'}
    for name,c in sorted(classes.items()):
        ms=[m for m in methods if m['class']==name]; ds=[m for m in members if m['class']==name]
        fs=[f for f in findings if f['class']==name]
        unknown=[m for m in ds if not m['body_start'] and m['virtual']!='pure-virtual']
        rows.append({'class':name,'file':c['file'],'line':c['line'],'bases':c['bases'],
            'nom_declared':len(ds),'nom_defined_measured':len(ms),'pure_virtual_declarations':sum(m['virtual']=='pure-virtual' for m in ds),
            'other_declarations_without_body':len(unknown),'wmc_sum_lizard_cc_defined':sum(m['cc'] for m in ms),
            'max_method_cc':max([m['cc'] for m in ms],default=0),'lcom4':None,'lcom4_status':'ROSE_unavailable_not_measured',
            'cbo_pyscn_equivalent':None,'outgoing_internal_type_dependencies':len({b for a,b in broad if a==name}),
            'in_detected_cycle':any(name in cc for cc in cycles),'infrastructure_categories':sorted({f['category'] for f in fs}),
            'direct_constructor_categories':sorted({f['category'] for f in fs if f['phase']=='constructor_direct'}),
            'setup_categories':sorted({f['category'] for f in fs if f['phase']=='setup'}),
            'ancestor_with_constructor_findings':sorted(nx.descendants(inheritance,name)&ctor_effect_classes),
            'constructor_parameter_candidates':sum(x['class']==name for x in candidates)})
    table(OUT/'classes.csv',rows)
    dump(OUT/'summary.json',{'files':len(selected),'classes':len(classes),'declared_methods':len(members),'defined_measured_methods':len(methods),
        'max_wmc':max(r['wmc_sum_lizard_cc_defined'] for r in rows),'max_cc':max(m['cc'] for m in methods),
        'methods_cc_gt_10':sum(m['cc']>10 for m in methods),'class_cyclic_components':cycles,
        'native_semgrep_occurrences':len(findings),'semgrep_categories':dict(Counter(f['category'] for f in findings)),
        'semgrep_classes_with_direct_findings':len({f['class'] for f in findings if f['class']}),
        'semgrep_findings_outside_classes':sum(not f['class'] for f in findings), 'semgrep_errors':data.get('errors'),
        'lcom4_status':'not_measured_ROSE_unavailable','cbo_equivalence':'not_established','testability_score':None})
    native_classes=[{k:v for k,v in c.items() if k!='node'} for c in classes.values()]
    dump(OUT/'class-inventory.json',native_classes)
    tools={p:importlib.metadata.version(p) for p in ['lizard','semgrep','networkx','Jinja2','Pygments']}
    tools['doxygen']=run([DOXYGEN,'--version'],OUT/'doxygen-version.txt').strip()
    tools['semgrep-core']=run([HERE/'semgrep-core','-version'],OUT/'semgrep-core-version.txt').strip()
    dump(OUT/'metadata.json',{'timestamp_utc':datetime.now(timezone.utc).isoformat(),'git_root':str(ROOT),
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        'working_tree_status':subprocess.check_output(['git','status','--short'],text=True),
        'python':sys.version,'tools':tools,'reference_ros2_commit':'e06541e63788439303ec6e5aa3ed03178f06dda4',
        'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in selected},
        'analysis_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(HERE.rglob('*')) if p.is_file() and '__pycache__' not in str(p)},
        'runtime_environment':{k:ENV[k] for k in ['PATH','LD_LIBRARY_PATH','SEMGREP_SETTINGS_FILE','SEMGREP_SEND_METRICS','SEMGREP_ENABLE_VERSION_CHECK'] if k in ENV},
        'no_ros_execution':True})
    dump(OUT/'commands.json',COMMANDS)
    print(json.dumps(json.loads((OUT/'summary.json').read_text()),indent=2))


if __name__=='__main__': main()
