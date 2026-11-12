#!/usr/bin/env python3
"""Join native cpplcom/CCCC values to the class inventory. No metric algorithms."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'reports/ros1_static'


def md(columns,rows):
    return '\n'.join(['| '+' | '.join(columns)+' |','| '+' | '.join('---' for _ in columns)+' |']+
                     ['| '+' | '.join(str(x).replace('|','\\|') for x in r)+' |' for r in rows])


def main():
    status=json.loads((OUT/'lcom4/parse-status.json').read_text());assert status['success'],status
    fixture=ET.parse(OUT/'validation/cccc/cccc.xml').getroot()
    fvalues={m.findtext('name'):int(m.find('coupling_between_objects').get('value')) for m in fixture.findall('./oo_design/module')}
    fchecks={n:fvalues.get(n)==v for n,v in {'Consumer':1,'SignatureUser':1,'Supplier':2,'Standalone':0}.items()}
    assert all(fchecks.values()),fchecks
    (OUT/'validation/cccc/checks.json').write_text(json.dumps(fchecks,indent=2)+'\n')
    values=json.loads((OUT/'native/cpplcom-native.json').read_text())
    raw={r['class']:r for r in values}
    assert len(raw)==len(values),'ambiguous native class names'
    tree=ET.parse(OUT/'native/cccc/cccc.xml').getroot()
    cbo={m.findtext('name'):int(m.find('coupling_between_objects').get('value')) for m in tree.findall('./oo_design/module')}
    rejected=int(tree.find('./project_summary/rejected_lines_of_code').get('value'))
    with (OUT/'classes.csv').open() as f: classes=list(csv.DictReader(f))
    counts=Counter(c['class'].rsplit('::',1)[-1] for c in classes)
    for c in classes:
        name=c['class'];n=name.rsplit('::',1)[-1];v=raw.get(name)
        c.update(lcom4=v['lcom4'] if v else '',lcom4_status='measured_cpplcom' if v else 'not_reported_fewer_than_two_eligible_methods',
                 lcom4_participating_methods=v['methods'] if v else '',lcom4_bare=v['bare'] if v else '',
                 cpplcom_jobs=v['jobs'] if v else '',lcom4_components=json.dumps(v['components'],ensure_ascii=False) if v else '',
                 cbo_cccc=cbo.get(n,'') if counts[n]==1 else '',
                 cbo_cccc_status='measured_parser_limited' if counts[n]==1 and n in cbo else 'not_reported_or_ambiguous',
                 cbo_pyscn_equivalent='')
    with (OUT/'classes.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,list(classes[0]));w.writeheader();w.writerows(classes)
    dist=dict(sorted(Counter(r['lcom4'] for r in values).items()))
    measured_cbo=[c for c in classes if c['cbo_cccc']!='']
    summary=json.loads((OUT/'summary.json').read_text())
    summary.update(lcom4_status='measured_cpplcom',lcom4_classes=len(values),lcom4_raw_distribution=dist,
        lcom4_classes_not_reported=[c['class'] for c in classes if c['lcom4']==''],
        cbo_status='measured_CCCC_variant_with_parser_limits',cbo_cccc_classes=len(measured_cbo),
        cbo_cccc_max=max(c['cbo_cccc'] for c in measured_cbo),cccc_rejected_lines=rejected)
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    ltable=md(['Classe','LCOM4 bruto','Participantes','BARE','Jobs (outra medida)'],[
        [r['class'],r['lcom4'],r['methods'],r['bare'],r['jobs']] for r in values])
    groups='\n\n'.join('### '+r['class']+'\n\n'+'\n'.join('- `'+', '.join(g)+'`' for g in r['components']) for r in values)
    lreport=f'''# LCOM4 C++ — cpplcom

Medição concluída com ferramenta externa existente: **cpplcom 0.1.0**, commit
`3b8ce6150d142da0e58bf7968144f26a5854b647`, libclang/Clang 18.1.8.
O código do analisador não foi modificado. O script local prepara entradas,
invoca o CLI e associa os valores originais ao inventário de classes.

## Relatórios nativos

- [JSON nativo completo](../native/cpplcom-native.json).
- [Texto nativo com todos os grupos reportados](../native/cpplcom-native.txt),
  gerado com `--threshold 0`, e [stderr](../native/cpplcom-native.txt.stderr).
- [Banco de comandos de parsing](compile_commands.json),
  [estado do parsing](parse-status.json), [comandos](../oo-commands.json)
  e [versões/hashes](../oo-metadata.json).
- [Definição original da ferramenta](../reference_tools/cpplcom-README.md).

O analisador apresentou {len(values)} classes com pelo menos dois métodos
elegíveis, distribuição bruta {dist}. As outras classes são explicitamente
identificadas em `classes.csv`, sem lhes atribuir coesão fictícia.

## Definição e participação

LCOM4 bruto conta componentes de métodos unidos por campo próprio compartilhado
ou chamada entre métodos. Entram funções membro próprias, não estáticas, com
corpo; saem construtores, destrutores, atribuição de cópia/movimento, funções
defaulted/deleted e virtuais puros sem corpo. Há tratamento especial para
estáticos que recebem o próprio objeto explicitamente e para chamadas de base
que alcançam overrides virtuais próprios. Campos herdados não unem métodos.

Métodos vazios, sem estado e acessores permanecem no **LCOM4 bruto**. A coluna
`jobs` é um ajuste próprio da ferramenta que desconta componentes unitários
classificados como `BARE`; não foi usada como sinônimo de LCOM4 nem de número
comprovado de responsabilidades.

## Validação e limites observados

Os casos básicos produziram 1 (estado compartilhado), 2 (estado separado) e 1
(ponte por chamada). Métodos sem estado e acessores isolados deram 2 no bruto;
OnlyConstructor não foi reportada. Declaração header e definição fonte foram
unidas. [Resultados nativos dos exemplos](../validation/cpplcom/native.json) e
[verificações](../validation/cpplcom/checks.json).

Também foram executados os dois testes de integração originais da ferramenta:
o teste de threshold passou; o teste agregado de scores divergiu no caso de
chamada a member template em `Tmpl` com Clang 18. O teste esperava LCOM4=1 e
obteve 2. [Log original](../validation/cpplcom/upstream-tests.txt.stderr).
Nos fontes próprios, templates de métodos aparecem somente nos dois structs
`comp`, ambos com um único método elegível e, portanto, fora da população
reportada de LCOM4. A discrepância permanece uma limitação da ferramenta;
não se afirma validação geral de templates.

O parsing final terminou sem erros reportados. A entrada legada
`apps/enactor.cpp`, que instancia classe abstrata e não pertence aos alvos
CMake, foi excluída apenas do banco de unidades de tradução. A implementação
de Enactor permanece incluída; Lizard/Semgrep mantêm o escopo inicial.
[Decisão](translation-unit-exclusions.json) e
[saída da tentativa com o arquivo legado](attempt-with-legacy-entrypoint/cpplcom-native.txt.stderr).
Os headers de mensagens foram gerados com **gencpp do ROS**, sem stubs,
compilar a BSN ou executar nós. Eles são dependências de parsing, não população
medida. Cada arquivo C++ próprio é analisado uma vez; os métodos são unidos
pelo identificador semântico da ferramenta.

## Comparação com ROS 2

Não comparar a distribuição bruta ao resultado filtrado do pyscn: as exclusões
não são iguais. Por exemplo, os DTOs de mensagens C++ têm LCOM4=6 por acessores
de seis campos independentes, enquanto o pyscn pode excluir seus métodos.
`tearDown` vazio também pode criar um componente isolado no C++.

DataAccess tem LCOM4 bruto 2 nas duas análises, mas isso não demonstra grupos
equivalentes: no C++ o componente separado é `tearDown` vazio; no ROS 2, o grupo
separado era `_setup_ros_interfaces`. ReliabilityEngine tem bruto 3 no C++,
com componentes `tearDown` e `get_prefix` separados do grupo principal. Essas
diferenças precisam ser consideradas antes de atribuir melhora à migração.

Critério comum: fixar nas duas linguagens a mesma população de métodos,
atributos próprios/herdados e regras de chamada; validar construtores,
estáticos, vazios, getters e casos com menos de dois participantes. Se as
ferramentas não permitirem essa política comum, preservar as variantes e
comparar a composição dos grupos, sem uma diferença numérica de coesão.

## Valores por classe (tabela derivada do JSON)

{ltable}

## Grupos originais, reorganizados por classe

{groups}

## ROSE-LCOM-Tools

ROSE foi o candidato inicial e sua configuração foi registrada, mas o runtime
ROSE não está instalado. Ele foi substituído nesta coleta pelo cpplcom;
`status.json` e `configure.*` documentam somente a avaliação daquele candidato,
não a ferramenta que produziu os valores finais.
'''
    (OUT/'lcom4/report.md').write_text(lreport)
    ctable=md(['Classe qualificada','Módulo CCCC','CBO nativo'],[
        [c['class'],c['class'].rsplit('::',1)[-1],c['cbo_cccc']] for c in measured_cbo])
    cpath=OUT/'coupling';cpath.mkdir(exist_ok=True)
    (cpath/'report.md').write_text(f'''# CBO — variante CCCC

Coleta com **CCCC 3.1.4**, pacote Ubuntu `1:3.1.4-9`. Relatórios nativos:
[HTML](../native/cccc/cccc.html), [XML](../native/cccc/cccc.xml),
[banco de análise](../native/cccc/cccc.db), [opções](../native/cccc/cccc.opt),
[stdout](../native/cccc/native.txt) e [diagnósticos](../native/cccc/native.txt.stderr).
O código da ferramenta não foi modificado.

## Definição validada

O CCCC calcula CBO como **quantidade de clientes + quantidade de fornecedores**
no seu banco de módulos, não como apenas dependências de saída de classes.
Inclui relações entre definições no mesmo arquivo e módulos externos/tipos
reconhecidos pelo parser. Relações recíprocas podem contribuir nos dois sentidos.
O banco não distingue todos os tipos de C++ de maneira semântica moderna.

No exemplo nativo, Consumer e SignatureUser usam Supplier no mesmo arquivo:
obtiveram CBO 1 cada, Supplier obteve 2 (clientes), Standalone obteve 0.
[XML do exemplo](../validation/cccc/cccc.xml) e
[checagem dos valores](../validation/cccc/checks.json).

## Cobertura e limites

Foram extraídos valores para {len(measured_cbo)} das {len(classes)} identidades
do inventário. Os dois structs `comp` não têm valor individual reportado.
O CCCC usa nomes não qualificados: a associação só foi feita quando o nome
curto é único no inventário. Identidades ambíguas não recebem um número.

O relatório nativo registra **{rejected} linhas rejeitadas**. Trata-se de um
parser antigo, com limitações em macros, templates e sintaxe C++11. Os valores
foram preservados como uma variante com cobertura limitada; ausência de
relações pode ser falta de reconhecimento. Não usar o valor como uma medida
semântica completa ou comparar seu total com o grafo Doxygen.

O máximo entre as classes próprias associadas é
{max(c['cbo_cccc'] for c in measured_cbo)}; os módulos adicionais do XML nativo,
como `string`, não foram acrescentados à população de classes de produção.
Esta tabela somente extrai `coupling_between_objects` do XML; não recalcula CBO.

## Comparabilidade ROS 1 × ROS 2

**Não há equivalência numérica direta com pyscn.** O pyscn exclui classes pares
do mesmo arquivo e conta usos externos reconhecidos, sem somar seus clientes
como faz o CCCC. CBO=11 no CCCC e CBO=15 no pyscn, por exemplo, não constituem
uma redução de quatro dependências.

Critério comum proposto: tipos de produção distintos usados em herança,
atributos, parâmetros e retornos, com relações internas e de infraestrutura
separadas, incluindo classes do mesmo arquivo em ambas as linguagens. O
indicador Doxygen `outgoing_internal_type_dependencies` continua separado e
não é renomeado para CBO. Uma comparação com essa política exige medi-la
também no ROS 2; os resultados pyscn originais permanecem preservados.

## Valores extraídos do XML nativo

{ctable}
''')
    manifest={'cpplcom_json_sha256':hashlib.sha256((OUT/'native/cpplcom-native.json').read_bytes()).hexdigest(),
        'cccc_xml_sha256':hashlib.sha256((OUT/'native/cccc/cccc.xml').read_bytes()).hexdigest(),
        'lcom4_classes':len(values),'lcom4_distribution':dist,'cccc_classes':len(measured_cbo),'cccc_rejected_lines':rejected}
    (OUT/'oo-summary.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (OUT/'lcom4/collection-status.json').write_text(json.dumps({'tool':'cpplcom','status':'measured',
        'parse_errors':[],'classes_reported':len(values),'native_json':'../native/cpplcom-native.json',
        'validation_limits':'upstream member-template fixture differs with Clang 18'},indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
