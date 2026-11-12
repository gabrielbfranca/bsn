#!/usr/bin/env python3
"""Render existing results; no source analysis or new metric computation."""
import csv
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'reports/ros1_static'


def load_csv(p):
    return list(csv.DictReader(p.open()))


def markdown_table(columns,rows):
    return '\n'.join(['| '+' | '.join(columns)+' |','| '+' | '.join('---' for _ in columns)+' |']+
                     ['| '+' | '.join(str(x).replace('|','\\|').replace('\n',' ') for x in row)+' |' for row in rows])


def main():
    summary=json.loads((OUT/'summary.json').read_text())
    classes=load_csv(OUT/'classes.csv'); methods=load_csv(OUT/'methods.csv')
    findings=load_csv(OUT/'isolation-findings.csv')
    cycles=json.loads((OUT/'class-cycles.json').read_text())
    ros2=load_csv(OUT/'reference_ros2/reports/class_modularity/classes.csv')
    metrics=[]
    for c in classes:
        metrics.append([c['class'],c['nom_declared'],c['nom_defined_measured'],c['wmc_sum_lizard_cc_defined'],
                        c['max_method_cc'],c['outgoing_internal_type_dependencies'],c.get('lcom4') or '—',c.get('cbo_cccc') or '—'])
    high=sorted(methods,key=lambda r:int(r['cc']),reverse=True)[:15]
    high_table=markdown_table(['Classe','Método','CC','Arquivo:linha'],[
        [m['class'],m['name'],m['cc'],m['file']+':'+m['start']] for m in high])
    infra_table=markdown_table(['Classe','Categorias diretas','Fase setUp','Efeito direto no construtor','Ancestral com efeito no construtor'],[
        [c['class'],', '.join(json.loads(c['infrastructure_categories'])),', '.join(json.loads(c['setup_categories'])),
         ', '.join(json.loads(c['direct_constructor_categories'])),', '.join(json.loads(c['ancestor_with_constructor_findings']))]
        for c in classes if json.loads(c['infrastructure_categories']) or json.loads(c['ancestor_with_constructor_findings'])])
    comparisons=[]
    for a,b in [('DataAccess','DataAccess'),('ReliabilityEngine','ReliabilityEngine'),('Engine','Engine'),
                ('Controller','Controller'),('Enactor','Enactor'),('Logger','Logger'),('PatientModule','Patient'),
                ('bsn::goalmodel::Context','Context'),('bsn::goalmodel::Node','Node'),('bsn::goalmodel::GoalTree','GoalTree'),
                ('bsn::goalmodel::LeafTask','LeafTask'),('bsn::goalmodel::Property','Property'),
                ('bsn::model::Formula','Formula'),('bsn::resource::Battery','Battery')]:
        x=next(c for c in classes if c['class']==a)
        y=next(c for c in ros2 if c['class']==b)
        comparisons.append([a+' → '+b,x['nom_defined_measured'],y['nom_declared'],x['wmc_sum_lizard_cc_defined'],
                            y['wmc_sum_lizard_cc'],x['max_method_cc'],y['max_method_cc_lizard']])
    comparison_table=markdown_table(['Responsabilidade/classe ROS 1 → ROS 2','NOM definido ROS 1','NOM ROS 2','WMC ROS 1','WMC ROS 2','CC máx. ROS 1','CC máx. ROS 2'],comparisons)
    pure=sum(int(c['pure_virtual_declarations']) for c in classes)
    no_body=sum(int(c['other_declarations_without_body']) for c in classes)
    text=f'''# Modularidade e indícios de isolamento — BSN ROS 1 × ROS 2

**Coleta estática concluída com Lizard, cpplcom, CCCC, Doxygen e Semgrep.**
Interpretação e tabelas derivadas produzidas pelo assistente; saídas nativas
ficam separadas. As variantes C++ de LCOM4 e CBO foram medidas e suas diferenças
em relação às ferramentas Python são explicitadas, sem presumir equivalência.

A coleta ROS 1 selecionou {summary['files']} arquivos próprios C++, inventariou
{summary['classes']} classes/structs e mediu {summary['defined_measured_methods']}
métodos com corpo usando Lizard 1.26.0. O maior WMC é {summary['max_wmc']} em
CostEngine; a maior CC de método é {summary['max_cc']}. Há
{summary['methods_cc_gt_10']} métodos com CC > 10. Esses valores apontam onde
revisar responsabilidades e decisões, sem provar qualidade ou testabilidade.

## Evidência original e reprodução

- [Lizard HTML](native/lizard-native.html), [CSV](native/lizard-native.csv) e [texto](native/lizard-native.txt): CC original por função/método.
- [Doxygen XML](native/doxygen/xml/index.xml): inventário e referências nativas; [configuração](native/Doxyfile) e [avisos](native/doxygen.stdout.stderr).
- [Semgrep JSON](native/semgrep-native.json), [SARIF](native/semgrep-native.sarif) e [texto](native/semgrep-native.text).
- [cpplcom JSON](native/cpplcom-native.json) e [texto com grupos](native/cpplcom-native.txt): LCOM4 original por classe.
- [CCCC HTML](native/cccc/cccc.html) e [XML](native/cccc/cccc.xml): CBO e outras métricas nativas; WMC/NOM comparativos continuam sendo os da agregação Lizard/Doxygen.
- [Resultados por classe](classes.csv), [por método](methods.csv), [achados de isolamento](isolation-findings.csv) e [grafo derivado](class-relations.csv).
- [Versões e hashes](metadata.json), [comandos](commands.json), [escopo](files.txt), [exclusões](excluded.csv) e [validação](validation/checks.json).
- [Instruções de reprodução e definições](../../tools/ros1_metrics/README.md).

A raiz Git é `/workspaces/bsn_`. A coleta mede o estado local dos fontes, que já
tinha modificações; o HEAD sozinho não identifica esse conteúdo. Não houve
execução ROS, compilação da BSN, commit, push ou edição dos fontes de produção.

A referência ROS 2 foi consultada na branch `metrics`, commit
`e06541e63788439303ec6e5aa3ed03178f06dda4`. Foram preservados os
[relatórios e tabelas consultados](reference_ros2/sources.json), incluindo
[LCOM4](reference_ros2/reports/lcom4/report.md),
[modularidade](reference_ros2/reports/class_modularity/report.md) e
[isolamento por classe](reference_ros2/reports/isolation/classes.csv).

## LCOM4: medição com cpplcom

O cpplcom 0.1.0, commit `3b8ce6150d142da0e58bf7968144f26a5854b647`, analisou
os corpos com Clang/libclang 18.1.8 e gerou JSON e texto nativos. Não houve
modificação do analisador nem implementação local de coesão. ROSE foi o
candidato inicial, substituído pelo cpplcom por disponibilidade operacional.

Foram reportadas {summary['lcom4_classes']} classes com pelo menos dois métodos
elegíveis, distribuição bruta {summary['lcom4_raw_distribution']}. Os casos
com menos de dois participantes são omitidos pela ferramenta e identificados
na tabela; não receberam valor fictício. O parsing final não reportou erros.
[Valores, grupos, definição e validação](lcom4/report.md).

Os exemplos de estado compartilhado, estado separado e ponte por chamada
produziram 1, 2 e 1. O cpplcom mantém vazios, getters e métodos sem estado no
LCOM4 bruto; suas colunas BARE/jobs são ajustes distintos. Há uma divergência
no teste upstream de member template com Clang 18, preservada no relatório.
Nos fontes próprios, os dois métodos template são dos structs comp com um
participante cada, fora das classes reportadas de LCOM4.

DataAccess tem bruto 2, sendo tearDown vazio o componente separado no C++;
no ROS 2, o grupo separado era _setup_ros_interfaces. ReliabilityEngine tem bruto 3, incluindo grupos
isolados tearDown/get_prefix. Os DTOs de mensagens têm bruto 6, composto de
acessores independentes. Não comparar a distribuição com 29 valores 1 e dois
valores 2 do subconjunto filtrado do pyscn sem igualar a política de métodos.
O relatório detalhado propõe esse critério comum e preserva os grupos.

## WMC, NOM e CC: resultados por classe

WMC é a soma dos CC originais do Lizard para métodos definidos diretamente na
classe. Inclui construtores, destrutores, operadores e estáticos com corpo;
exclui herdados, funções livres e funções friend livres. A unidade de associação
é a definição, unida à declaração sem duplicar header e fonte.

Há {summary['declared_methods']} declarações: {summary['defined_measured_methods']}
corpos medidos, {pure} virtuais puros e {no_body} outras declarações sem corpo
reconhecido. Não atribuímos complexidade fictícia a essas declarações. Assim,
NOM declarado C++ e NOM definido não são intercambiáveis, nem o total declarado
deve ser comparado diretamente aos 318 métodos Python com corpo.

`Deps internas` abaixo é um indicador derivado de tipos internos em atributos,
assinaturas e herança, **não CBO**. As duas últimas colunas extraem LCOM4 bruto
do cpplcom e CBO do CCCC. `—` significa que a ferramenta não reportou um valor
individual; o motivo está em `classes.csv`. Nenhuma coluna afirma equivalência
automática com pyscn.

{markdown_table(['Classe','NOM declarado','NOM definido medido','WMC definido','CC máxima','Deps internas','LCOM4 cpplcom','CBO CCCC'],metrics)}

O Doxygen agrupou os dois structs globais `comp`; as identidades derivadas
incluem o arquivo para distingui-los. Dois construtores com aliases de
`std::array` foram associados pelo nome qualificado e aridade única do Lizard,
com marcação em `methods.csv`. Os avisos originais permanecem preservados.

### Métodos que mais concentram decisões

{high_table}

CostEngine e ReliabilityEngine são prioridades de inspeção. DataAccess também
reúne várias responsabilidades: assinaturas e callbacks ROS, manutenção de
buffers, consulta de dados, cálculo de custo/confiabilidade e persistência.
A existência de um estado comum não prova que essas responsabilidades devam
permanecer na mesma classe.

Parte do processamento ROS 1 aparece em funções livres, como
`bsn::processor::data_fuse` (CC 13), fora do WMC de classes. Isso importa ao
comparar com responsabilidades implementadas como classes na versão Python.
[Funções sem associação a classe](unassigned-functions.csv) permanecem visíveis.

## Acoplamento e ciclos

CCCC 3.1.4 produziu HTML, XML, banco e opções nativos. Seu CBO é a soma de
clientes e fornecedores reconhecidos no banco de módulos. A validação
confirmou Supplier=2, Consumer=1, SignatureUser=1, Standalone=0 em definições
no mesmo arquivo. Foram associados valores a {summary['cbo_cccc_classes']}
classes próprias; o máximo é {summary['cbo_cccc_max']}.

O parser antigo do CCCC rejeitou {summary['cccc_rejected_lines']} linhas,
registradas no relatório nativo. Seus nomes não qualificados só foram
associados quando únicos no inventário. Os dois comp não recebem valor
individual. Esses limites afetam a cobertura das relações; os números são
preservados como variante CCCC, sem tratá-los como um CBO semântico completo.
[Definição, limites e tabela por classe](coupling/report.md).

O pyscn exclui classes pares do mesmo arquivo e conta usos externos de saída,
sem somar os clientes como faz o CCCC. Portanto, não comparar diretamente
esta variante com a média ROS 2 de 2,8936 ou máximo de 15. O indicador Doxygen
de dependências internas permanece separado do CBO nativo.

O critério comum proposto é contar tipos de produção distintos usados por
herança, atributos, parâmetros e retornos, incluindo classes no mesmo arquivo
e excluindo infraestrutura em uma camada separada. Exige nova coleta nas duas
linguagens com as mesmas relações; o indicador C++ atual é só uma primeira
extração verificável, não uma comparação concluída.

O grafo de herança e tipos de atributos contém {cycles['nodes']} classes e
{cycles['distinct_directed_edges']} pares dirigidos distintos. NetworkX não
detectou componentes com mais de uma classe. O exemplo A↔B foi detectado antes
da coleta. [Arestas](class-relations.csv) e [resultado dos componentes](class-cycles.json)
permitem conferir a conclusão. Autorrelações foram excluídas.

ROS 2 também não apresentou ciclos no grafo parcial Pyreverse (47 classes,
37 relações), mas as inferências são diferentes. Pode-se dizer que ambos os
grafos observados não revelaram ciclos; não que tenham cobertura equivalente
ou que todas as dependências do sistema sejam acíclicas. Não foram medidos
ciclos entre arquivos, includes ou pacotes nesta coleta.

## Indícios de isolamento para testes

Semgrep CE 1.185.0 executou 18 regras locais em {summary['files']} arquivos, sem
erros reportados, com aproximadamente 100% das linhas analisadas segundo a
saída da ferramenta. Os testes nativos passaram: 11/11 regras originais e
7/7 adicionais. Isso valida os exemplos fornecidos, não cobertura de toda a
linguagem ou de todos os padrões de infraestrutura.

Foram preservadas {summary['native_semgrep_occurrences']} ocorrências, com
achados diretos em {summary['semgrep_classes_with_direct_findings']} classes e
{summary['semgrep_findings_outside_classes']} fora delas. Esses números são um
inventário local, **não uma pontuação de testabilidade e não são comparados
aos 251 achados ROS 2**. Regras podem se sobrepor.

{infra_table}

`arch::ROSComponent` chama `ros::init` e instala tratadores de sinal em seu
construtor. Subclasses herdam esse contrato mesmo quando seus construtores
aparentam apenas inicializar campos. A coluna de ancestral registra esse
indício sem duplicar os achados da base. Ela deriva do grafo de herança, sem
substituir uma análise interprocedural completa.

Muitas operações estão em `setUp`, não diretamente no construtor: a fase deve
ser considerada ao comparar com `__init__`, `on_configure` e helpers Python.
O próprio `DataAccess::setUp` prepara arquivos e recursos ROS, enquanto métodos
de persistência e relógio mantêm efeitos posteriores. A função livre
`fetch_formula` também consulta caminho de pacote e arquivo.

Classes de domínio, como as de `bsn::goalmodel`, não apresentaram achados
diretos nessas regras; isso sugere um ponto de partida para testes isolados,
sem provar ausência de dependências indiretas. Referências recebidas por
construtores são listadas em [candidatos](constructor-parameter-candidates.csv).
A lista inclui tipos de valor/cópia e exige revisão de armazenamento e uso;
não representa automaticamente interfaces substituíveis.

As regras adicionais cobrem formas reais antes omitidas: `fstream`, abertura
por nomes `fp/file/stream/input/output`, `high_resolution_clock`, parâmetros,
espera por receptor de nome contendo `rate`, sinais, caminho ROS e diretórios.
Heurísticas por nome podem produzir falsos positivos ou negativos; `call`
continua dependente do nome do receptor. Não há cobertura completa de macros,
aliases, ponteiros, fluxos de chamadas, todos os construtores de recursos ou
colaboradores fornecidos por factories. Ausência de achado não prova pureza.

## Comparação por responsabilidades

A tabela abaixo compara a complexidade **observada nos corpos**, usando a
mesma versão do Lizard e a mesma soma por classe. A validação simples CC 1/2
nas duas linguagens passou. Diferenças de funcionalidades, métodos especiais,
herança e organização impedem interpretar toda diferença como efeito de ROS 2.

{comparison_table}

ReliabilityEngine diminui de WMC 74 para 67 e sua maior CC de 39 para 16 nesta
correspondência. DataAccess passa de WMC 70 para 99, com CC máxima 17 em ambas.
São observações locais sobre o código medido; não sustentam uma afirmação
geral de que a migração melhora ou piora modularidade.

Há correspondências de uma classe antiga para várias novas:

| Responsabilidade ROS 1 | Responsabilidades/classes ROS 2 | Consequência |
| --- | --- | --- |
| Sensor + G3T1_1…G3T1_6 + SensorConfiguration e apoio de processamento | Sensor, ConfigManager, DataProcessor, RiskManager, RiskEvaluator e PublisherManager | Comparar aquisição, configuração, risco e publicação separadamente; NOM de Sensor sozinho omite trabalho distribuído no ROS 1 |
| CentralHub + G4T1 + processamento livre | CentralHub, ConfigManager, SensorDataHandler, DataFusionEngine, RiskAnalyzer, PublisherManager e Visualizer | Seguir a responsabilidade, inclusive funções livres, antes de agregar complexidade |
| ROSComponent + Component/Effector/Probe | RosComponent, LifecycleManager, BatteryManager, AdaptationHandler e bases de configuração/publicação | Investigar ciclo de vida, herança e efeitos indiretos; não comparar apenas classes de mesmo nome |
| CostEngine | Sem par direto identificado nos relatórios ROS 2 consultados | Não usar seu WMC como comparação pareada com ReliabilityEngine |

Esse mapeamento é uma hipótese de correspondência para inspeção estática,
baseada nos fontes ROS 1 e nos relatórios/nomes de classes ROS 2; não valida
equivalência funcional de todos os componentes.

## Quais comparações são válidas

| Item | Situação ROS 1 × ROS 2 | Limite/critério |
| --- | --- | --- |
| CC Lizard por método | Comparável sob a definição da ferramenta | Mesma versão 1.26.0; selecionar responsabilidades e construções equivalentes; macros/templates não equivalem a expansão compilada |
| WMC dos corpos medidos | Comparação condicionada ao escopo | Mesma soma de CC; conferir métodos especiais, abstrações, herança e funções livres |
| NOM | Definidos podem ser comparados com ressalvas | Separar 422 corpos C++ de 498 declarações; Python tem 318 métodos diretamente declarados com corpo |
| LCOM4 | Variantes medidas; comparar grupos com ressalvas | cpplcom bruto e pyscn filtrado têm populações diferentes; validar política comum antes de comparar distribuições |
| CBO | Variante CCCC medida; sem equivalência direta ao pyscn | CCCC soma clientes/fornecedores, inclui pares do mesmo arquivo e tem rejeições de parsing |
| Ciclos entre classes | Conclusão descritiva dos grafos observados | Zero componentes cíclicos em ambos; Doxygen e Pyreverse têm cobertura/inferências diferentes |
| Isolamento Semgrep | Comparação qualitativa por responsabilidade e fase | Mesmo conceito de presença de efeitos, mas regras locais distintas; não comparar totais brutos |
| Testabilidade geral | Não demonstrada | Coesão, acoplamento e ausência de achados não provam facilidade de teste; não houve execução nem medida de cobertura |

## Condições para ampliar a comparação

1. Uma diferença numérica de LCOM4 entre linguagens exige a mesma população
   de métodos e política de atributos/chamadas. Os valores nativos atuais
   permanecem como variantes distintas, acompanhadas de seus grupos.
2. Uma diferença numérica de CBO exige uma definição comum de direções e
   tipos incluídos; o indicador interno proposto requer nova medição ROS 2.
3. Uma conclusão mais ampla de testabilidade exigiria avaliar caminhos
   indiretos, construção de membros e substituição de colaboradores além
   das regras locais. Os resultados atuais não justificam uma nota global.
'''
    (OUT/'report.md').write_text(text)
    (OUT/'comparison-by-responsibility.csv').write_text('')
    with (OUT/'comparison-by-responsibility.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['mapping','nom_ros1_defined','nom_ros2','wmc_ros1','wmc_ros2','max_cc_ros1','max_cc_ros2']);w.writerows(comparisons)
    # Record documentation state without rerunning native analyzers.
    meta=json.loads((OUT/'metadata.json').read_text())
    meta['documentation_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [OUT/'report.md',OUT/'lcom4/report.md',ROOT/'tools/ros1_metrics/README.md',Path(__file__)]}
    (OUT/'metadata.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n')
    print(OUT/'report.md')


if __name__=='__main__': main()
