# LCOM4 C++ — cpplcom

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

O analisador apresentou 41 classes com pelo menos dois métodos
elegíveis, distribuição bruta {1: 10, 2: 14, 3: 11, 5: 1, 6: 5}. As outras classes são explicitamente
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

| Classe | LCOM4 bruto | Participantes | BARE | Jobs (outra medida) |
| --- | --- | --- | --- | --- |
| bsn::goalmodel::LeafTask | 5 | 9 | 1 | 4 |
| bsn::goalmodel::Context | 3 | 7 | 0 | 3 |
| CostEngine | 3 | 9 | 1 | 2 |
| ReliabilityEngine | 3 | 9 | 1 | 2 |
| G3T1_1 | 3 | 6 | 1 | 2 |
| G3T1_2 | 3 | 6 | 1 | 2 |
| G3T1_3 | 3 | 6 | 1 | 2 |
| G3T1_4 | 3 | 6 | 1 | 2 |
| G3T1_5 | 3 | 6 | 1 | 2 |
| G3T1_6 | 3 | 6 | 1 | 2 |
| bsn::goalmodel::GoalTree | 2 | 10 | 0 | 2 |
| bsn::goalmodel::Property | 2 | 5 | 0 | 2 |
| arch::ROSComponentDescriptor | 2 | 4 | 0 | 2 |
| AdaptationMessage | 6 | 6 | 6 | 1 |
| EnergyStatusMessage | 6 | 6 | 6 | 1 |
| EventMessage | 6 | 6 | 6 | 1 |
| StatusMessage | 6 | 6 | 6 | 1 |
| UncertaintyMessage | 6 | 6 | 6 | 1 |
| Injector | 3 | 7 | 2 | 1 |
| Enactor | 3 | 6 | 2 | 1 |
| DataAccess | 2 | 17 | 1 | 1 |
| Logger | 2 | 9 | 1 | 1 |
| arch::target_system::Component | 2 | 9 | 1 | 1 |
| Engine | 2 | 8 | 1 | 1 |
| G4T1 | 2 | 7 | 1 | 1 |
| arch::target_system::Probe | 2 | 6 | 1 | 1 |
| ParamAdapter | 2 | 5 | 1 | 1 |
| PatientModule | 2 | 5 | 1 | 1 |
| arch::target_system::Effector | 2 | 2 | 2 | 1 |
| bsn::goalmodel::Task | 2 | 2 | 2 | 1 |
| bsn::time::TimeData | 2 | 2 | 2 | 1 |
| bsn::configuration::SensorConfiguration | 1 | 21 | 0 | 1 |
| bsn::goalmodel::Node | 1 | 11 | 0 | 1 |
| bsn::resource::Battery | 1 | 11 | 0 | 1 |
| Sensor | 1 | 9 | 0 | 1 |
| CentralHub | 1 | 8 | 0 | 1 |
| bsn::range::Range | 1 | 8 | 0 | 1 |
| bsn::model::Formula | 1 | 7 | 0 | 1 |
| bsn::filters::MovingAverage | 1 | 6 | 0 | 1 |
| Controller | 1 | 4 | 0 | 1 |
| bsn::generator::DataGenerator | 1 | 4 | 0 | 1 |

## Grupos originais, reorganizados por classe

### bsn::goalmodel::LeafTask

- `getContext, setContext`
- `getCost, setCost`
- `getReliability, setReliability`
- `getFrequency, setFrequency`
- `addChild`

### bsn::goalmodel::Context

- `getID, operator==, setID`
- `getDescription, setDescription`
- `getValue, setValue`

### CostEngine

- `analyze, execute, initialize_priority, initialize_strategy, monitor, plan, setUp`
- `tearDown`
- `get_prefix`

### ReliabilityEngine

- `analyze, execute, initialize_priority, initialize_strategy, monitor, plan, setUp`
- `tearDown`
- `get_prefix`

### G3T1_1

- `collect, label, setUp, transfer`
- `tearDown`
- `process`

### G3T1_2

- `collect, label, setUp, transfer`
- `tearDown`
- `process`

### G3T1_3

- `collect, label, setUp, transfer`
- `tearDown`
- `process`

### G3T1_4

- `collect, label, setUp, transfer`
- `tearDown`
- `process`

### G3T1_5

- `collect, label, setUp, transfer`
- `tearDown`
- `process`

### G3T1_6

- `collect, label, setUp, transfer`
- `tearDown`
- `process`

### bsn::goalmodel::GoalTree

- `addChildren, addNode, addRootGoal, getLeafTasks, getNode, getNodes, getSize`
- `getActor, operator==, setActor`

### bsn::goalmodel::Property

- `getID, operator==, setID`
- `getValue, setValue`

### arch::ROSComponentDescriptor

- `getName, setName`
- `getFreq, setFreq`

### AdaptationMessage

- `getName`
- `getTimestamp`
- `getLogicalClock`
- `getSource`
- `getTarget`
- `getContent`

### EnergyStatusMessage

- `getName`
- `getTimestamp`
- `getLogicalClock`
- `getSource`
- `getTarget`
- `getCost`

### EventMessage

- `getName`
- `getTimestamp`
- `getLogicalClock`
- `getSource`
- `getTarget`
- `getEvent`

### StatusMessage

- `getName`
- `getTimestamp`
- `getLogicalClock`
- `getSource`
- `getTarget`
- `getState`

### UncertaintyMessage

- `getName`
- `getTimestamp`
- `getLogicalClock`
- `getSource`
- `getTarget`
- `getContent`

### Injector

- `body, gen_noise, inject, seconds_in_cycles, setUp`
- `tearDown`
- `cycles_in_seconds`

### Enactor

- `body, receiveAdaptationParameter, receiveStatus, receiveStrategy`
- `tearDown`
- `print`

### DataAccess

- `applyTimeWindow, body, calculateComponentCost, calculateComponentReliability, flush, now, nowInSeconds, persistAdaptation, persistEnergyStatus, persistEvent, persistStatus, persistUncertainty, processQuery, processTargetSystemData, receivePersistMessage, setUp`
- `tearDown`

### Logger

- `body, now, receiveAdaptationCommand, receiveEnergyStatus, receiveEvent, receiveStatus, receiveUncertainty, setUp`
- `tearDown`

### arch::target_system::Component

- `activate, deactivate, run, sendEnergyStatus, sendEvent, sendStatus, setUp, tearDown`
- `reconfigure`

### Engine

- `body, calculate_qos, fetch_formula, receiveException, sendAdaptationParameter, setUp, setUp_formula`
- `tearDown`

### G4T1

- `collect, getPatientStatus, getSensorId, process, setUp, transfer`
- `tearDown`

### arch::target_system::Probe

- `body, collectEnergyStatus, collectEvent, collectStatus, setUp`
- `tearDown`

### ParamAdapter

- `body, moduleConnect, receiveAdaptationCommand, setUp`
- `tearDown`

### PatientModule

- `body, configureDataGenerator, getPatientData, setUp`
- `tearDown`

### arch::target_system::Effector

- `setUp`
- `tearDown`

### bsn::goalmodel::Task

- `addChild`
- `addChild`

### bsn::time::TimeData

- `elapsedTime`
- `get_time`

### bsn::configuration::SensorConfiguration

- `convertRealPercentage, evaluateNumber, getDisplacement, getHighPercentage, getHighRisk, getId, getLowPercentage, getLowRisk, getMediumRisk, getMidPercentage, isHighRisk, isLowRisk, isMediumRisk, setHighPercentage, setHighRisk, setId, setLowPercentage, setLowRisk, setMediumRisk, setMidPercentage, toString`

### bsn::goalmodel::Node

- `addChild, findChild, getChild, getChildren, getDescription, getID, hasChildren, operator==, removeChild, setDescription, setID`

### bsn::resource::Battery

- `consume, generate, getCapacity, getCurrentLevel, getId, getUnit, setCapacity, setCurrentLevel, setId, setUnit, toString`

### Sensor

- `apply_noise, body, injectUncertainty, isActive, recharge, reconfigure, run, turnOff, turnOn`

### CentralHub

- `apply_noise, body, isActive, recharge, reconfigure, run, turnOff, turnOn`

### bsn::range::Range

- `convert, getLowerBound, getUpperBound, in_range, setLowerBound, setUpperBound, toString, to_print`

### bsn::model::Formula

- `evaluate, getExpression, getTermValueMap, getTerms, setExpression, setTermValueMap, setTermValueMap`

### bsn::filters::MovingAverage

- `getBuffer, getRange, getValue, insert, setRange, toString`

### Controller

- `apply_cost_strategy, apply_reli_strategy, receiveEvent, setUp`

### bsn::generator::DataGenerator

- `calculateValue, getValue, nextState, setSeed`

## ROSE-LCOM-Tools

ROSE foi o candidato inicial e sua configuração foi registrada, mas o runtime
ROSE não está instalado. Ele foi substituído nesta coleta pelo cpplcom;
`status.json` e `configure.*` documentam somente a avaliação daquele candidato,
não a ferramenta que produziu os valores finais.
