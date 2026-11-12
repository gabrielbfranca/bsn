# CBO — variante CCCC

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

Foram extraídos valores para 44 das 46 identidades
do inventário. Os dois structs `comp` não têm valor individual reportado.
O CCCC usa nomes não qualificados: a associação só foi feita quando o nome
curto é único no inventário. Identidades ambíguas não recebem um número.

O relatório nativo registra **313 linhas rejeitadas**. Trata-se de um
parser antigo, com limitações em macros, templates e sintaxe C++11. Os valores
foram preservados como uma variante com cobertura limitada; ausência de
relações pode ser falta de reconhecimento. Não usar o valor como uma medida
semântica completa ou comparar seu total com o grafo Doxygen.

O máximo entre as classes próprias associadas é
11; os módulos adicionais do XML nativo,
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

| Classe qualificada | Módulo CCCC | CBO nativo |
| --- | --- | --- |
| AdaptationMessage | AdaptationMessage | 2 |
| CentralHub | CentralHub | 7 |
| Controller | Controller | 4 |
| CostEngine | CostEngine | 4 |
| DataAccess | DataAccess | 11 |
| Enactor | Enactor | 5 |
| EnergyStatusMessage | EnergyStatusMessage | 2 |
| Engine | Engine | 11 |
| EventMessage | EventMessage | 2 |
| G3T1_1 | G3T1_1 | 8 |
| G3T1_2 | G3T1_2 | 9 |
| G3T1_3 | G3T1_3 | 9 |
| G3T1_4 | G3T1_4 | 9 |
| G3T1_5 | G3T1_5 | 9 |
| G3T1_6 | G3T1_6 | 9 |
| G4T1 | G4T1 | 5 |
| Injector | Injector | 6 |
| Logger | Logger | 6 |
| ParamAdapter | ParamAdapter | 7 |
| PatientModule | PatientModule | 7 |
| ReliabilityEngine | ReliabilityEngine | 4 |
| Sensor | Sensor | 11 |
| StatusMessage | StatusMessage | 2 |
| UncertaintyMessage | UncertaintyMessage | 2 |
| arch::ROSComponent | ROSComponent | 11 |
| arch::ROSComponentDescriptor | ROSComponentDescriptor | 2 |
| arch::target_system::Component | Component | 9 |
| arch::target_system::Effector | Effector | 5 |
| arch::target_system::Probe | Probe | 6 |
| bsn::configuration::SensorConfiguration | SensorConfiguration | 10 |
| bsn::filters::MovingAverage | MovingAverage | 9 |
| bsn::generator::DataGenerator | DataGenerator | 8 |
| bsn::generator::Markov | Markov | 9 |
| bsn::goalmodel::Context | Context | 3 |
| bsn::goalmodel::Goal | Goal | 2 |
| bsn::goalmodel::GoalTree | GoalTree | 2 |
| bsn::goalmodel::LeafTask | LeafTask | 5 |
| bsn::goalmodel::Node | Node | 5 |
| bsn::goalmodel::Property | Property | 2 |
| bsn::goalmodel::Task | Task | 4 |
| bsn::model::Formula | Formula | 5 |
| bsn::range::Range | Range | 1 |
| bsn::resource::Battery | Battery | 3 |
| bsn::time::TimeData | TimeData | 1 |
