# Análise estática de classes ROS 1

Raiz Git desta coleta: `/workspaces/bsn_`. Não há execução ROS, compilação da
BSN, commit ou push. Fontes de produção e alterações prévias ficam preservados.

## O que cada ferramenta calcula

| Informação | Origem | Consolidação local |
|---|---|---|
| CC de função/método | Lizard 1.26.0, saídas nativas CSV/HTML/texto | Associação à classe pelo inventário Doxygen e localização |
| WMC dos métodos definidos | CC original do Lizard | Soma dos CC dos métodos definidos diretamente na classe |
| NOM | Inventário Doxygen e métodos reconhecidos pelo Lizard | Contagens distintas de declarações e definições medidas |
| Relações de classes | Referências de tipos e herança no XML nativo Doxygen 1.8.13 | Extração de arestas; sem inferência própria de tipos |
| Ciclos | `networkx.strongly_connected_components`, NetworkX 3.4.2 | Seleção de componentes com mais de um vértice |
| Dependência de infraestrutura | Semgrep CE 1.185.0 | Associação de achados à classe/método/fase |
| LCOM4 | cpplcom 0.1.0, commit 3b8ce6150d142da0e58bf7968144f26a5854b647, JSON e texto nativos | Associação por classe qualificada; valor bruto preservado |
| CBO C++ | CCCC 3.1.4, HTML/XML/banco/opções nativos | Extração do XML e associação por nome curto único; variante diferente de pyscn |

`outgoing_internal_type_dependencies` é uma contagem **derivada** das relações
de tipos internos presentes em assinaturas e atributos no XML. Não é saída
nativa de CBO e não equivale ao CBO do pyscn. Não inclui uma resolução completa
dos usos de classes em corpos nem das dependências externas.

O código local não implementa LCOM4 nem CC. Implementa seleção de arquivos,
invocação de ferramentas, associação de resultados, somas, contagens e tabelas.
As regras Semgrep são critérios locais de inspeção, não uma métrica padronizada
de testabilidade. Onze regras vieram da branch ROS 2; sete adicionais cobrem
`fstream`, abertura por receptor de nome reconhecido, relógio de alta resolução,
parâmetros, espera de `rate`, sinais, caminhos de pacote e diretórios.

## Reprodução neste ambiente

As ferramentas estão instaladas em `tools/.metrics-venv` e
`tools/.metrics-runtime`; ambas as pastas são ignoradas pelo Git. Python usado:
3.11.14; o Python 3.6 do ROS permanece disponível.

```bash
bash tools/ros1_metrics/run_all.sh
```

O coletor substitui apenas os seus relatórios em `reports/ros1_static`.
Os relatórios de referência ROS 2 e o diagnóstico de LCOM4 são preservados.
Saídas das ferramentas ficam em `native/`; tabelas e interpretação ficam fora
desse diretório. Validações e saídas correspondentes ficam em `validation/`.

Em outro ambiente com Python 3.11, crie `tools/.metrics-venv` e instale
`requirements-lock.txt` com `--no-deps` (a lista já fixa todas as dependências;
isso permite reutilizar as bindings do pacote libclang no cpplcom).
Doxygen 1.8.13 é a versão da coleta. Se o executável
estiver no PATH, será usado; caso contrário, o coletor procura a extração local
em `tools/.metrics-runtime/doxygen-root/usr/bin/doxygen`. A repetição com outra
versão deve ser identificada como outra coleta.

O wheel glibc do Semgrep 1.185.0 exige glibc 2.34; o Ubuntu 18.04 tem 2.27.
Foi usado o **binário oficial musllinux da mesma versão**, com carregador musl
1.2.5-r13 extraído localmente. `semgrep-core` é somente um wrapper de execução,
não uma modificação do analisador. A interface utilizada é `pysemgrep`, porque
o wrapper não preserva o nome de execução exigido pelo despachante `osemgrep`.
O coletor registra versões e o caminho do carregador. Essa preparação não
instala bibliotecas sobre as do sistema.

### Preparação adicional de LCOM4/CBO

O cpplcom é uma ferramenta externa Python/libclang, instalada sem modificar
seus fontes, a partir do commit indicado. Clang/libclang 18.1.8 foram extraídos
dos pacotes oficiais `apt.llvm.org/bionic` para `tools/.metrics-runtime/clang18-root`.
O CCCC veio do pacote Ubuntu `1:3.1.4-9`, extraído em `cccc-root`. URLs, versões
e hashes estão em `oo-metadata.json`; os pacotes originais ficam no runtime
ignorado pelo Git. As bindings Python são de `libclang==18.1.1`. O cpplcom é
instalado com `--no-deps` para reutilizar essas bindings, pois a distribuição
PyPI `libclang` já fornece o módulo `clang.cindex`.

`collect_oo.py` gera os headers de mensagens e serviços com o gencpp instalado
em `/opt/ros/melodic`, usando Python 2 do ROS. São dependências legítimas do
parser; não há stubs, execução ROS nem build dos executáveis. O banco de
comandos de parsing é explícito e inclui os diretórios próprios e do ROS.
O código legado `apps/enactor.cpp` não é alvo CMake e instancia uma classe
abstrata: permanece em Lizard/Semgrep, mas sai do banco AST; a classe Enactor
continua medida por sua implementação real. A decisão e o diagnóstico nativo
inicial estão em `lcom4/`.

O cpplcom reporta somente classes com ao menos dois métodos elegíveis. Seu
LCOM4 bruto inclui vazios, getters e métodos sem estado; `BARE` e `jobs` são
ajustes próprios e ficam separados. O teste upstream de member template
divergiu com Clang 18; os logs originais foram preservados. Nos fontes próprios,
templates de métodos estão apenas nos structs comp de um método, não reportados
por LCOM4. Não se afirma validade geral de templates.

O CCCC mede clientes + fornecedores, usa nomes não qualificados e rejeitou
313 linhas na coleta. O valor é nativo, porém sua cobertura é limitada;
não deve ser comparado numericamente com CBO pyscn. Os dois comp não recebem
um valor individual. O indicador de tipos internos Doxygen permanece separado.

## Escopo e convenções

Entram headers e fontes C++ próprios de `src/archlib`, `src/libbsn` e
`src/sa-bsn`, incluindo executáveis de entrada. Saem testes, auxiliares de
testes, Lepton (terceiro), código gerado e o exemplo de binding não incluído
nos alvos CMake. `files.txt` e `excluded.csv` registram as decisões.

Uma classe é identificada pelo nome qualificado. Os dois `struct comp`
globais, definidos separadamente em CostEngine.cpp e ReliabilityEngine.cpp,
são desambiguados pelo arquivo na tabela derivada. O XML nativo Doxygen os
agrupa e permanece intacto. A análise não decide a identidade dessas
definições segundo a ODR nem faz linkagem.

NOM declarado inclui declarações sem corpo, inclusive virtuais puros e cópias
privadas sem definição. NOM definido e WMC usam somente corpos medidos.
Construtores, destrutores, operadores e estáticos com corpo entram; herdados
e funções livres, incluindo `friend operator<<`, não entram novamente.
Uma declaração sem corpo não recebe CC inventado. Métodos implicitamente
gerados pelo compilador também não são acrescentados.

O Doxygen não resolveu dois construtores que usam aliases de `std::array`.
A associação derivada usa nome qualificado, nome do método e aridade única
do Lizard; `methods.csv` identifica esses casos em `mapping`. Nenhum método
com corpo reconhecido pelo Doxygen ficou sem CC. `unassigned-functions.csv`
preserva as funções livres excluídas da agregação por classe.

O grafo de ciclos usa somente herança e tipos de atributos próprios resolvidos
pelo Doxygen, excluindo autorrelações. Não é grafo de includes, arquivos ou
pacotes. Referências em assinaturas entram apenas no indicador separado de
dependências internas. Macros não são expandidas e includes externos não são
percorridos; o grafo é parcial e não prova ausência de ciclos em toda execução.

## Validação e rastreabilidade

Antes da coleta principal, exemplos verificam CC 1/2 nas duas linguagens,
classes homônimas, união header/fonte, herança e ciclo conhecido. Os testes
nativos Semgrep validam 11 regras originais e 7 adicionais. Os exemplos de
LCOM4 foram executados pelo cpplcom, com resultados 1/2/1 nos três casos
básicos. Os exemplos CCCC confirmam relações no mesmo arquivo, clientes e
fornecedores. Saídas nativas de validação permanecem em `validation/`.

`metadata.json` registra versões, SHA-256 dos fontes e scripts, Git HEAD e
estado local. `commands.json` contém comandos, códigos de saída e arquivos de
stdout/stderr da coleta. `requirements-lock.txt` fixa dependências Python.
`reference_ros2/sources.json` identifica commit e blobs consultados no GitHub.

Não compare o total bruto de ocorrências Semgrep entre linguagens: regras e
cobertura diferem e achados podem se sobrepor. Um efeito no construtor da base
é sinalizado na coluna de ancestrais, sem duplicar seus achados em subclasses.
Isso é evidência de contrato herdado, não uma análise completa do fluxo de
chamadas. Candidatos a colaboradores recebidos no construtor exigem inspeção:
parâmetros de cópia e objetos de valor não comprovam injeção de dependência.
