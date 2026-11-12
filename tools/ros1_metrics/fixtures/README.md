# Validação anterior à coleta

`classes.hpp/cpp`: Lizard deve dar CC=1 e 2 em `fixtures::Branch`,
igual ao exemplo Python. Doxygen deve unir declarações e implementações,
distinguir `second::Branch`, reconhecer herança Derived→Base e o ciclo A↔B.
Métodos puramente virtuais devem aparecer apenas no inventário de declarações.

Casos de coesão para futura execução ROSE: SharedState=1, SplitState=2 **apenas
com o construtor excluído**, CallBridge=1. O construtor em SplitState acessa
ambos os campos e pode unir os grupos. OnlyConstructor, Stateless e
SeparateGetters verificam convenções diferentes de inclusão; não têm um valor
universal intercambiável com pyscn. Estes casos não foram medidos por ROSE.
