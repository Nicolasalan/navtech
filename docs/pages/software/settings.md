# Configuração do Sistema

O Sistema utilizado no projheto foi dividio em pacotes, mesmo utilizado em ROS, alem disso. Nesse caso o pacote robot, contem todo o sistema de simualcao e navegacao do robo.

```
docs/                     # Documentação do projeto
src/
├── navtech/
│   ├── robot/
│   │   ├── config/       # Arquivos de configuração
│   │   ├── description/  # Arquivos URDF
│   │   ├── launch/       # Arquivos de lançamento
│   │   ├── robot/        # Arquivos de código-fonte
│   │   ├── worlds/       # Arquivos de mundo
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
```  

## Configuração do Ambiente
Para configurar o ambiente é necessario ter o docker instalado em sua maquina, pois todo o sistema é construido em cima do container. [Docker](https://docs.docker.com/engine/install/ubuntu/)

O arquivo `makefile` contem todos os comandos necessarios para a execucao do sistema, para executar o sistema basta executar o comando `make run` no terminal.

Isso iniciara a simulacao e junto com o objetivo do robo, no qual é passar por todos os waypoints, demostrando a navegacao do robo.