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

* A pasta `config` contem todas as especificacoes do robo sobre mapeamento e navegacao, alem de possuir o mapa do ambiente e arquivo de rviz para visualizacao do robo.
* O `description` contem o arquivo URDF do robo, que é utilizado para a simulacao do robo, junto com os controladores e sensores utilizado no robo, que sao especificados no arquivo `robot.urdf.xacro`.
* O `launch` contem os arquivos de lancamento do robo, que sao utilizados para iniciar a simulacao do robo. Os principais sao navigation e simulation, que sao utilizados para iniciar a navegacao e simulacao do robo, respectivamente.
* O `test` e `worlds` contem os arquivos de teste e mundo, respectivamente.

## Configuração do Ambiente
Para configurar o ambiente é necessario ter o docker instalado em sua maquina, pois todo o sistema é construido em cima do container. [Docker](https://docs.docker.com/engine/install/ubuntu/)

O arquivo `makefile` contem todos os comandos necessarios para a execucao do sistema, para executar o sistema basta executar o comando `make run` no terminal.

Isso iniciara a simulacao e junto com o objetivo do robo, no qual é passar por todos os waypoints, demostrando a navegacao do robo.