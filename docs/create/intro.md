# Exemplo de modelo de robô

Antes da demonstração, você precisa executar um comando git clone da sua pasta `ros2\_ws/src` para obter o código de demonstração.

```bash
cd ~/ros2_ws/src
git clone https://bitbucket.org/theconstructcore/my_robot_model.git
```

Com os arquivos no lugar, instale-os criando o espaço de trabalho:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_model
```
Levará alguns segundos para exibir uma conclusão bem-sucedida da compilação:
Como de costume, você deve fornecer seu terminal após a compilação:
```bash
source install/setup.bash
```
Agora estamos prontos para iniciar uma simulação:
```bash
ros2 launch my_robot_model start_world.launch.py
```
Aguarde até ver a janela de simulação aberta no canto superior direito da tela. Em seguida, em um novo shell, gere um modelo de robô:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_model spawn_robot_ros2.launch.xml
```
Para ampliar o modelo, clique na tela Gazebo para ativá-lo e, em seguida, use a roda de rolagem do mouse para ampliar onde o ponteiro do mouse está.

## O que você vai aprender?
Este seção ensinará como ir de um robô físico, ou mesmo um desenho de robô, para uma simulação completa com física, atuadores e sensores.

Para isso, você aprenderá a criar um arquivo que define seu robô no ecossistema Gazebo-ROS2. Este arquivo determinará pesos, inércias, articulações, links, plugins de sensores e tudo o que for necessário para simular um robô.

Esses arquivos são chamados de arquivos URDF. Eles são baseados na linguagem XML, por isso é simples de entender.

## Por que preciso do URDFS?
URDFS são usados para duas aplicações principais:

* Tenha uma definição do modelo para o sistema ROS que precisa dele, como publicação, navegação e manipulação de TF.
* Você pode usá-los para gerar uma versão simulada do seu robô em um Gazebo Simulator.

## Por que preciso de um modelo de robô?
Existem muitas razões pelas quais alguém precisaria simular um robô:

* Você precisa de alguma forma para descrever seu robô físico no ROS: Os modelos URDF não são necessários apenas para simulações; eles também são usados para fornecer uma descrição virtual do robô real que está publicando dados no ROS. Por exemplo, programas ROS como o RVIZ podem representar o robô com base nos valores reais das juntas.
* É impossível trabalhar com o robô real:
Existem muitos robôs que não estão disponíveis para o público em geral, como ASIMO, VALKIRIA, ROBONAUT e ATLAS, para citar alguns. Eles são robôs bem projetados, mas poucas pessoas podem usá-los; e se puderem usar os robôs, estarão sujeitos a condições estritas. Então, por que você não cria sua própria versão virtual dele?
* Eu quero criar um robô físico que não existe:
Construir um robô é caro, então você deve confirmar com antecedência se não há falhas significativas de design.
Ao criar primeiro o robô em uma simulação, você pode iterar muito mais rápido no design e detectar quaisquer erros antes de iniciar a impressão 3D.
Você também pode testar e alterar o design sem nenhum custo real. Por exemplo, o que aconteceria se você adicionasse uma ou duas pernas, câmeras, usasse servidores mais poderosos ou o tornasse mais pesado? A lista não tem fim.
*  É muito arriscado fazer meus testes preliminares no robô físico:
Você tem o robô, mas pode querer testar algo muito perigoso para a integridade do robô.
Talvez você queira experimentar um algoritmo para interação humano-robô pela primeira vez e não queira que ninguém se machuque nos testes.