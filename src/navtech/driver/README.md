# Robot Base Driver

## 💻 Pre-requisites

Antes de começar, certifique-se de que sua máquina atende aos seguintes requisitos:

* [Docker](https://docs.docker.com/get-docker/)
* [Arduino IDE](https://www.arduino.cc/en/software)

## 🚀 Instalando Robot Base Driver

Após baixar o Arduino IDE, você precisa adicionar os pacotes .zip ao Arduino IDE. Para fazer isso, vá em `Sketch -> Include library -> Add .ZIP Library...`, e selecione os pacotes `.zip` localizados na pasta `libraries` do projeto.

Depois de adicionar os pacotes, você precisa modificar o caminho do Arduino. Abra a visualização de arquivos ocultos e abra o arquivo `~/.arduino15`.

```bash
export ARDUINO_PATH=~/.arduino15/packages/arduino
cd $ARDUINO_PATH/hardware/sam/1.6.12/
curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt
```

Não se esqueça de modificar a porta do Arduino para a porta em que o Arduino está conectado. No caso do Arduino Due, a porta é `/dev/ttyUSB0`. Para modificar, vá para `Tools -> Port -> /dev/ttyUSB0`. E também verifique a placa que está sendo utilizada. No caso do Arduino Due, a placa é `Arduino Due`. Para modificar, vá para `Tools -> Board -> Arduino ARM (32-bits) Boards -> Arduino Due (Programming Port)`.

## ☕ Iniciando o Robot Base Driver

Abra o arquivo `robot_base_driver.ino`, localizado na pasta `robot_base_driver`, dentro do Arduino IDE. Clique em `Verify` para verificar se não há erros e, em seguida, clique em `Upload` para enviar o código para o Arduino.

Nesta etapa, o código já está no Arduino, mas ainda não é capaz de visualizar nenhuma mensagem `ROS Humble`. Para começar a receber e criar mensagens, inicie o agente Micro ROS.

```bash
docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6
```
Depois disso, o agente Micro ROS já está pronto para receber e enviar mensagens.