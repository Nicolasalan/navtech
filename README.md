# NavTech

<p align="center">
  <img src="doc/image.png" alt="NavTech">
</p>

- **A - Robô doméstico**

Pedido do cliente: “Sou idoso e quero um robô que limpe o chão e o carpete da
minha casa, e que entenda minhas necessidades, tais como pegar meus
remédios e um copo d’agua para eu tomá-los no horário correto”.

- **B - Robô auxiliar de hospital**

Pedido do cliente: “Preciso de um robô que possa trabalhar no hospital
entregando alimentos e remédios para os pacientes. Ele deverá entregar as
refeições e remédios corretamente a cada paciente, porque cada paciente possuí
diferentes restrições alimentares e medicações prescrita pelo médico”.

- **C - Robô de limpeza**

Pedido do cliente: “Sou gerente de hotel, mas esse hotel é bem grande, então
pensei que um robô poderia ajudar na limpeza dos quartos, pelo menos
limpando o chão dos quartos e limpando o banheiro, se possível trocando as
toalhas sujas por limpas”. 

## Phase 1: Project Definition

### Aplicação
* Navegação waypoints
* Navegação waypoints com obstáculos
* Cobertura de área
* Trajetória de limpeza
* Seguir pessoas
* Procurar objetos
* Entrega de objetos
### Empresas
* [Robotics companies](https://github.com/vmayoral/ros-robotics-companies)

## Phase 2: Early Design

- [ ] Conceptual Design
- [ ] Preliminary Design
- [ ] Detailed Design

## Phase 3: Hardware Components
- [ ] Travel Router | Price: R$ 326,00 | [site](https://www.amazon.com.br/GL-iNet-Roteador-port%C3%A1til-viagem-GL-MT300N-V2/dp/B073TSK26W/ref=sr_1_3?__mk_pt_BR=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3I3TCEPMLWI7V&keywords=travel+router&qid=1677383528&sprefix=%2Caps%2C390&sr=8-3&ufe=app_do%3Aamzn1.fos.fcd6d665-32ba-4479-9f21-b774e276a678)
- [ ] Raspberry Pi 4B | Description - (64 bits (ARM ou x86) e, de preferência, tenha pelo menos 4 GB de RAM.) ***RoboFEI**
- [ ] 5V Regulator | Description - (Conversor buck de modo chaveado de 12V para 5V.)
- [ ] Main power switch | Price $ 3,50 | [site](https://www.sparkfun.com/products/11310)
- [ ] Momentary switch (Pi soft power) | Price $ 1,50 | [site](https://www.adafruit.com/product/1443)
- [ ] Terminal strip | Price R$ 12,87 | [site](https://www.amazon.com.br/Barra-Terminais-Conector-Sindal-Bornes/dp/B0B787Y1GH/ref=asc_df_B0B787Y1GH/?tag=googleshopp00-20&linkCode=df0&hvadid=426453809273&hvpos=&hvnetw=g&hvrand=14602387887549824872&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1001767&hvtargid=pla-1968031377834&psc=1)
- [ ] Various wires	***RoboFEI**
- [ ] Appropriate connectors | Description - (Verificar as classificações atuais) ***RoboFEI**
- [ ] Breadboard 	***RoboFEI**
- [ ] Arduino (e.g. Nano) | Price R$ 53,67 | [site](https://br.banggood.com/Geekcreit-ATmega328P-Nano-V3-Module-Improved-Version-No-Cable-Development-Board-Geekcreit-for-Arduino-products-that-work-with-official-Arduino-boards-p-959231.html?imageAb=1&warehouse=CN&ID=6300464&p=F6190022409212015085&custlixnkid=1756659&akmClientCountry=BR&a=1677384194.9769&akmClientCountry=BR&cur_warehouse=CN)
- [ ] 2x12V DC Motors w/ encoders | Description - (Certifique-se de que ele tenha codificadores, para feedback de velocidade!) | Price R$ 103,70 | [site](https://au.banggood.com/Machifit-25GA370-DC-12V-Micro-Gear-Reduction-Encoder-Motor-with-Mounting-Bracket-and-Wheel-p-1532242.html?cur_warehouse=CN)
- [ ] Motor driver board | Description - (Ou BTS7960) | Price R$ 45,62 | [site](https://www.banggood.com/Wholesale-L298N-Dual-H-Bridge-Stepper-Motor-Driver-Board-p-42826.html?warehouse=HK&ID=0&p=F6190022409212015085&custlinkid=1756662&cur_warehouse=CN)
- [ ] Câmera Raspberry Pi v2 8MP | Price R$ 69,00 | [site](https://www.robocore.net/acessorios-raspberry-pi/camera-para-raspberry-pi-rev-1-3)
- [ ] RPLidar A1 | Description - (Verificar no Lab) | Price R$ R$741,99 | [site](https://www.amazon.com.br/Waveshare-RPLIDAR-A1-Omnidirectional-Acquisition/dp/B0B6B5MWSJ/ref=sr_1_1?__mk_pt_BR=%C3%85M%C3%85%C5%BD%C3%95%C3%91&keywords=RPLidar+A1&qid=1677384697&sr=8-1&ufe=app_do%3Aamzn1.fos.25548f35-0de7-44b3-b28e-0f56f3f96147)
- [ ] Panel mount connectors | Description - (USB, HDMI)
- [ ] 3d printer 	***RoboFEI**
- [ ] Cooling Raspberry Pi 	***RoboFEI**
- [ ] Various crimp tools and their connectors (bootlace, spade, JST, etc.) ***RoboFEI**
- [ ] Soldering iron and solder ***RoboFEI**
- [ ] Multimeter ***RoboFEI**
- [ ] Various screwdrivers ***RoboFEI**
- [ ] Portable keyboard/mouse/screen ***RoboFEI**
- [ ] Fasteners and adhesives (cable ties, screws, bolts, tape, etc.) ***RoboFEI**
- [ ] Rotary Tool/Dremel (hand tools or other power tools can also be used) ***RoboFEI**

## Phase 4: Assembly

- [ ] Placa de circuito impresso
- [ ] Montagem do robô
- [ ] Impressão 3D

## Phase 5: Software

### Requirements

- [ ] Git
- [ ] Docker
- [ ] VSCode
- [ ] ROS Noetic
- [ ] Gazebo
- [ ] Python 3.8
- [ ] OpenCV
- [ ] PyTorch
- [ ] Arduino IDE

### Applications

- [ ] Robot description
- [ ] Robot control
- [ ] Robot perception
- [ ] Robot learning
- [ ] Robot driver
- [ ] robot interface

## Phase 6: Testing

- [ ] Simulation
- [ ] Real robot

## Phase 7: Documentation

- [ ] Wiki
- [ ] Video

## Phase 8: Presentation

- [ ] Powerpoint