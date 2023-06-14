# Navegação ROS2
A navegação é uma habilidade fundamental para os robôs. Ela permite que eles se movam de um ponto a outro e evitem obstáculos. No entanto, como um robô sabe onde está, para onde deve ir e como evitar colidir com qualquer coisa em seu caminho?

Neste documento, discutiremos o processo de navegação dos robôs, que é semelhante ao dos humanos. Para se mover de um ponto a outro, o robô precisa realizar os seguintes passos:

`Mapeamento`: O robô precisa conhecer o local em que está e construir um mapa dele.

`Localização`: Em seguida, ele precisa saber onde está no ambiente em relação ao mapa criado.

`Planejamento de caminho`: O robô precisa planejar a melhor rota para se mover entre dois pontos, levando em consideração o mapa criado e sua posição atual.

`Controle do robô e prevenção de obstáculos`: Por fim, o robô precisa enviar mensagens para as rodas ou outros dispositivos de movimento para seguir o caminho planejado, evitando obstáculos no caminho.

Todos esses processos são complexos para construir do zero. É aí que o ROS (Robot Operating System) ajuda a construir robôs de forma mais eficiente. O ROS possui pacotes pré-construídos para navegação, permitindo que os desenvolvedores se concentrem em personalizar e otimizar a navegação para o seu robô específico.


## Ferramentas de navegação ROS2

* Carregue, sirva e armazene mapas de ambiente (`Map Server`)
* Localize o robô no mapa (`AMCL`)
* Planeje um caminho de A a B contornando obstáculos (`Nav2 Planner`)
* Controle o robô conforme ele segue o caminho (`Nav2 Controller`)
* Converta os dados do sensor em uma representação do mundo com reconhecimento de obstáculos (`Nav2 Costmap 2D`)
* Compute comportamentos de recuperação em caso de falha (`Nav2 Recoveries`)
* Gerenciar o ciclo de vida dos servidores (`Nav2 Lifecycle Manager`)
* plug-ins para habilitar seus próprios algoritmos e comportamentos personalizados (`Nav2 BT Server`)