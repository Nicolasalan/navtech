# **NavTech**
<p align="center">
  <a href="http://wiki.ros.org/Foxy"><img alt="ROS Foxy Compatible" src="https://img.shields.io/badge/ROS-Foxy-red"></a>
  <a href="https://docs.docker.com"><img alt="Docker" src="https://img.shields.io/badge/Docker-v20.10.21-blue"></a>
  <a href="https://www.python.org/"><img alt="Python3" src="https://img.shields.io/badge/Python-v3.8.10-brightgreen"></a>
  <a href="https://releases.ubuntu.com/"><img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-v20.04-9cf"></a>
</p>

O desenvolvimento completo de um robô omnidirecional autônomo envolve a utilização da pilha de navegação do ROS 2 (Robot Operating System 2) juntamente com componentes eletrônicos para a construção do robô. O ROS 2 é um framework amplamente utilizado para desenvolvimento de sistemas robóticos, fornecendo uma variedade de ferramentas e bibliotecas para facilitar a navegação autônoma.

<p align="center">
  <img src="docs/public/image.png" alt="NavTech">
</p>

Este repositório contém o código-fonte do projeto NavTech, que é um robô omnidirecional autônomo desenvolvido utilizando o ROS 2 e Docker.

## Pré-requisitos

Certifique-se de ter as seguintes dependências instaladas em seu sistema:

* [Git](https://git-scm.com/download/)
* [Make](https://www.gnu.org/software/make/#download)
* [Docker](https://docs.docker.com/get-docker/)

## Clonando o Repositório

Para começar, clone este repositório em seu ambiente local:
```bash
git clone https://github.com/Nicolasalan/navtech.git
``` 

## Build
Agora, vá para o diretório raiz do projeto e execute o seguinte comando para construir a imagem do Docker:

```bash
make build
```

## Executando o Robô

Após a construção da imagem do Docker, você pode executar o robô utilizando o Makefile fornecido. Veja os principais comandos disponíveis:

- `make sim`: inicia o ambiente Docker e executa o robô na simualção.
- `make slam`: inicia a execução do SLAM.
- `make nav`: inicia o pacote de navegação.

Exemplo de uso:
```bash
# terminal 1
make sim
# terminal 2
make slam
# terminal 3
make nav
```

Exemplo de utilização: 

<center>
  <div style="display: flex; justify-content: space-between; align-items: center;">
    <figure style="text-align: center; flex: 1;">
      <img src="docs/gif/gazebo.gif" alt="Summit Lake - Charles Donlea" width="400px">
    </figure>
    <figure style="text-align: center; flex: 1;">
      <img src="docs/gif/rviz.gif" alt="Confident Data Skills - Kirill Eremenko" width="400px" height="343px">
    </figure>
  </div>
</center>


## Estrutura de pastas

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
entrypoint.sh             # Script de entrada
Makefile                  # Comandos de execução MakeFile
README.md                 # README do projeto
Dockefile                 # Imagem Docker
```

## Contribuição

Contribuições são bem-vindas! Se você quiser colaborar com o projeto, sinta-se à vontade para abrir uma issue ou enviar um pull request.

## Licença

Este projeto está licenciado sob a MIT.
