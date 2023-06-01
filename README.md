# **NavTech**

<p align="center">
  <a href="http://wiki.ros.org/humble">
    <img src="https://img.shields.io/badge/ROS-Humble-red" alt="ROS Humble Compatible">
  </a>
  <a href="https://docs.docker.com/">
    <img src="https://img.shields.io/badge/Docker-v20.10.21-blue" alt="Docker">
  </a>
  <a href="https://www.python.org/">
    <img src="https://img.shields.io/badge/Python-v3.8.10-brightgreen" alt="Python3">
  </a>
  <a href="https://releases.ubuntu.com/">
    <img src="https://img.shields.io/badge/Ubuntu-v20.04-9cf" alt="Ubuntu">
  </a>
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
git clone https://github.com/Nicolasalan/NavTech.git
``` 

## Build
Agora, vá para o diretório raiz do projeto e execute o seguinte comando para construir a imagem do Docker:

```bash
make build
```

## Executando o Robô

Após a construção da imagem do Docker, você pode executar o robô utilizando o Makefile fornecido. Veja os principais comandos disponíveis:

- `make run`: inicia o ambiente Docker e executa o robô.
- `make stop`: para a execução do robô.
- `make clean`: remove os contêineres Docker e limpa o ambiente.

Exemplo de uso:
```bash
make run
make stop
make clean
```

## Estrutura de pastas

```
docs/                  # Documentação do projeto
src/
├── navtech/
│   ├── robot/
│   │   ├── config/      # Arquivos de configuração
│   │   ├── description/ # Arquivos URDF
│   │   ├── launch/      # Arquivos de lançamento
│   │   ├── robot/       # Arquivos de código-fonte
│   │   ├── worlds/      # Arquivos de mundo
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
entrypoint.sh          # Script de entrada
Makefile               # Comandos de execução MakeFile
README.md              # README
Dockefile              # Imagem Docker
```

## Contribuição

Contribuições são bem-vindas! Se você quiser colaborar com o projeto, sinta-se à vontade para abrir uma issue ou enviar um pull request.

## Licença

Este projeto está licenciado sob a MIT.