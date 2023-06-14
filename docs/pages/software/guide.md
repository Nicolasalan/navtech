# Software Guide

## Comandos e Configurações para utilizar o robo

* **Iniciar robo na simulacao**

```bash
make tf
```

* **Iniciar robo na simulacao sem mundo**

```bash
make sim
```

* **Iniciar robo na simulacao com mundo**

```bash
make sim2
```

* **Iniciar robo na simulacao com SLAM**

```bash
# terminal 1
make sim2
# terminal 2
make slam
```

* **Iniciar robo na simulacao com AMCL**

```bash

```bash
# terminal 1
make sim2
# terminal 2
make amcl
```

* **Iniciar robo na simulacao com Navigation**

```bash
# terminal 1
make sim2
# terminal 2
make navigation
```

* **Atualizar o Docker Hub**

```bash
make build

make push-docker
``` 