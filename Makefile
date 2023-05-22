DOCKER_VOLUMES = \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
  	--volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
 	--volume="/dev:/dev" \
	--volume="${PWD}/src":"/ws_navtech/src/":rw 

DOCKER_ENV_VARS = \
 	--env="DISPLAY=${DISPLAY}" \
 	--env="QT_X11_NO_MITSHM=1" \
 	--ipc=host \
 	--privileged 

DOCKER_ARGS = ${DOCKER_VOLUMES} ${DOCKER_ENV_VARS}

define xhost_activate
	@echo "Enabling local xhost sharing:"
	@echo "  Display: ${DISPLAY}"
	@xhost local:root
endef

.PHONY: help
help:
	@echo '  help						--Display this help message'
	@echo '  build 					--Build docker image for machine architecture'
	@echo '  clean 					--Docker image cleanup'
	@echo '  start						--Start training session'
	@echo '  terminal					--Start terminal in docker'

# === Build docker ===
.PHONY: build
build:
	@echo "Building docker image ..."
	@docker build --platform linux/arm64 -t navtech . 

# === Clean docker ===
.PHONY: clean
clean:
	@echo "Closing all running docker containers ..."
	@docker system prune -f

# === Run terminal docker ===
.PHONY: terminal
terminal:
	@echo "Terminal docker ..."
	@xhost local:root
	@docker run --platform linux/arm64 -it  --rm --net=host ${DOCKER_ARGS} navtech bash

# === Start train docker ===
.PHONY: start 
start:
	@echo "Starting Follow Waypoints ..."
	@docker run --platform linux/arm64 -it --net=host ${DOCKER_ARGS} navtech bash -c "ls"