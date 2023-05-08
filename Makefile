DOCKER_VOLUMES = \
	--volume="$(shell pwd)/src":"/ws_navtech/src/":rw \

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
	@docker run --platform linux/arm64 -it  --rm --net=host ${DOCKER_VOLUMES} navtech bash

# === Start train docker ===
.PHONY: start 
start:
	@echo "Starting Follow Waypoints ..."
	@docker run --platform linux/arm64 -it --net=host ${DOCKER_VOLUMES} navtech bash -c "ls"