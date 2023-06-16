DOCKER_VOLUMES = \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
  	--volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
 	--volume="/dev:/dev" \
	--volume="$(shell pwd)/src":"/ws_navtech/src/":rw \
	--volume="$(shell pwd)":"/ws_navtech/":rw 

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

# === Build docker === #
.PHONY: build
build:
	@echo "Building docker image ..."
	@sudo docker build -t ninim/navtech:latest . 

# === Clean docker === #
.PHONY: clean
clean:
	@echo "Closing all running docker containers ..."
	@sudo docker system prune -f

# === Run terminal docker === #
.PHONY: terminal
terminal:
	@echo "Terminal docker ..."
	@xhost local:root
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash

# === Start TFs docker === #
.PHONY: tf 
tf:
	@echo "Starting TFs ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot robot.launch.py"

# === Gazebo docker === #
.PHONY: sim2 
sim2:
	@echo "Starting Simulations ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot sim.launch.py  world:=/ws_navtech/src/navtech/robot/worlds/simulation.world"

# === Gazebo docker === #
.PHONY: sim
sim:
	@echo "Starting Simulations ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot sim.launch.py"

# === Gazebo docker === #
.PHONY: sim3
sim3:
	@echo "Starting Simulations ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot hospital.launch.py"


# === Teleop docker === #
.PHONY: teleop 
teleop:
	@echo "Starting Teleop ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# === SLAM ToolBox docker === #
.PHONY: slam
slam:
	@echo "Starting SLAM ToolBox ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot slam.launch.py"

# === AMCL docker === #
.PHONY: amcl
amcl:
	@echo "Starting AMCL ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot localization.launch.py use_sim_time:=true map_yaml_file:=/ws_navtech/src/navtech/robot/config/map_save.yaml"

# === Nav docker === #
.PHONY: nav
nav:
	@echo "Starting AMCL ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot navigation.launch.py"

# === Save Map docker === #
.PHONY: save
save:
	@echo "Saving Map ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 run nav2_map_server map_saver_cli -f /ws_navtech/src/navtech/robot/config/map"

# === Build Docs === #
.PHONY: docs-build
docs-build:
	@echo "Building Docs ..."
	@npm run docs:build

# === Run Docs === #
.PHONY: docs
docs:
	@echo "Starting Docs ..."
	@npm run docs:dev

# === Run === #
.PHONY: run
run:
	@echo "Starting ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} ninim/navtech:latest bash -c "ros2 launch robot run.launch.py"

# === Stop Docker === #
.PHONY: kill
kill:
	@echo "Stopping Docker ..."
	@sudo docker stop $(shell sudo docker ps -a -q)

## =========== CI/CD =========== ##

.PHONY: push-docker
push-docker: 
	@echo "Pushing Docker Image ..."
	@sudo docker tag navtech:latest navtech:latest
	@sudo docker push navtech:latest


