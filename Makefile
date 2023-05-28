DOCKER_VOLUMES = \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
  	--volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
 	--volume="/dev:/dev" \
	--volume="$(shell pwd)/src":"/ws_navtech/src/":rw 

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
	@sudo docker build -t navtech . 

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
	@sudo docker run -it --net=host ${DOCKER_ARGS} navtech bash

# === Start TFs docker === #
.PHONY: tf 
tf:
	@echo "Starting TFs ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} navtech bash -c "ros2 launch robot robot.launch.py"

# === Gazebo docker === #
.PHONY: sim2 
sim2:
	@echo "Starting Simulations ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} navtech bash -c "ros2 launch robot sim.launch.py  world:=/ws_navtech/src/navtech/robot/worlds/obstacles.world"

# === Gazebo docker === #
.PHONY: sim
sim:
	@echo "Starting Simulations ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} navtech bash -c "ros2 launch robot sim.launch.py"

# === Teleop docker === #
.PHONY: teleop 
teleop:
	@echo "Starting Teleop ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} navtech bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# === SLAM ToolBox docker === #
.PHONY: slam
slam:
	@echo "Starting SLAM ToolBox ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} navtech bash -c "ros2 launch robot slam.launch.py"