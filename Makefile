DOCKER_ENV_VARS = \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" 

DOCKER_VOLUMES = \
	--volume="$(shell pwd)/src":"/ws/src/":rw \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${HOME}/.Xauthority:/root/.Xauthority:rw" 
	
DOCKER_ARGS = ${DOCKER_VOLUMES} ${DOCKER_ENV_VARS}

define xhost_activate
	@echo "Enabling local xhost sharing:"
	@echo "  Display: ${DISPLAY}"
	@xhost local:root
endef

.PHONY: help
help:
	@echo '  help						--Display this help message'
	@echo '  update 					--Update ROS packages from git'
	@echo '  build 					--Build docker image for machine architecture'
	@echo '  clean 					--Docker image cleanup'
	@echo '  start						--Start training session'
	@echo '  terminal					--Start terminal in docker'
	@echo '  setup						--setup world and robot'
	@echo '  view						--setup view gazebo'
	@echo '  library					--Test library functions'
	@echo '  ros						--Test ROS topics'
	@echo '  sim						--Test Simulation Gazebo'
	@echo '  package					--Test Dependencies'
	@echo '  integration					--Test All'

# === Build docker ===
.PHONY: build
build:
	@echo "Building docker image ..."
	@UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"
	@docker build ${UPAR} -t robot-docker . 
# === Clean docker ===
.PHONY: clean
clean:
	@echo "Closing all running docker containers ..."
	@sudo docker system prune -f

# === Run terminal docker ===
.PHONY: terminal
terminal:
	@echo "Terminal docker ..."
	@sudo xhost + 
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash

# === setup model ===
.PHONY: setup
setup:
	@echo "Setup world ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roslaunch robot bringup.launch"

# === setup view ===
.PHONY: view 
view:
	@echo "Setup View world ..."
	@sudo xhost +
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roslaunch robot view.launch"

# === Start train docker ===
.PHONY: start 
start:
	@echo "Starting training ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roslaunch robot start.launch"

# === Test Library ===
.PHONY: library
library:
	@echo "Testing ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roscd robot && python3 test/library.py"

# === Test ROS ===
.PHONY: ros
ros:
	@echo "Testing ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roscd robot && python3 test/ros.py"

# === Test Simulation ===
.PHONY: sim
sim:
	@echo "Testing ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roscd robot && python3 test/sim.py"

# === Test Package ===
.PHONY: package
package:
	@echo "Testing ..."
	@sudo docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roscd robot && python3 test/package.py"

# === Test Full ===
.PHONY: integration
integration:
	@echo "Testing ..."
	@docker run -it --net=host ${DOCKER_ARGS} robot-docker bash -c "source devel/setup.bash && roscd robot && python3 test/ros.py && python3 test/library.py && python3 test/package.py && python3 test/sim.py"
