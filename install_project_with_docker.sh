#!/bin/bash

if [ -z $1 ];then
	echo "Wrong script usage, one argument 'projects/my_project' is expected"
	exit 1
fi

if [ ! $(command -v docker) ]; then
	INSTALL=0
	while [ $INSTALL -eq 0 ]; do
		read -p "Docker is not installed, do you wish to install it?(y/n)" yn
		case $yn in
			[y]* ) INSTALL=1 ;;
			[n]* ) INSTALL=2 ;;
			* ) echo "Please enter a valid option number";;
		esac
	done
	if [ $INSTALL -eq 1 ]; then
		sudo apt-get update
		sudo apt-get install docker.io
	else
		echo "Error, docker is not installed in this system"
		echo "$1 was not installed"
		exit 1
	fi
fi
echo "Pulling cvar/aerostack_melodic_full docker image"
sudo docker pull cvar/aerostack_melodic_full

AEROSTACK_STACK=$(bash -i -c 'echo $AEROSTACK_STACK') #This line sources .bashrc in an interactive subshell and saves the AEROSTACK_STACK value
if [ -z $AEROSTACK_STACK ]; then
	echo "Setting .bashrc variables"
	AEROSTACK_WORKSPACE="$HOME/workspace/ros/aerostack_catkin_ws"
	AEROSTACK_STACK="$AEROSTACK_WORKSPACE/src/aerostack_stack"
	echo "export AEROSTACK_WORKSPACE=$AEROSTACK_WORKSPACE" >> ~/.bashrc
	echo "export AEROSTACK_STACK=$AEROSTACK_STACK" >> ~/.bashrc
fi

if [ ! -d "$AEROSTACK_STACK" ]; then
	# Absolute path of the aerostack workspace
	mkdir -p $AEROSTACK_STACK
	cd $AEROSTACK_STACK
	git clone -b master https://github.com/Vision4UAV/Aerostack ./
fi

PROJECTDIR=$AEROSTACK_STACK/$1
if [ ! -f "$PROJECTDIR/project_modules.txt" ]; then
	cd $AEROSTACK_STACK
	if echo $1 | grep -q "projects_cvar"; then
		git submodule update --init --recursive projects_cvar || exit 1
	else
		git submodule update --init --recursive $1 || echo "The project: $1 does not exist" && exit 1
	fi
fi

DOCKER_LAUNCHER="
if [ \$(command -v docker) ]; then
	#!/bin/bash 
	DIR=\$(pwd)
	xhost +local:root
	sudo docker run -dit --rm \\
		--name=\"$(sed 's@/@_@g' <<<$1)\" \\
		--volume=\"\$DIR:/root/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/$1\" \\
		--env=\"DISPLAY\" \\
		--env=\"QT_X11_NO_MITSHM=1\"  \\
		--volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \\
		--volume=\"/home/\$USER:/home/\$USER\" \\
		--volume=\"/etc/group:/etc/group:ro\" \\
		--volume=\"/etc/passwd:/etc/passwd:ro\" \\
		--volume=\"/etc/shadow:/etc/shadow:ro\" \\
		--volume=\"/etc/sudoers.d:/etc/sudoers.d:ro\" \\
		-e LOCAL_USER_ID=\`id -u \$USER\` \\
		-e LOCAL_GROUP_ID=\`id -g \$USER\` \\
		-e LOCAL_GROUP_NAME=\`id -gn \$USER\` \\
		-e DISPLAY=unix\$DISPLAY \\
		--device /dev/dri \\
		--privileged \\
		--volume /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \\
		cvar/aerostack_melodic_full
	ID=\$(sudo docker ps -aq --no-trunc -f \"name=\$(sed 's@/@_@g' <<<$1)\")
	sudo docker exec -it \$ID /bin/bash -c \"[ \\\"\\\$(glxinfo | grep \\\"direct rendering: Yes\\\")\\\" == \\\"direct rendering: Yes\\\" ] || (echo 'ERROR: 3D acceleration is not enabled for Docker, with the current video driver, try installing nvidia-docker' && exit 1)\" ; RES=\$?
	if [ \$RES -ne 0 ]; then
		./docker_stop.sh >/dev/null
		PROMPT=\"Do you wish to install nvidia-docker and update this script?(y/n)\"
		command -v nvidia-docker >/dev/null && NVIDIADOCKER=0 && PROMPT=\"nvidia-docker is installed, do you wish to update this script to use it?(y/n)\"
		INSTALL=0
		while [ \$INSTALL -eq 0 ]; do
			read -p \"\$PROMPT\" yn
			    case \$yn in
				[y]* ) INSTALL=1 ;;
				[n]* ) INSTALL=2 ;;
				* ) echo \"Please enter a valid option number\";;
			    esac
		done
		if [ \$INSTALL -eq 1 ] && [ -z \$NVIDIADOCKER ]; then
			distribution=\$(. /etc/os-release;echo \$ID\$VERSION_ID)
			curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
			curl -s -L https://nvidia.github.io/nvidia-docker/\$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

			sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
			sudo systemctl restart docker
		fi
		[ \$INSTALL -eq 1 ] && \$(echo -e \"\$(cat \$0 | sed 's/[^]|-]docker run/ nvidia-docker run/g' | sed 's/[^]|-]docker exec/ nvidia-docker exec/g' | sed 's/[^]|-]docker ps/ nvidia-docker ps/g')\" > docker_temp.sh) && mv docker_temp.sh \$0 && chmod +x \$0 && \$(echo -e \"\$(cat docker_stop.sh | sed 's/[^-]docker/ nvidia-docker/g')\" > docker_stop.sh) && chmod +x docker_stop.sh && ./docker_launcher.sh
	fi
	sudo docker exec -it \$ID /bin/bash -c \"gnome-terminal --disable-factory\"
	./docker_stop.sh
	xhost -local:root
else
	INSTALL=0
	while [ \$INSTALL -eq 0 ]; do
		read -p \"Docker is not installed, do you wish to install it?(y/n)\" yn
		    case \$yn in
			[y]* ) INSTALL=1 ;;
			[n]* ) INSTALL=2 ;;
			* ) echo \"Please enter a valid option number\";;
		    esac
	done
	if [ \$INSTALL -eq 1 ] && [ -z \$NVIDIADOCKER ]; then
		sudo apt-get update
		sudo apt-get install docker.io
	fi
fi"

DOCKER_STOP="
#!/bin/bash
ID=\$(sudo docker ps -aq --no-trunc -f 'name=$(sed 's@/@_@g' <<<$1)')

if [ ! -z \$ID ] && sudo docker inspect -f '{{.State.Running}}' \$ID; then
	echo 'Stopping container: '\$ID
	sudo docker stop \$ID
fi"

echo -e "$DOCKER_LAUNCHER" > $PROJECTDIR/docker_launcher.sh
sudo chmod +x $PROJECTDIR/docker_launcher.sh

echo -e "$DOCKER_STOP" > $PROJECTDIR/docker_stop.sh
sudo chmod +x $PROJECTDIR/docker_stop.sh
echo "docker_launcher.sh created in $PROJECTDIR"
