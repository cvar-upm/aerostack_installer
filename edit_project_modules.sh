#!/bin/bash

#######################
# Check if dpkg database is locked and ros melodic or ros kinetic is installed
VERSION="$(rosversion -d)"
if [ -z "$VERSION" ];then
	VERSION=$ROS_DISTRO
fi
if [ ! "$VERSION" = 'melodic' ] && [ ! "$ROS_DISTRO" = 'kinetic' ]; then
	if [ -z "$VERSION" ];then
		echo "Ros is not installed"
		exit 1
	fi
	echo "Ros $VERSION is not supported"
	exit 1
fi
sudo apt-get -y install ros-$ROS_DISTRO-mavlink &>/dev/null
if [ "$?" -ne 0 ]; then
	echo $(sudo apt-get --simulate install ros-$ROS_DISTRO-mavlink) &>/dev/null
	if [ "$?" -ne 0 ]; then
		echo "Failed to accept software from packages.ros.org"
	fi
	echo "$(sudo apt-get -y install ros-$ROS_DISTRO-mavlink)"
	echo "Unable to install Aerostack ros dependencies, cancelling installation"
	exit 1
fi

# Absolute path of the aerostack workspace
AEROSTACK_WORKSPACE="$HOME/workspace/ros/aerostack_catkin_ws"
AEROSTACK_STACK="$AEROSTACK_WORKSPACE/src/aerostack_stack"
export AEROSTACK_WORKSPACE=$AEROSTACK_WORKSPACE
export AEROSTACK_STACK=$AEROSTACK_STACK

if [[ `lsb_release -rs` == "18.04" ]]; then
	ROS_DISTRO="melodic"
else
	ROS_DISTRO="kinetic"
fi
export ROS_DISTRO=$ROS_DISTRO

if [ ! -d $AEROSTACK_STACK/.git ]; then
	echo "------------------------------------------------------"
	echo "Obtaining aerostack git info and root source code"
	echo "------------------------------------------------------"
	[ -d $AEROSTACK_WORKSPACE/.temp ] && rm -r -f $AEROSTACK_WORKSPACE/.temp
	mkdir -p $AEROSTACK_WORKSPACE/.temp
	mkdir -p $AEROSTACK_STACK
	cd $AEROSTACK_WORKSPACE/.temp
	git clone -b master https://github.com/Vision4UAV/Aerostack ./
	cp -r $AEROSTACK_WORKSPACE/.temp/. $AEROSTACK_STACK/
	rm -rf $AEROSTACK_WORKSPACE/.temp
else
	echo "------------------------------------------------------"
	echo "Updating git info"
	echo "------------------------------------------------------"
	cd $AEROSTACK_STACK
	git pull origin master
fi

# Path of the project to edit
PROJECTDIR=$AEROSTACK_STACK/$1
if [ ! -f "$PROJECTDIR/project_modules.txt" ] || [ ! "$(ls -A $PROJECTDIR)" ]; then
	echo "------------------------------------------------------"
	echo "Downloading the project"
	echo "------------------------------------------------------"
	cd $AEROSTACK_STACK
	if echo $1 | grep -q "projects_cvar"; then
		git submodule update --init --recursive projects_cvar && python $AEROSTACK_STACK/installation/project_customizer.py $AEROSTACK_STACK/ $1/ || exit 1
	else
		git submodule update --init --recursive $1 && python $AEROSTACK_STACK/installation/project_customizer.py $AEROSTACK_STACK/ $1/ ||
		while : ; do  
    		read -n1 -p "The project: $1 does not exist in Aerostack. Do you want to create it? (Y/n)" varname
			echo
    		case "$varname" in
        		y|Y|"") python $AEROSTACK_STACK/installation/project_customizer.py $AEROSTACK_STACK/ $1/; break ;; 
        		n|N) echo "The project: $1 won't be created. Exiting"; exit 1 ;; 
        		*) echo "Invalid option given." ;;
    		esac
		done 
	fi
else 
	python $AEROSTACK_STACK/installation/project_customizer.py $AEROSTACK_STACK/ $1/
fi
echo "------------------------------------------------------"
echo "Installing project modules from project_modules.txt"
echo "------------------------------------------------------"
if [ -f "$PROJECTDIR/project_modules.txt" ]; then
	cd $AEROSTACK_STACK
	DIRECTORIES=$(cat $PROJECTDIR/project_modules.txt | sed -n '/path.*=.*/p' | sed 's/path.*=//g')
	git submodule update --init --jobs=$(echo "$DIRECTORIES" | wc -l) $DIRECTORIES
else
	echo "ERROR: The project: $1 does not contain the required file \"project_modules.txt\"" && exit 1
fi

echo "------------------------------------------------------"
echo "Creating the ROS Workspace"
echo "------------------------------------------------------"
source /opt/ros/melodic/setup.bash
cd $AEROSTACK_WORKSPACE/src
catkin_init_workspace
cd $AEROSTACK_WORKSPACE
catkin_make

echo "-------------------------------------------------------"
echo "Sourcing the ROS Aerostack WS"
echo "-------------------------------------------------------"
. ${AEROSTACK_WORKSPACE}/devel/setup.bash

echo "-------------------------------------------------------"
echo "Fixing CMakeLists.txt to be able to open QTCreator"
echo "-------------------------------------------------------"
cd $AEROSTACK_WORKSPACE/src
rm CMakeLists.txt
cp /opt/ros/melodic/share/catkin/cmake/toplevel.cmake CMakeLists.txt

echo "-------------------------------------------------------"
echo "Installing dependencies"
echo "-------------------------------------------------------"
cd $AEROSTACK_STACK/installation
./install_dependencies.sh

echo "-------------------------------------------------------"
echo "Compiling the Aerostack"
echo "-------------------------------------------------------"
. ${AEROSTACK_STACK}/setup.sh
cd ${AEROSTACK_WORKSPACE}
catkin_make

grep -q "source $AEROSTACK_WORKSPACE/devel/setup.bash" $HOME/.bashrc || echo "source $AEROSTACK_WORKSPACE/devel/setup.bash" >> $HOME/.bashrc
sed -i '/export AEROSTACK_STACK/d' $HOME/.bashrc && echo "export AEROSTACK_STACK=$AEROSTACK_WORKSPACE/src/aerostack_stack" >> $HOME/.bashrc
sed -i '/export AEROSTACK_WORKSPACE/d' $HOME/.bashrc && echo "export AEROSTACK_WORKSPACE=$AEROSTACK_WORKSPACE" >> $HOME/.bashrc
sed -i '/export LD_LIBRARY_PATH/d' $HOME/.bashrc && echo "export LD_LIBRARY_PATH=$AEROSTACK_WORKSPACE/devel/lib:/opt/ros/$ROS_DISTRO/lib:$AEROSTACK_WORKSPACE/devel/lib/parrot_arsdk" >> $HOME/.bashrc
