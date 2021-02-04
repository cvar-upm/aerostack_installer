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
AEROSTACK_WS_PATH="$HOME/workspace/ros/aerostack_catkin_ws"

# Relative path of the aerostack. The parent path is ${AEROSTACK_WS_PATH}/src
AEROSTACK_STACK_SUBPATH="aerostack_stack"

if [[ `lsb_release -rs` == "18.04" ]]
then
	ROS_DISTRO="melodic"
else
	ROS_DISTRO="kinetic"
fi

# Path of the project to install
PROJECT_DIR=$1

# Configuration flag
CONFIGURE=$2

clear

if [ ! -d "$AEROSTACK_STACK" ];
then
echo "------------------------------------------------------"
echo "Setting Aerostack Paths"
echo "------------------------------------------------------"
AEROSTACK_WORKSPACE=$AEROSTACK_WS_PATH
AEROSTACK_STACK="$AEROSTACK_WS_PATH/src/$AEROSTACK_STACK_SUBPATH"
echo "-The Aerostack WS path is: $AEROSTACK_WORKSPACE"
echo "-The Aerostack stack path is: $AEROSTACK_STACK"

echo "------------------------------------------------------"
echo "Removing previous installs of Aerostack (If Installed)"
echo "------------------------------------------------------"
rm -rf $AEROSTACK_STACK
rm -rf $AEROSTACK_WORKSPACE
sed -i '/AEROSTACK_WORKSPACE/d' $HOME/.bashrc
sed -i '/AEROSTACK_STACK/d' $HOME/.bashrc

echo "------------------------------------------------------"
echo "Creating the ROS Workspace for Downloading the Aerostack and the Aerostack path"
echo "------------------------------------------------------"
mkdir -p $AEROSTACK_WORKSPACE/src
mkdir -p $AEROSTACK_STACK

echo "-------------------------------------------------------"
echo "Downloading the Aerostack directory tree"
echo "-------------------------------------------------------"
cd $AEROSTACK_STACK
git clone -b master https://github.com/cvar-upm/aerostack ./

echo "--------------------------------------------------------------------"
echo "Installing the environment Variables AEROSTACK_WORKSPACE and AEROSTACK_STACK"
echo "--------------------------------------------------------------------"
cd $AEROSTACK_WORKSPACE
$AEROSTACK_STACK/installation/installers/installWS.sh

cd $AEROSTACK_STACK
./installation/installers/installStack.sh

export AEROSTACK_WORKSPACE
export AEROSTACK_STACK

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
fi

echo "-----------------------------------------------------------"
echo "Fetching the required projects_cvar and projects submodules"
echo "-----------------------------------------------------------"
cd ${AEROSTACK_STACK}

if [ ! -d "$AEROSTACK_STACK/projects_cvar" ] || [ ! "$(ls -A $AEROSTACK_STACK/projects_cvar)" ];
then
git submodule update --init --recursive projects_cvar
fi

git submodule update --init --recursive $1

echo "-------------------------------------------------------"
echo "Installing project"
echo "-------------------------------------------------------"
if [ "$CONFIGURE" != "-c" ] && [ ! -d "$AEROSTACK_STACK/$PROJECT_DIR" ];
then
    echo "The project does not exist"
    exit
else 
    mkdir -p $AEROSTACK_STACK/$PROJECT_DIR
    cd ${AEROSTACK_STACK}/installation
    python project_customizer.py $AEROSTACK_STACK/ $PROJECT_DIR/ $CONFIGURE
    cd ${AEROSTACK_STACK}
    echo "Installing selected project..."
    sh update_project_modules.sh
fi

echo "-------------------------------------------------------"
echo "Installing dependencies"
echo "-------------------------------------------------------"
cd ${AEROSTACK_STACK}
./installation/install_dependencies.sh

echo "-------------------------------------------------------"
echo "Compiling the Aerostack"
echo "-------------------------------------------------------"
. ${AEROSTACK_STACK}/setup.sh
cd ${AEROSTACK_WORKSPACE}
[ ! -f "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion" ] && touch "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion/CATKIN_IGNORE"
catkin_make

[ -f "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion/CATKIN_IGNORE" ] && rm "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion/CATKIN_IGNORE"
catkin_make -j1



