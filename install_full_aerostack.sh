#!/bin/bash

#######################
# Check if dpkg database is locked and ros melodic or ros kinetic is installed
VERSION="$(rosversion -d)"
if [ -z "$VERSION" ];then
	VERSION=$ROS_DISTRO
fi
if [ ! "$VERSION" = 'melodic' ] && [ ! "$ROS_DISTRO" = 'kinetic' ]; then
	if [ -z "$VERSION" ];then
		echo "ROS is not installed"
		exit 1
	fi
	echo "ROS $VERSION is not supported, install ROS melodic or ROS kinetic"	
	exit 1
fi
sudo apt-get -y install ros-$ROS_DISTRO-mavlink &>/dev/null
if [ "$?" -ne 0 ]; then
	echo $(sudo apt-get --simulate install ros-$ROS_DISTRO-mavlink) &>/dev/null
	if [ "$?" -ne 0 ]; then
		echo "Failed to accept software from packages.ros.org, make sure you have ROS repository properly sourced and the valid key set up"
	fi
	echo "$(sudo apt-get -y install ros-$ROS_DISTRO-mavlink)"
	echo "Unable to install Aerostack ros dependencies, other process is using APT package management tool, try again after rebooting your system. Cancelling installation"
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


clear

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
echo "Downloading the Aerostack"
echo "-------------------------------------------------------"
cd $AEROSTACK_STACK
git clone -b master https://github.com/Vision4UAV/Aerostack ./


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
source /opt/ros/$ROS_DISTRO/setup.bash
cd $AEROSTACK_WORKSPACE/src
catkin_init_workspace
cd $AEROSTACK_WORKSPACE
catkin_make


echo "-------------------------------------------------------"
echo "Sourcing the ROS Aerostack WS"
echo "-------------------------------------------------------"
. ${AEROSTACK_WORKSPACE}/devel/setup.bash


echo "-------------------------------------------------------"
echo "Fetching the required git submodule"
echo "-------------------------------------------------------"
cd ${AEROSTACK_STACK}
git submodule update --init --recursive projects
git submodule update --init --recursive stack
git submodule update --init --recursive stack_deprecated


echo "-------------------------------------------------------"
echo "Installing dependencies"
echo "-------------------------------------------------------"
cd ${AEROSTACK_STACK}
./installation/install_dependencies.sh


echo "-------------------------------------------------------"
echo "Fixing CMakeLists.txt to be able to open QTCreator"
echo "-------------------------------------------------------"
cd $AEROSTACK_WORKSPACE/src
rm CMakeLists.txt
cp /opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake CMakeLists.txt


read -p "Do you want to include the component \"quadrotor_motion_with_mpc_control\". Its compilation may take a long time (e.g., 15 minutes in a regular computer) (y/n)?" choice
case "$choice" in 
  y|Y ) [ -f $AEROSTACK_STACK/stack/behaviors/behavior_packages/quadrotor_motion_with_mpc_control/CATKIN_IGNORE ] && rm $AEROSTACK_STACK/stack/behaviors/behavior_packages/quadrotor_motion_with_mpc_control/CATKIN_IGNORE;;
  n|N ) touch $AEROSTACK_STACK/stack/behaviors/behavior_packages/quadrotor_motion_with_mpc_control/CATKIN_IGNORE;;
  * ) echo "invalid";;
esac


echo "-------------------------------------------------------"
echo "Compiling the Aerostack"
echo "-------------------------------------------------------"
. ${AEROSTACK_STACK}/config/setup.sh
cd ${AEROSTACK_WORKSPACE}

[ ! -f "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion" ] && touch "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion/CATKIN_IGNORE"
catkin_make

[ -f "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion/CATKIN_IGNORE" ] && rm "$AEROSTACK_STACK/stack/behaviors/behavior_packages/multi_sensor_fusion/CATKIN_IGNORE"
catkin_make -j1
