#!/bin/bash

#######################
# The user should fill this variables

# Absolute path of the aerostack workspace
AEROSTACK_WS_PATH="$HOME/prueba/workspace/ros/quadrotor_stack_catkin_ws"

# Relative path of the aerostack. The parent path is ${AEROSTACK_WS_PATH}/src
AEROSTACK_STACK_SUBPATH="quadrotor_stack"

# ROS DISTRO
ROS_DISTRO="jade"


#
# End of user configuration
##################################

clear

echo "------------------------------------------------------"
echo "Setting Aerostack Paths"
echo "------------------------------------------------------"
AEROSTACK_PATH="$AEROSTACK_WS_PATH/src/$AEROSTACK_STACK_SUBPATH"
echo "-The Aerostack WS path is: $AEROSTACK_WS_PATH"
echo "-The Aerostack path is: $AEROSTACK_PATH"

DRONE_WORKSPACE=$AEROSTACK_WS_PATH
DRONE_STACK=$AEROSTACK_PATH


echo "------------------------------------------------------"
echo "Removing previous installs of Aerostack (If Installed)"
echo "------------------------------------------------------"
sed -i '/DRONE_WORKSPACE/d' $HOME/.bashrc
sed -i '/DRONE_STACK/d' $HOME/.bashrc


echo "------------------------------------------------------"
echo "Creating the ROS Workspace for Downloading the Aerostack and the Aerostack path"
echo "------------------------------------------------------"
mkdir -p $AEROSTACK_WS_PATH/src
mkdir -p $AEROSTACK_PATH


echo "-------------------------------------------------------"
echo "Downloading the Aerostack"
echo "-------------------------------------------------------"
cd ${AEROSTACK_PATH}
git clone -b master https://github.com/Vision4UAV/Aerostack ./


echo "--------------------------------------------------------------------"
echo "Installing the environment Variables DRONE_WORKSPACE and DRONE_STACK"
echo "--------------------------------------------------------------------"
cd $AEROSTACK_WS_PATH
$AEROSTACK_PATH/installation/installers/installWS.sh

cd $AEROSTACK_PATH
./installation/installers/installStack.sh

source ~/.bashrc

echo "------------------------------------------------------"
echo "Creating the ROS Workspace"
echo "------------------------------------------------------"
source /opt/ros/$ROS_DISTRO/setup.bash
cd $DRONE_WORKSPACE/src
catkin_init_workspace
cd $DRONE_WORKSPACE
catkin_make


echo "-------------------------------------------------------"
echo "Sourcing the ROS Aerostack WS"
echo "-------------------------------------------------------"
. ${DRONE_WORKSPACE}/devel/setup.bash


echo "-------------------------------------------------------"
echo "Fetching the required git submodule"
echo "-------------------------------------------------------"
cd ${DRONE_STACK}
#./installation/gitSubmoduleUpdateInit.sh installation/configSubmodules.cfg
git submodule update --init


echo "-------------------------------------------------------"
echo "Installing dependencies"
echo "-------------------------------------------------------"
cd ${DRONE_STACK}
./installation/installation_dep_script.sh


echo "-------------------------------------------------------"
echo "Fixing CMakeLists.txt to be able to open QTCreator"
echo "-------------------------------------------------------"
cd $DRONE_WORKSPACE/src
rm CMakeLists.txt
cp /opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake CMakeLists.txt


echo "-------------------------------------------------------"
echo "Compiling the Aerostack"
echo "-------------------------------------------------------"
. ${DRONE_STACK}/setup.sh
cd ${DRONE_WORKSPACE}
catkin_make



