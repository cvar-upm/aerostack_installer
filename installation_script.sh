#!/bin/bash

#######################
# The user should fill this variables

# Absolute path of the aerostack workspace
AEROSTACK_WS_PATH="$HOME/workspace/ros/aerostack_catkin_ws"

# Relative path of the aerostack. The parent path is ${AEROSTACK_WS_PATH}/src
AEROSTACK_STACK_SUBPATH="aerostack_stack"

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

AEROSTACK_WORKSPACE=$AEROSTACK_WS_PATH
AEROSTACK_STACK=$AEROSTACK_PATH


echo "------------------------------------------------------"
echo "Removing previous installs of Aerostack (If Installed)"
echo "------------------------------------------------------"
sed -i '/AEROSTACK_WORKSPACE/d' $HOME/.bashrc
sed -i '/AEROSTACK_STACK/d' $HOME/.bashrc


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
git submodule update --init --recursive configs
git submodule update --init --recursive documentation
git submodule update --init --recursive etc
git submodule update --init --recursive installation
git submodule update --init --recursive launchers
git submodule update --init --recursive logs

git submodule update --init --recursive stack
git submodule update --init --recursive stack_deprecated
#git submodule update --init --recursive stack_devel
#git submodule update --init --recursive stack_obsolete


echo "-------------------------------------------------------"
echo "Installing dependencies"
echo "-------------------------------------------------------"
cd ${AEROSTACK_STACK}
./installation/installation_dep_script.sh


echo "-------------------------------------------------------"
echo "Fixing CMakeLists.txt to be able to open QTCreator"
echo "-------------------------------------------------------"
cd $AEROSTACK_WORKSPACE/src
rm CMakeLists.txt
cp /opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake CMakeLists.txt


echo "-------------------------------------------------------"
echo "Compiling the Aerostack"
echo "-------------------------------------------------------"
. ${AEROSTACK_STACK}/setup.sh
cd ${AEROSTACK_WORKSPACE}
catkin_make



