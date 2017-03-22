#!/usr/bin/env bash

# This script will take all necessary sources from repository, put them in
# the home folder, compile them and run the system.

# Please, make change in the repository, and test them running this script.

# WARNING! A folder called crazyflie_ws in your home directory will be removed!

ABSOLUTE_SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
DIR="$(dirname $ABSOLUTE_SCRIPT_PATH)"
echo "Script $ABSOLUTE_SCRIPT_PATH is in $DIR directory."

rm -rf ~/crazyflie_ws           # removes folder

source /opt/ros/kinetic/setup.bash

rosws init ~/crazyflie_ws /opt/ros/kinetic # creates folder with config inside

mkdir ~/crazyflie_ws/sandbox    # create sandbox directory

source ~/crazyflie_ws/setup.bash

rosws set ~/crazyflie_ws/sandbox

cd ~/crazyflie_ws/sandbox       # move to sandbox directory

roscreate-pkg crazypkg std_msgs rospy roscpp


# Copy files from repository to local folder
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/gui ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/include ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/launch ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/lib ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/log ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/msg ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/nodes ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/scripts ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/src ~/crazyflie_ws/sandbox/crazypkg
cp -r $DIR/crazyflie_ws/sandbox/crazypkg/CMakeLists.txt ~/crazyflie_ws/sandbox/crazypkg

source ~/crazyflie_ws/setup.bash
rospack profile
rosmake crazypkg                # compile sources

gnome-terminal -x sh -c "roscore; bash" # opens new terminal with roscore command
sleep 1
source ~/crazyflie_ws/setup.bash
roslaunch crazypkg crazyLaunch.launch # runs the system
