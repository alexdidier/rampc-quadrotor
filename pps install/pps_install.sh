#!/bin/bash

die () {
    echo >&2 "$@"
    exit 1
}

[ "$#" -eq 1 ] || die "1 argument required (StudentID), $# provided"
echo $1 | grep -E -q '^[0-9]+$' || die "Numeric argument required, $1 provided"

#ros repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

#system update and installation
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install ros-kinetic-desktop-full

#rosdep
sudo rosdep init
rosdep update

#untar catkin workspace
#needs to run after ros installation because of symbolic link to CMakeLists.txt
mkdir -p ~/pps_ws/src
tar -xf package.tar.gz -C ~/pps_ws/src

#environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source /opt/ros/kinetic/setup.bash

sudo sh -c "echo '10.42.0.32	teacher' >> /etc/hosts"
sudo sh -c "echo $1 >> /etc/StudentID"

#build workspace
cd ~/pps_ws
catkin_make -j4

echo "source ~/pps_ws/devel/setup.bash" >> ~/.bashrc
source ~/pps_ws/devel/setup.bash
echo "source ~/pps_ws/src/d_fall_pps/launch/Config.sh" >> ~/.bashrc
source ~/pps_ws/src/d_fall_pps/launch/Config.sh

sudo mv ./99-crazyflie.rules /etc/udev/rules.d
sudo mv ./99-crazyradio.rules /etc/udev/rules.d
