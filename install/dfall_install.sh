#!/bin/bash

die () {
    echo >&2 "$@"
    exit 1
}

# Check the input argument is supplied and correct
[ "$#" -eq 1 ] || die "1 argument required (AgentID), $# provided"
echo $1 | grep -E -q '^[0-9]+$' || die "Numeric argument required, $1 provided"

# Ensure the ROS repository is available
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Update the list of programs
sudo apt-get update
# Install all available upgrades
# > Note: the -y option means Automatic yes to prompts
sudo apt-get -y upgrade
# Install ROS Kinetic
sudo apt-get -y install ros-kinetic-desktop-full
# Install the python package management system "python-pip"
sudo apt-get -y install python-pip
# Install the python USB package "pyusb"
# > Note: this is needed to connected to the Crazyradio USB dongle
sudo pip install pyusb

# Initialise and update the ROS dependencies
sudo rosdep init
rosdep update

# Make the "work" directory under the users root
# > Note: the -p option means: no error if existing, make parent directories as needed
mkdir -p ~/work
# Change directory to this folder
cd ~/work
# Clone the D-FaLL-System git repository
git clone https://gitlab.ethz.ch/D-FaLL/PandS-System/D-FaLL-System.git

# Add the ROS environment setup to the .bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
# Run it now so that it is valid for this terminal session
source /opt/ros/kinetic/setup.bash

# Add the teacher's IP address to the /etc/hosts file
sudo sh -c "echo '10.42.0.10 dfallmaster' >> /etc/hosts"

# Add a new file with the default agent ID
sudo sh -c "echo $1 >> /etc/dfall_default_agent_id"
# Add a new file with the default coordinator ID
sudo sh -c "echo 1 >> /etc/dfall_default_coord_id"

# Copy rules necessary for using the Crazyradio
sudo cp ~/work/D-FaLL-System/install/99-crazyflie.rules /etc/udev/rules.d
sudo cp ~/work/D-FaLL-System/install/99-crazyradio.rules /etc/udev/rules.d

# Build the D-FaLL ROS Package, called dfall_pkg
# > This is done by calling "catkin_make" from the dfall_ws workspace
cd ~/work/D-FaLL-System/dfall_ws
catkin_make -j4

# Add the D-FaLL ROS Package setup to the .bashrc
echo "source ~/work/D-FaLL-System/dfall_ws/devel/setup.bash" >> ~/.bashrc
source ~/work/D-FaLL-System/dfall_ws/devel/setup.bash

# Add the "Config.sh" shell script to the .bashrc
echo "source ~/work/D-FaLL-System/dfall_ws/src/dfall_pkg/launch/Config.sh" >> ~/.bashrc
source ~/work/D-FaLL-System/dfall_ws/src/dfall_pkg/launch/Config.sh
