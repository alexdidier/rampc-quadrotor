# Installation

## For Student and Teacher

### Install Script
Installation with the install script is the easiest. You will need:
- the install script
- the ``d_fall_pps`` package compressed in a file called ``package.tar.gz``
- the rule files for the USB connection to the crazyradio, called ``99-crazyflie.rules`` and ``99-crazyradio.rules``

These files all need to be in the same directory. To run the installation, move to the containing directory and call it with
```
./pps_install.sh <student id>
```
The student id needs to be a unique number that is used as identication for the student laptops connected to the teacher. Make sure not that the script file is marked executable and to not run the script as root, as it will ask for the password and only execute some commands with root privilege.

### Manual Installation
The installation process consists of the following steps:

- Installation of ROS: <br />
The detailed instructions for the installation of ROS can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

- Workspace: <br />
Create a new catkin workspace and copy the ``d_fall_pps`` package into the ``src`` folder of the workspace. Then build the package with ``catkin_make`` called from the workspace root.

- Environment Setup: <br />
Add a new line in the ``/etc/hosts`` file that links the teacher's IP with the domain name ``teacher`` and create a file called ``/etc/StudentID`` that contains the student id. Only write digits without any other symbols or whitespace characters.

- USB Crazyradio: <br />
To set up the crazyradio USB dongle just copy the rule files ``99-crazyflie.rules`` and ``99-crazyradio.rules`` to the directory ``/etc/udev/rules.d``.
- Source scripts in ``.bashrc``: <br />
You need to source the following scripts in the ``.bashrc`` file:
  - the ROS setup script: ``/opt/ros/kinetic/setup.bash``
  - the workspace setup script: ``<catkin workspace>/devel/setup.bash``
  - the student setup script: ``<catkin workspace>/src/d_fall_pps/launch/Config.sh``

The workspace setup script will only appear after the first compilation of the catkin workspace.

If you are not sure at any point you can check out the install script.

## Additional Steps for Teacher
### Installation of cfclient
The steps to install the crazyflie client are taken from [here](https://github.com/bitcraze/crazyflie-clients-python). To install the cfclient you need to install its dependencies:
```
sudo apt-get install python3 python3-pip python3-pyqt5
```
clone the git repository, change the directory to the root directory of the repository and execute
```
pip3 install -e .
```
