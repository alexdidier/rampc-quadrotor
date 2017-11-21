# Frequently Asked Questions (FAQ)
Contents:
- Remind me of that command again...
- Troubleshooting :-(
- Control algorithm hints

## Remind me of that command again...

This section contains those Frequently Asked Questions that pertain to "those commands" that are regularly used to interact with the ``D-FaLL-System`` and bring your control algorithms to life through a [Crazyflie](https://www.bitcraze.io/) quadrotor.

### How do I to get a clean version of the repository?
<b>Step 1)</b> Change to the directory where you have a copy of the ``D-FaLL-System`` repository under version control. On a machine setup as per the instructions this is:
```
cd ~/work/D-FaLL-System
```
<b>Step 2)</b> It is good practice always check the status of the git repository:
```
git status
```
<b>Step 3)</b> Remove all changes on the current branch since the previous pull
```
git checkout .
```
<b>Step 4)</b> Switch to the ``master`` branch of the repository
```
git checkout master
```
<b>Step 5)</b> Update to the latest version of the ``master`` branch from the remote repository
```
git pull
```

### How do I launch the ``Student GUI``?

The ``Student GUI`` can be launched from a terminal window with the following command:
```
roslaunch d_fall_pps Student.launch
```
This can be run from any directory because ``d_fall_pps`` is defined as an environment variable that points to the absolute foler location ``~/work/D-FaLL-System/pps_ws/src/d_fall_pps/launch/``


### How do I make changes to a ``*.cpp`` file take effect?
In essence you need to re-compile the code and re-launch all ROS nodes.

<b>Step 1)</b> Kill all ROS processes that are running nodes from the ``D-FaLL-System`` repository, i.e., you must kill the ``Student GUI`` node but you do not need to kill an ``rqt`` plotting node. To kill a process press ``Ctrl+c`` from the terminal window that is running the process.

<b>Step 2)</b> In a terminal window, change to the ``pps_ws`` folder of the repository (where ``ws`` stands for workspace. On a machine setup as per the instructions this is:
```
cd ~work/D-FaLL-System/pps_ws/
``` 
<b>Step 3)</b> Compile the repository, which includes your changes, using the command:
```
catkin_make
```
<b>Step 4)</b> Relaunch the Student GUI


## Troubleshooting :-(

This section contains those Frequently Asked Questions that pertain to commonly encountered errors that render the ``D-FaLL-System`` as seemingly "broken".


### The ``Student GUI`` fails to launch due to connection errors

This can happen for a variety of reasons, and generally relates to the local computer not being able to access the "ROS core" that is running on the Teacher's computer. Such an error can be fixed by one or all of the following steps:

<b>Step 1)</b> Check that the network cable is actually plugged in, and that the lights of the ethernet port are actually flashing (this needs to be checked both for the local computer and for the Teacher's computer that is running the "ROS core")

<b>Step 2)</b> Check that the local computer's operating system is actually connected to the local network. This can be checked via the network icon on the right of the panel in Ubuntu (i.e., at the top right of the screen just near the battery level indicator)

<b>Step 3)</b> Sometimes other network connections disrupt the connection to the local network on which the ROS system is communicating. Disable all other network connections via the network icon on the right of the panel in Ubuntu (i.e., disable WiFi)

<b>Step 4)</b> Check that the Teacher's computer is actually running a "ROS core".

<b>Step 5)</b> Close all terminal windows (and hence kill all processes running via a terminal window), then open a new terminal window and try to launch the Student GUI again.

To understand how Step 5) could actually fix the problem, consider that ``~/.bashrc`` is run when a new terminal window is opened, and as part of installing the ``D-FaLL-System`` the following line is added to the  ``.bashrc``:
``
source <catkin workspace>/src/d_fall_pps/launch/Config.sh
``
And if you look at the ``Config.sh`` file in the repository you see that it defines environment variable relating to the ``ROS_MASTER_URI``, ``ROS_IP``, and ``ROS_NAMESPACE``. On occassion these are not properly defined on start up or are changed, hence closing and re-launching Terminal can resolve the problem.


### The ``Student GUI`` fails to launch due to Vicon related errors

The Vicon motion capture system provides position and attitude information via a piece of propriety software named the Datastream SDK. Hence, the proprietry software cannot be ditributed with this piece of Open Source software, and the error is likley cause by the software not being present on the local computer.

To check whether the Vicon Datastream software is properly added to the local computer, goto the [Installation](installation.md) wiki page and follow the instructions under the title "Vicon Datastream SDK Installation".

The main requirements are that:
- The ``DataStreamClient.h`` header file needs to be located in ``~/pps_ws/src/d_fall_pps/lib/vicon/``,
- The ``libViconDataStreamSDK_CPP.so` shared object needs to be located in ``~/pps_ws/src/d_fall_pps/lib/vicon/``, and
- A number of files of the form ``libboost_*`` should also be located in ``~/pps_ws/src/d_fall_pps/lib/vicon/``.



## Control algorithm hints

This section contains those Frequently Asked Questions that pertain to commonly coding errors in the implementation of your control algorithm that cause the [Crazyflie](https://www.bitcraze.io/) to behave unexpectedly (remember that computers always follow the instructions they are given).


### Why does my crazyflie drop to the ground when I request a large setpoint change in altitude?

The per motor command that your algorithm requests from the Crazyflie via setting the ``response.controlOutput.motorCmd1`` property in the function ``calculateControlOutput`` is subseuently cast as a ``uint16`` typer variable. Hence you should consider saturating the per motor commands computed by your algorithm before setting the value of ``response.controlOutput.motorCmd1``.


### Why does my crazyflie stop and ``Stuent GUI`` freeze when I enable my custom controller?

This is most likely caused by your code causing a segmentation fault, which is commonly caused due to a division by zero. This have been commonly observed in relation to the computation of sampling time from frequency. You need to think carefully about the order in which variables are instantiated, give a value, and subsequently used in your code. Specifically, in the tempmlate controller provided the ``control_frequency`` variable is set to the value loaded from the ``.yaml'' parameter file, and it is common to instantiate a new variable as ``ts = 1.0 / control_frequency``. However, if ``ts`` is accidentally used before it is set to this value then it is possible that you are dividing by zero.

