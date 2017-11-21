# Frequently Asked Questions (FAQ)
Contents:
- Remind me of that command again...
- Troubleshooting :-(
- Control

## Remind me of that command again...

This section contain those Frequently Asked Questions that pertain to "those commands" that are regularly used to interact with the ``D-FaLL-System`` and bring your control algorithms to life through a Crazyflie quadrotor.

### How do I to get a clean version of the repository?
<b>Step 1)<\b> Change to the directory where you have a copy of the ``D-FaLL-System`` repository under version control. On a machine setup as per the instruction this will be
```
cd ~/work/D-FaLL-System
```
Step 2) It is good practice always check the status of the git repository,
``
git status
``
Step 3) 


## Troubleshooting :-(

This section contains those Frequently Asked Questions that pertain to commonly encountered errors that render the ``D-FaLL-System`` as seemingly "broken"

### The ``Student GUI`` fails to launch due to connection errors

This can happen for a variety of reasons, and generally relates to the local computer not being able to access the "ROS core" that is running on the Teacher's computer. That error can be fixed by one or all of the following step:

Step 1) Check that the network cable is actually plugged, and that the lights of the ethernet port are actually flashing (this needs to be check both for the local computer and for the Teacher's computer that is running the "ROS core")

Step 2) Check that the local computer's operating system is actually connected to the local network. This can be checked via the network icon on the right of the panel in Ubuntu (i.e., at the top right of the screen just near the battery level indicator)

Step 3) Sometimes other network connection disrupt the connection to the local network on which the ROS system communicating. Disable all other network connection via the network icon on the right of the panel in Ubuntu (i.e., disable WiFi)

Step 4) Close all terminal windows (and hence kill all processes running via a terminal window), then open a new terminal window and try to launch the Student GUI again.

To understand how Step 4) could actually fix the problem, consider that ``~/.bashrc`` is run when a new terminal window is opened, and as part of setting the ``D-FaLL-System`` the following line is added to the  ``.bashrc``:
``
source <catkin workspace>/src/d_fall_pps/launch/Config.sh
``
And if you look at the ``Config.sh`` file in the repository you see that it defines environment variable relating to the ``ROS_MASTER_URI``, ``ROS_IP``, and ``ROS_NAMESPACE``. On occassion these are not properly defined on start up or are changed, hence closing and re-launching terminal and resolve the problem.


