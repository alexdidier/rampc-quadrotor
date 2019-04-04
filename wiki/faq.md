# Frequently Asked Questions (FAQ)
Contents:
- [Remind me of that command again...](#remind-me-of-that-command-again)
- [Troubleshooting :-(](#troubleshooting-)
- [Control algorithm hints](#control-algorithm-hints)
- [Connect to custom buttons](#connect-to-custom-buttons)

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
roslaunch dfall_pkg Student.launch
```
This can be run from any directory because ``dfall_pkg`` is defined as an environment variable that points to the absolute foler location ``~/work/D-FaLL-System/dfall_ws/src/dfall_pkg/launch/``


### How do I make changes to a ``*.cpp`` file take effect?
In essence you need to re-compile the code and re-launch all ROS nodes.

<b>Step 1)</b> Kill all ROS processes that are running nodes from the ``D-FaLL-System`` repository, i.e., you must kill the ``Student GUI`` node but you do not need to kill an ``rqt`` plotting node. To kill a process press ``Ctrl+c`` from the terminal window that is running the process.

<b>Step 2)</b> In a terminal window, change to the ``dfall_ws`` folder of the repository (where ``ws`` stands for workspace. On a machine setup as per the instructions this is:
```
cd ~work/D-FaLL-System/dfall_ws/
``` 
<b>Step 3)</b> Compile the repository, which includes your changes, using the command:
```
catkin_make
```
This will compile all the code, or throw an error if your changes have introduced errors that prevent the compilation from completing successfully.

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
source <catkin workspace>/src/dfall_pkg/launch/Config.sh
``
And if you look at the ``Config.sh`` file in the repository you see that it defines environment variable relating to the ``ROS_MASTER_URI``, ``ROS_IP``, and ``ROS_NAMESPACE``. On occassion these are not properly defined on start up or are changed, hence closing and re-launching Terminal can resolve the problem.


### The ``Student GUI`` fails to launch due to Vicon related errors

The Vicon motion capture system provides position and attitude information via a piece of propriety software named the Datastream SDK. Hence, the proprietry software cannot be ditributed with this piece of Open Source software, and the error is likley cause by the software not being present on the local computer.

To check whether the Vicon Datastream software is properly added to the local computer, goto the [Installation](installation.md) wiki page and follow the instructions under the title "Vicon Datastream SDK Installation".

The main requirements are that:
- The ``DataStreamClient.h`` header file needs to be located in ``~/dfall_ws/src/dfall_pkg/lib/vicon/``,
- The ``libViconDataStreamSDK_CPP.so`` shared object needs to be located in ``~/dfall_ws/src/dfall_pkg/lib/vicon/``, and
- A number of files of the form ``libboost_*`` should also be located in ``~/dfall_ws/src/dfall_pkg/lib/vicon/``.



## Control algorithm hints

This section contains those Frequently Asked Questions that pertain to common coding errors in the implementation of your control algorithm that causes the [Crazyflie](https://www.bitcraze.io/) to behave in an undesirable manner (remember that computers always follow the instructions they are given).


### Why does my crazyflie drop to the ground when I request a large setpoint change in altitude?

The per motor command that your algorithm requests from the Crazyflie via setting the ``response.controlOutput.motorCmd1`` property in the function ``calculateControlOutput`` is subseuently cast as a ``uint16`` type variable. Hence you should consider saturating the per motor commands computed by your algorithm before setting the value of ``response.controlOutput.motorCmd1``.


### Why does my crazyflie stop and ``Stuent GUI`` freeze when I enable my student controller?

This is most likely caused by your code causing a segmentation fault, which is commonly caused due to a division by zero. This is often observed in relation to the calculation of sampling time from frequency. You need to think carefully about the order in which variables are instantiated, set to a value, and subsequently used in your code. Specifically, in the template controller code provided contains the ``control_frequency`` variable that is set to a value loaded from the ``.yaml`` parameter file. It is common to instantiate a new variable as ``t_s = 1.0 / control_frequency``. However, if the ``t_s`` variable is accidentally used before it is set to this value then it is possible that you are dividing by zero. Remember also that the values from the ``.yaml`` parameter file are loaded during runtime, as they can be changed at any time during the running of the application, so those values are not present during compile time of your code.


### How can we send and receive the Crazyflies position between our controllers?

In order to make information available between the laptops, you can use the publish and subscribe messaging framework availble in ROS.

In order to publish the position of your Crazyflie you need to introduce a variable of type ``ros::Publisher`` and you can use the ``Setpoint`` message type to publish the x,y,z, and yaw of your Crazyflie.

To add a publisher follow these steps:
- Add a ``ros::Publisher`` type class variable to your ``StudentControllerService.cpp`` file:
```
ros::Publisher my_current_xyz_yaw_publisher
```

- In the ``main()`` function of your ``StudentControllerService.cpp`` file initialise your publisher:
```
ros::NodeHandle nodeHandle("~");
my_current_xyz_yaw_publisher = nodeHandle.advertise<Setpoint>("/<ID>/my_current_xyz_yaw_topic", 1);
```
where ``<ID>`` should be replaced by the ID of your computer, for example the number 7, ``my_current_xyz_yaw_topic`` is the name of the message topic that will be published, and ``Setpoint`` specifies the stucture of the message as defined in the file ``/msg/Setpoint.msg`` and included with:
```
#include "dfall_pkg/Setpoint.h"
```
added at the top of  your ``StudentControllerService.cpp`` file.

- In the ``calculateControlOutput`` function of your ``StudentControllerService.cpp`` file you can publish the current position of your Crazyflie by adding the following:
```
Setpoint temp_current_xyz_yaw;
temp_current_xyz_yaw.x   = request.ownCrazyflie.x;
temp_current_xyz_yaw.y   = request.ownCrazyflie.y;
temp_current_xyz_yaw.z   = request.ownCrazyflie.z;
temp_current_xyz_yaw.yaw = request.ownCrazyflie.yaw;
my_current_xyz_yaw_publisher.publish(temp_current_xyz_yaw);
```

In order to subscribe to a message that is published in the way described above from another student's laptop, you need to define a variable of type ``ros::Subscriber`` and you specify the function that should be called each time a message is receive on the topic to which you subscribe.

To subscribe to a topic follw these steps:
- In the ``main()`` function of your ``StudentControllerService.cpp`` file initialise the subscriber:
```
ros::NodeHandle nodeHandle("~");
ros::Subscriber xyz_yaw_to_follow_subscriber = nodeHandle.subscribe("/<ID>/my_current_xyz_yaw_topic", 1, xyz_yaw_to_follow_callback);
```
where ``<ID>`` should be replaced by the ID of the computer from which you wish to subscribe, for example the number 8, ``my_current_xyz_yaw_topic`` must match the name of the topic being published, and ``xyz_yaw_to_follow_callback`` is the function in your ``StudentControllerService.cpp`` file that will be called when the message is received.

- To add the callback function, first add the following function prototype to the top of your ``StudentControllerService.cpp`` file:
```
void xyz_yaw_to_follow_callback(const Setpoint& newSetpoint);
```

- Implement the ``xyz_yaw_to_follow_callback`` function in your ``StudentControllerService.cpp`` file to achieve the behaviour desired, the following example makes the setpoint of my own Crazyflie equal to the position received in the message:
```
void xyz_yaw_to_follow_callback(const Setpoint& newSetpoint)
{
		setpoint[0] = newSetpoint.x;
		setpoint[1] = newSetpoint.y;
		setpoint[2] = newSetpoint.z;
		setpoint[3] = newSetpoint.yaw;
}
```



### How can my controller receive the position of another object recognised by the Vicon system?

The ``calculateControlOutput`` function of your ``StudentControllerService.cpp`` file is called everytime that the Vicon system provides new data, where this data is provided to the ``calculateControlOutput`` function in the ``request`` variable and the calculated control output to apply is returned in the ``response`` variable. In this way the ``calculateControlOutput`` function is performing in ROS what is referred to as a service. This service is advertised in the ``main()`` function of your ``StudentControllerService.cpp`` file by the line of code:
```
ros::ServiceServer service = nodeHandle.advertiseService("StudentController", calculateControlOutput);
```
This service is called on from the ``FlyingAgentClient.cpp`` file, and it is expected to adhere to the data structures described in ``/srv/Controller.srv``:
```
CrazyflieData ownCrazyflie
CrazyflieData[] otherCrazyflies
---
ControlCommand controlOutput
```
Hence why the position and attitude information of your own Crazyflie is accessed from inside the ``calculateControlOutput`` function via ``request.ownCrazyflie.{x,y,z,roll,pitch,yaw}``.

By default the property ``otherCrazyflies`` is left empty when the service request is constructed in the ``FlyingAgentClient.cpp``. Thus, in order to have access to the position of another object recognised by the Vicon system you can edit the ``FlyingAgentClient.cpp`` as per the following steps:
- In the ``FlyingAgentClient.cpp`` file locate the implementation of the function ``viconCallback``, which has the full prototype:
```
void viconCallback(const ViconData& viconData)
```
where the argument ``viconData`` has the structure defined in ``ViconData.msg``:
```
CrazyflieData[] crazyflies
UnlabeledMarker[] markers
```
thus ``viconData.crazyflies`` is an array of all the objects that the Vicon system is attempting to recognise, where each element of the array adheres to the following data structure:
```
string crazyflieName
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 acquiringTime #delta t
bool occluded
```

- Locate the following commented out line near the start of the ``viconCallback`` in the ``FlyingAgentClient.cpp``:
```
//m_otherObjectPoseDataIndex = getPoseDataForObjectNameWithExpectedIndex( viconData, "NameOfOtherObject" , m_otherObjectPoseDataIndex , poseDataForOtherAgent );
```

- Uncomment this line and change ``"NameOfOtherObject"`` to be the name of the object that you would like the data of. For the Crazyflies, the format used is ``"CFXX"`` where you replace ``XX`` with the number of the Crazyflie you want (zero padded), i.e., ``"CF01"`` would give you the data for Crazyflie one.

- Now the ``.otherCrazyflies`` property of the ``request`` variable that is passed to the ``calculateControlOutput`` function of your ``StudentControllerService.cpp`` file will contain the position of the ``NameOfOtherObject`` as the first entry in the array, i.e., you can access the data via ``request.otherCrazyflies[0].{x,y,z,roll,pitch,yaw}``.

- IMPORTANT: if the other object is not visible by Vicon in a particular frame, then the ``occluded`` property will indicate this, and the ``{x,y,z,roll,pitch,yaw}`` properties will have garbage values. Hence anywhere you use the data in your code, you should always check its validity with something like the following:
```
if (request.otherCrazyflies[0].occluded)
{
	// The data is garbage, do NOT use it
}
else
{
	// The data is valid and can be used
}

```



### I added some advertise, publish, subscribe topics but I am not getting the desired behaviour, what debugging tools are avilable?

If your code changes compile successfully and your node runs without crashing, then the command line tool ``rostopic`` is the most useful tool for debugging errors. Open a new terminal and type ``rostopic`` to read the desription and help files.

After launching your node, open a separate command window and type the command
```
rostopic list
```
this will list all the topics that are currently active, including both advertised and subscribed to topics.

Common pitfalls to watch out for are:
- If there are two topics with a very similar name, then double check the spelling of the topic is identical in both the ``.advertise`` and ``.subscribe`` lines of code.
- Even if the topic name is exactly matching, the name space may be different.
- If you advertise or subscribe to a topic from with a block of code enclosed by curly braces, for example:

```
ros::Subscriber my_subscriber;
if (true)
{
		ros::NodeHandle nodeHandle("~");
		my_subscriber = nodeHandle.subscribe("/fortytwo",1,fortytwoCallback);
}
```

then the subscriber only exists while the code between the ``{}`` is being exectuted and it is removed after the ``if`` statement has finished executing. To make the subscriber persist, then you need to declare the subscriber varaible outside of the ``if`` statement's context, i.e.,
```
ros::Subscriber my_subscriber;
if (true)
{
		ros::NodeHandle nodeHandle("~");
		my_subscriber = nodeHandle.subscribe("/fortytwo",1,fortytwoCallback);
}
```
This would work if it is in the ``main()`` function of a node, but if it is in another function then you may need to declare the subscriber as a class variable.

To observe all messages published on a particular topic, after launching your node, open a separate command window and type the command:
```
rostopic echo /namespace/topicname
```
where ``/namespace/topicname`` needs to be replaced with the topic that you wish to echo. All messages published to this topic will be directly printed out in the command window.





## Connect to custom buttons

The Graphical User Interface for flying your Crazyflie offers three buttons that can be used to trigger functions within your ``StudentControllerService.cpp`` file. The buttons are already connected to your code and you can respond to button presses by locating the following function within your

``StudentControllerService.cpp`` file:

```
//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D
//    C      O   O  M M M  M M M  A   A  N N N  D   D
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD
//    ----------------------------------------------------------------------------------

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButton& commandReceived)
{
	// Extract the data from the message
	int custom_button_index   = commandReceived.button_index;
	float custom_command_code = commandReceived.command_code;

	// Switch between the button pressed
	switch(custom_button_index)
	{

		// > FOR CUSTOM BUTTON 1
		case 1:
		{
			// Let the user know that this part of the code was triggered
			ROS_INFO("Custom button 1 received in controller.");
			// Code here to respond to custom button 1
			
			break;
		}

		// > FOR CUSTOM BUTTON 2
		case 2:
		{
			// Let the user know that this part of the code was triggered
			ROS_INFO("Custom button 2 received in controller.");
			// Code here to respond to custom button 2

			break;
		}

		// > FOR CUSTOM BUTTON 3
		case 3:
		{
			// Let the user know that this part of the code was triggered
			ROS_INFO_STREAM("Custom button 3 received in controller, with command code:" << custom_command_code );
			// Code here to respond to custom button 3

			break;
		}

		default:
		{
			// Let the user know that the command was not recognised
			ROS_INFO_STREAM("A custom command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.command_code = " << custom_command_code );
			break;
		}
	}
}
```

To respond to each of the button, add your code within the respective switch cases.
