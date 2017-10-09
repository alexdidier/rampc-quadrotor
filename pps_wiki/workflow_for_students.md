# Workflow for students

### Prerequisite:
* Make sure you are connected to the network (cable inserted and check if connected to Vicon in the settings)
* Insert a CrazyRadio into one of your USB-ports on your Laptop
* The Crazyflie must be started on a flat surface, as the gyrosensor needs to initialize
* The Crazyflie atennas had to face the right direction (antenna facing positive x-axis) upon defining the object in ViconTracker!

---

### Files of interest:

##### -- Changeable files:
in `pps_ws/src/d_fall_pps/src`
* _CustomControllerService.cpp_ <br>
The file where students can implement their own controller. It provides already the ros service with the teacher. It can be used as a template.

in `pps_ws/src/d_fall_pps/param`
* _ClientConfig.yaml_ <br>
This file needs to be changed to define names for the custom controller. **The safeController property shouldn't be changed!** <br>
Usage: <br>
`customController: "SERVICENAME/TOPICNAME"` <br>
where SERVICENAME is the name of the cpp-file that contains the custom controller (e.g. the provided template CustomControllerService) and <br>
where TOPICNAME is the defined name for the topic which is defined insided the controller's code.
<br><br>
There are two additional values. By setting _strictSafety_ to true you limit your custom controller from assuming certain angles before the safe controller takes over. Set it to false and the safe controller only takes over when the crazyflie flies out of its defined range. <br>
The _angleMargin_ value can be used to change the acceptable angles for pitch and roll. angleMargin=1 means that your crazyflie could flip 90°. The safe controller might not recover from such angles. Therefore you should use values in the range of 0.4 to 0.7.


in `pps_ws/src/d_fall_pps/`
*  _CMakeLists.txt_ <br>
This file defines all important things that are needed for build process.
You need this file, if you for example choose to add a new controller with a new name. You then need to add several lines in this file.
Easiest way is to search for a file that exists and just add all the same things for your new file. <br>

##### -- Useful files:
in `pps_ws/src/d_fall_pps/scripts`
--> call scripts in terminal by going to the above path and then typing ./SCRIPTNAME, e.g.: `./enable_crazyflie`
* *disable_crazyflie*
* *enable_crazyflie*
* *load_custom_controller*
* *load_safe_controller*
* *safe_controller_setpoint* <br>
this one needs 4 parameters for x,y,z and yaw. The setpoint of the crazyflie is then set to those values.


##### -- Files to look at:
in `pps_ws/src/d_fall_pps/param`
* _SafeController.yaml_ <br>
This file contains the control parameters that the SafeControllerService uses. The SafeControllerService loads this file when it starts. You might want to use a similar approach and can try to copy some functionality from  SafeControllerService.cpp.

in `pps_ws/scr/d_fall_pps/launch` <br>
The launch files contained in this directory are used to launch several nodes and some parameter files to be launched simultaneously. It is best, that you take a look at them yourself, but here is a brief explanation what the different launch files are for.<br>
To start the whole thing type the following in a terminal whilst being in the launch directory.<br>
`roslaunch filename.launch`

* _Teacher.launch_<br>
This doesn't concern the students, nor will it work. This launches the GUI for the teacher and the services he needs.
* _Student.launch_<br>
This launches the nodes for the CrazyRadio, the PPSClient, SafeController and CustomController. Make sure that __ClientConfig__ is correctly set up.
<br><br>
* _StudentCirlce.launch_ : as an example<br>
This launches CircleControllerService instead of the normal CustomControllerService. Therefore the ClientConfig has to be adjusted. This should show a way of how to work with the CustomControllerService.
* _StudentFollow.launch_ : as an example<br>
As the circle launcher, this starts another service that enables one crazyflie to _copy_ the behavior of another crazyflie. For this to work, two student groups have to collaborate because some things have to manipulated manually in the cpp files of the Circle and Follow code.
---


## Workflow:
**Setup**
1.  Teacher must run his part, that publishes ViconData for students and hosts the roscore.
2.  Each student/group has a CrazyFlie and a laptop.
3.  Use `roscd d_fall_pps/launch` in a terminal as well as `roscd d_fall_pps/scripts` in another terminal

<br>
**Working**
1.  Adjust your custom controller
2.  Use `catkin_make` in the pps_ws directory to compile your controller implementation
3.  Start your crazyflie
4.  Launch the correct file in the launch directory as described above. ClientConfig.yaml has to be correct.
5. Use the scripts to change from the safe to your custom controller.
6. When your done, you can turn of your crazyflie by using the script `disable_crazyflie`.
7. Repeat


---
**Troubleshooting**
- _SafeController is not working_ <br>
Was the antenna of the crazyflie facing in the *opposite* direction of the defined Vicon x-axis? --> Define it again! <br>
The crazyflie has to lie on the table when you turn it on because the gyro sensor is initialized upon start-up. <br>
Is the crazyflie still properly showing in the ViconTracker software? --> Define it again and check that the markers don't move!
- If you have added a new controller. Don't forget to adjust the CMakeList.txt file and use catkin_make again.
