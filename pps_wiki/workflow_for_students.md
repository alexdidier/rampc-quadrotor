# Workflow for students

### Prerequisite:
* Make sure you are connected to the network (cable inserted and check if connected to Vicon in the settings)
* Insert a CrazyRadio into one of your USB-ports on your Laptop
* The Crazyflie atennas had to face the right direction (antenna facing positive x-axis) upon defining the object in ViconTracker!!

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

##### -- Useful files:
in `pps_ws/src/d_fall_pps/scripts`
--> call scripts in terminal by going to the above path and then typing ./SCRIPTNAME, e.g.: `./enable_crazyflie`
* *disable_crazyflie*
* *enable_crazyflie*
* *load_custom_controller*
* *load_safe_controller*
* *safe_controller_setpoint*


##### -- Files to look at:
in `pps_ws/src/d_fall_pps/param`
* _SafeController.yaml_ <br>
This file contains the control parameters that the SafeControllerService uses. The SafeControllerService loads this file when it starts. You might want to use a similar approach and can try to copy some functionality from  SafeControllerService.cpp.

---


## Workflow:
**Setup**
1.  Teacher must run his part, that publishes ViconData for students and hosts the roscore.
2.  Each student/group has a CrazyFlie
3.  

<br>
**Working**
1.  Adjust your custom controller
2.  Use `make CustomController.cpp` to compile your controller implementation
3.  Start your crazyflie
4.  Go to

<br><br>
**Troubleshooting**
- _SafeController is not working_ <br>
Was the antenna of the crazyflie facing in the *opposite* direction of the defined Vicon x-axis? --> Define it again! <br>
The crazyflie has to lie on the table when you turn it on because the gyro sensor is initialized upon start-up. <br>
Is the crazyflie still properly showing in the ViconTracker software? --> Define it again and check that the markers don't move!
- Config.sh sourced? <br>
