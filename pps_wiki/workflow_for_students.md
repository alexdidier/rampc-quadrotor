# Workflow for students

### Prerequisite:
* Make sure you are connected to the network (cable inserted andcheck if connected in the settings)
* Insert a CrazyRadio into one of your USB-ports on your Laptop
* The Crazyflie atennas had to face the right direction upon defining the object in ViconTracker!! (antenna in positive x direction?)


REMINDER: NEED to CHANGE THE PARAM FILE THAT INCLUDES THE CUSTOM CUNTROLLER!!!!!!
--> Templates



<br><br><br>
### Files of interest:
Their role in the workflow is described below under Workflow.

##### -- Changeable files:
in `pps_ws/src/d_fall_pps/src`
* CustomControllerService.cpp
* dsfdsf
* sdfdsf

##### -- Useful files:
in `pps_ws/src/d_fall_pps/scripts`
--> call scripts in terminal by going to the above path and then typing ./SCRIPTNAME, e.g.: `./enable_crazyflie`
* disable_crazyflie
* enable_crazyflie
* load_custom_controller
* load_safe_controller
* safe_controller_setpoint


##### -- Files to look at:
in `pps_ws/src/d_fall_pps/param`
* SafeController.yaml
in
*
*

<br><br><br>
## Workflow:
**Setup**
1.  Teacher must run his part, that publishes ViconData for students
2.  Each student/group has a CrazyFlie
3.  

<br>
**Working**
1.  Adjust your custom controller
2.  Use `make CustomController.cpp` to compile your controller implementation
3.  Start your crazyflie
4.  dsfdsfds

<br><br>
**Troubleshooting**
- SafeController is not working <br>
Was the antenna of the crazyflie facing in the *opposite* direction of the defined Vicon x-axis? --> Define it again! <br>
The crazyflie has to lie on the table when you turn it on because the gyro sensor is initialized upon start-up. <br>
Is the crazyflie still properly showing in the ViconTracker software? --> Define it again and check that the markers don't move!
