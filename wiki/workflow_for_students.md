# Workflow for students.

### Hardware prerequisites:
* Make sure you are connected to the network (cable inserted and check if connected to Vicon in the settings).
* Insert a CrazyRadio into one of your USB-ports on your Laptop.
* Start your crazyflie. To know which one is the one you have been linked to,
ask your teacher. The Crazyflie must be started on a flat surface, as the
gyrosensor needs to initialize.

### Software prerequisites:
  * In the software side, everything has already been set up for the course, but
    it would be helpful to check if the repository is in the last version, and
    if the source code has been properly compiled. To do this, follow the next steps:
       1. Go to the next folder: `cd ~/work/D-FaLL-System/pps_ws`
       2. Checkout master branch of the repository and pull:<br />
         ``git checkout master``<br />
         ``git pull origin master``<br />
       *Note: to do this step, you will be asked a username and a password. Use
       the same credentials you use for your ETH account. Also, make sure you
       have an active account in gitlab: [https://gitlab.ethz.ch/](https://gitlab.ethz.ch/)*<br />
       3. Compile the source code running `catkin_make`

---

### Start the student's GUI

  * Once all the prerequisites have been fulfilled, we can start the student's
    GUI by going to a terminal and typing:
    `roslaunch d_fall_pps Student.launch`

    *Note: for this to work, the teacher's computer has to be connected to the
    network and the teacher's GUI has to be started before. Please wait until
    your teacher has already set up everything.*

  * Once started, you will see something like this: <br>

    <img src="./pics/student_gui.png" style="width: 800px;"/> <br><br>

  * Connect to/Disconnect from Crazyflie: physically connects/disconnects our computer to
  the assigned crazyflie using the Crazyradio dongle.<br><br>
    * Crazyradio status: can take the values "Connected!", "Disconnected" or
      "Connecting..." <br><br>
    * Below the disconnect button we see two lines of text. The first one
  contains information about our StudentID number, and the Crazyflie we are
  linked to. This is the Crazyflie we must connect to. <br><br>
  Below that, we can also see the Raw voltage line, which contains the instantaneous voltage of the
  battery of the Crazyflie, in Volts.<br><br>
    * In the right part, there are 3 buttons to control the flying state of our
  Crazyflie, and a text label containing the current flying state. **It is
  important to know that we can only take off when we are in the state "Motors
  OFF", and we can only land if we are NOT in the state "Motors OFF"**<br><br>
    * In the middle-bottom part of the GUI there are two tabs: Safe and Custom
  controller. <br><br>
  In the left part of these tabs, there is information about the
  current position of the Crazyflie, and also the difference between the current
  position and the current setpoint, useful for keeping track of the error of
  our controller.<br><br>
  In the right part of the tabs, there is information about the current
  setpoint, and boxes to edit the new setpoint. When we press the button "Set
  setpoint", we change the current setpoint with the information filled.<br><br>
    * The button called "Enable <controller> Controller" enables the selected
  controller. The current enabled controller is the one which is highlighted in
  green in the tab name.<br><br>
    * The button "Load <filename> YAML file" loads and refreshes the parameters
  that are in the corresponding YAML file.<br><br>


  * You can now play with the landing, take off and change of setpoint using the
    safe controller to get familiar with the system.

    *Note: there are different parameters in the file called
    `SafeController.yaml`, in the folder param (use `roscd d_fall_pps/param` in a
    terminal to go there).* **These are the safe controller parameters and should NOT be
    changed.** *Take a look at the file and get familiar with the format used,
    since may have to write your own for the custom controller.*

### Creating your own custom controller!
In this part, we will learn how to design our own custom controller and actually
deploy it and see it working in our Crazyflie. For this, there are a set of
important files that should be taken into account.

#### Files of interest:

* In `d_fall_pps/src`

  * _CustomControllerService.cpp_ <br>
  The file where students can implement their own controller. It provides already the ros service with the teacher. It can be used as a template.

* In `d_fall_pps/param`

  * _ClientConfig.yaml_ <br><br>
          <img src="./pics/client_config_yaml.png" style="width: 400px;"/> <br><br>
  This file needs to be changed to define names for the custom controller. **The safeController property shouldn't be changed!** <br>

      *Usage:*

      *`customController: "SERVICENAME/TOPICNAME"`*

      *where SERVICENAME is the name of the cpp-file that contains the custom controller (e.g. the provided template CustomControllerService) and
      where TOPICNAME is the defined name for the topic which is defined insided
      the controller's code.*

      *`strictSafety: <bool>`*

      *By setting _strictSafety_ to true you limit
      your custom controller from assuming certain angles before the safe controller
      takes over. Set it to false and the safe controller only takes over when the
      crazyflie flies out of its defined range.*

      *`angleMargin: <float>`*

      *This value can be used to change the acceptable angles for pitch
      and roll. angleMargin=1 means that your crazyflie could flip 90°. The safe
      controller might not recover from such angles. Therefore you should use values
      in the range of 0.4 to 0.7.*

      *`battery_threshold_while....`*

      *Sets the low battery thresholds at which the crazyflie will automatically
      land. **Do not change these values.***

* In `d_fall_pps/`

  * CMakeLists.txt

      *This file defines all important things that are needed for build process.
      You need this file, if you for example choose to add a new controller with a new name. You then need to add several lines in this file.
      The easiest way is to search for a file that exists and just add all the
      same things for your new file.*



<!-- ##### -- Useful files: -->
<!-- in `pps_ws/src/d_fall_pps/scripts` -->
<!-- -\-> call scripts in terminal by going to the above path and then typing -->
<!-- ./SCRIPTNAME, e.g.: `./enable_crazyflie` -->

<!--   * *disable_crazyflie* -->
<!--   * *enable_crazyflie* -->
<!--   * *load_custom_controller* -->
<!--   * *load_safe_controller* -->
<!--   * *safe_controller_setpoint* <br> -->
<!--   This one needs 4 parameters for x,y,z and yaw. The setpoint of the crazyflie is then set to those values. -->


<!-- ##### -- Files to look at: -->
<!-- in `pps_ws/src/d_fall_pps/param` -->
<!-- * _SafeController.yaml_ <br> -->
<!-- This file contains the control parameters that the SafeControllerService uses. The SafeControllerService loads this file when it starts. You might want to use a similar approach and can try to copy some functionality from  SafeControllerService.cpp. -->

<!-- in `pps_ws/scr/d_fall_pps/launch` <br> -->
<!-- The launch files contained in this directory are used to launch several nodes and some parameter files to be launched simultaneously. It is best, that you take a look at them yourself, but here is a brief explanation what the different launch files are for.<br> -->
<!-- To start the whole thing type the following in a terminal whilst being in the launch directory.<br> -->
<!-- `roslaunch filename.launch` -->

<!--   * _Teacher.launch_<br> -->
<!--   This doesn't concern the students, nor will it work. This launches the GUI for the teacher and the services he needs. -->
<!--   * _Student.launch_<br> -->
<!--   This launches the nodes for the CrazyRadio, the PPSClient, SafeController and CustomController. Make sure that __ClientConfig__ is correctly set up. -->
<!--   <br><br> -->
<!--   * _StudentCirlce.launch_ : as an example<br> -->
<!--   This launches CircleControllerService instead of the normal CustomControllerService. Therefore the ClientConfig has to be adjusted. This should show a way of how to work with the CustomControllerService. -->
<!--   * _StudentFollow.launch_ : as an example<br> -->
<!--   As the circle launcher, this starts another service that enables one crazyflie to _copy_ the behavior of another crazyflie. For this to work, two student groups have to collaborate because some things have to manipulated manually in the cpp files of the Circle and Follow code. -->

#### Steps to create a custom controller
1. Open the file `CustomControllerService.cpp` and go through it. You should see
   that the file is already filled with some code. This code is a simple LQR
   controller that is offered as a template for the student to start developing
   their own controller. Change the file as you wish with your own controller
   algorithm. The function partially shown below is the most important part of
   our this file:<br><br>
        <img src="./pics/custom_controller_src.png" style="width: 700px;"/> <br><br>

     In the template you can also see an example of how to use the
     `CustomController.yaml` to load parameters that you may want to change
     without having to compile or restart the system. Here, as an example, we
     pass some parameters that can be seen below:<br><br>
        <img src="./pics/custom_controller_yaml.png" style="width: 400px;"/> <br><br>
2. Go to `cd ~/work/D-FaLL-System/pps_ws` and write `catkin_make`.

3. Once everything has compiled without errors, run the next launch file:
   `roslaunch d_fall_pps Student.launch`. This will run the student's GUI.

4. Make sure that your Crazyflie is turned ON and connect to it. Choose the tab
   called *Custom Controller* and click on the button *Enable Custom Controller*

5. Press the *Take Off* button and you should see your crazyflie taking OFF.

      *Note: for take off and landing, the crazyflie uses the safe controller. Once we
finish taking off, if the custom controller was enabled, we automatically switch
to it*

6. If everything went correctly, now your Crazyflie is using your own
   controller!

      *Note: if your controller is unstable, or it does not compute a valid
   output, your crazyflie will start moving in a wrong way. Please be careful
   and take care, since the crazyflie can unexpectedly go and hit the ceiling,
   the wall, or other student. If your crazyflie
   goes out of the assigned zone, or if the pitch or roll go out of the safety
   margin (when strict safety is enabled), the safe controller will be
   automatically enabled to try to recover the crazyflie and bring it to the
   center of your zone.*

7. If you decided to put some parameter in the `CustomController.yaml` file, you
   can now change it and press the *Load CustomController YAML file* button in
   the GUI. This way, you can try and see the effect of changing some parameters
   on the fly.


#### Steps to plot debug variables from Custom Controller in a graph

1. Choose the variables that we want to see in a plot from the file
   `CustomControllerService.cpp`. Inside the function `calculateControlOutput`,
   a part where we fill a DebugMsg with data has been written (lines 133-145 in
   previous figure). Vicon data has already been filled in (vicon\_x,
   vicon\_y,...). Additionally, there are up to 10 general purpose variables
   that can be filled with any data we may be interested in plotting (value\_1,
   value\_2,...., value\_10). <br><br>
2. Once chosen the variables, save the file and go to `cd ~/work/D-FaLL-System/pps_ws` and write `catkin_make`.<br><br>
3. Open another terminal and type `rqt`. Then, in the top bar, go to
   Plugins->Visualization->Plot. A new plot will be added to the screen. If you
   want more than one plot, just add several ones doing the same thing. You will
   be seeing something like this:<br><br>

    <img src="./pics/rqt_window_subplots.png" style="width: 400px;"/> <br><br>
4. In each subplot, to add data to plot, write the name of the topic you want to
   plot in the field "Topic", e.g., if we want to plot the Z position of our
   crazyflie, we would have to write here
   `/<student_id>/CustomControllerService/DebugTopic/vicon_z`. You can see an
   autocompletion of the
   list of all the topics available when you start typing in the field "Topic". <br><br>
5. Start the Student node following the steps mentioned before (`roslaunch d_fall_pps Student.launch`) and enable the Custom Controller.<br><br>
6. Once we are using the Custom Controller, we will be seeing how the data
   selected gets plotted in the rqt plots.

<!-- --- -->


<!-- ## Workflow: -->
<!-- **Setup** -->
<!-- 1.  Teacher must run his part, that publishes ViconData for students and hosts the roscore. -->
<!-- 2.  Each student/group has a CrazyFlie and a laptop. -->
<!-- 3.  Use `roscd d_fall_pps/launch` in a terminal as well as `roscd d_fall_pps/scripts` in another terminal -->

<!-- <br> -->
<!-- **Working** -->
<!-- 1.  Adjust your custom controller -->
<!-- 2.  Use `catkin_make` in the pps_ws directory to compile your controller implementation -->
<!-- 3.  Start your crazyflie -->
<!-- 4.  Launch the correct file in the launch directory as described above. ClientConfig.yaml has to be correct. -->
<!-- 5. Use the scripts to change from the safe to your custom controller. -->
<!-- 6. When your done, you can turn of your crazyflie by using the script `disable_crazyflie`. -->
<!-- 7. Repeat -->


<!-- --- -->
<!-- **Troubleshooting** -->
<!-- - _SafeController is not working_ <br> -->
<!-- Was the antenna of the crazyflie facing in the *opposite* direction of the defined Vicon x-axis? -\-> Define it again! <br> -->
<!-- The crazyflie has to lie on the table when you turn it on because the gyro sensor is initialized upon start-up. <br> -->
<!-- Is the crazyflie still properly showing in the ViconTracker software? -\-> Define it again and check that the markers don't move! -->
<!-- - If you have added a new controller. Don't forget to adjust the CMakeList.txt file and use catkin_make again. -->
