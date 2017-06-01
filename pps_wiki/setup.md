# Setup

## Vicon
### What is Vicon
Vicon is the tracking system for this project. The four cameras in each corner of the room can track the gray markers. One can define objects in the ViconTracker software. Those are then visible in the software or their location data can be shared over a network with others.

### Our Vicon Setup
<img src="./pics/setup_pics/differentlayouts.jpg" style="width: 250px;float: right;"/>
For the PPS we have a separate computer that runs the ViconTracker software. There we define the crazyflie objects that need three markers in slightly different layouts to be distinguishable from another. (see right side)

ViconTracker sends ViconData over the network to the teacher's computer. The teacher computer distributes the data to the students.

### Instructions
#### Calibrating Vicon
1. For calibrating Vicon go to *CALIBRATE* where you should choose *Camera-PixelView* as the *view type*.
<img src="./pics/setup_pics/calibration1.png" style="width: 500px"/> <br><br>

2. Click on *START* in the *Calibrate Cameras* section. Now you have to walk around the room with the L-wand and swing it in every direction. You can stop when the program is finished.
<img src="./pics/setup_pics/calibration2.png" style="width: 500px"/> <br><br>

3. Then you have to lay the wand down somewhere in the room, where you want the origin of the coordinate system to be. How the wand defines the x- and y-axis is shown below. (The handle defines the y-axis, where positive is in direction of orange part. The x-axis is positive in the direction of the longer arm)<br>
Click on *SET ORIGIN* and you're set.<br>
<img src="./pics/setup_pics/calibration3.png" style="width: 300px;"/> <img src="./pics/setup_pics/wand.jpg" style="width: 220px;"/> <br><br>


#### Defining a CrazyFlie in ViconTracker
1. Go to *OBJECTS* and choose *Camera-3D* as the *view type* <br> There you should see some markers.
<br><img src="./pics/setup_pics/defining1.png" style="width: 500px;"/> <br><br>

2. Make sure that the crazyflie antenna is facing in the positive x-axis when you define the crazyflie, because the controller relies on this configuration.
<br> <img src="./pics/setup_pics/cforientation.jpg" style="width: 350px;"/> <br><br>

3. Define an object by `ctrl-rightclick` on the markers and name it in the down left corner. Finish by clicking *CREATE* and saving with `ctrl-s`. Vicon doesn't share the object's data if you don't save. <br>
<br><img src="./pics/setup_pics/defining2.png" style="width: 500px;"/>  <br><br>

4. That's how it looks afterwards <br>
<br><img src="./pics/setup_pics/defining3.png" style="width: 500px;"/> <br><br>

## Network
