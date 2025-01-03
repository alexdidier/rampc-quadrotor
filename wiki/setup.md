# Setup
Contents:
- [Vicon](#vicon)
- [Firmware and channel](#firmware-and-channel)
- [Network](#network)

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


## Firmware and channel
The firmware of a crazyflie is updated by the teacher. Additionally, the teacher can set the channel of the crazyflies to prevent interference.<br>
### Firmware
Instructions for flashing the firmware with the Crazyflie client can be found either here:
<br>
https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index#firmware_upgrade
<br>
or here:
<br>
https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#update-fw
<br>
The compiled versions of the Crazyflie firmware that are compatible with the `D-FaLL-System` can be found in the `crazyflie-firmware` folder of the repository.
<br>
<br>
If you have installed the Crazyflie Client properly, as described in the installation section, it can be started via a terminal window by typing `cfclient`.
<br>
<br>
If flashing the firmware on the NRF bluetooth chip, then you must connect the Crazyflie to the computer using a USB cable. If you only need to flash the STM32 main processor chip, this can be done wirelessly and you need to specify the correct address in the Crazyflie Client prior to following the steps below, i.e., the 0xE7E7E7E701 type address.
<br>
<br>
The steps to flash the crazyflie are:
1. Start the Crazyflie Client from terminal using the command `cfclient`<br>
2. Turn the Crazyflie off<br>
3. Start the Crazyflie in bootloader mode by pressing the ON/OFF button for 3 second. Two blue LEDs will start blinking to indicate the the Crazyflie has powered on into bootloader mode<br>
4. In the Crazyflie Client, select `Connect -> Bootloader` from the top menu. This causes a window titled `Crazyflie Service` to appear<br>
5. In the `Crazyflie Service` window, press the `Initiate bootloader cold boot` button<br>
6. Once the status says `Connected to bootloader`, click the `Browse` button and select the file you wish to flash on the Crazyflie. Typically this file will be something like `cf2.bin` for flashing only the STM32 main processor, or `crazyflie-firmware.zip` for flashing both the NRF and STM32 processors<br>
7. Click the `Program` button. The progress bar will go from 0% to 100% one time for each of the processors to be flashed<br>
8. Wait until the uploading and writing of the new firmware is complete<br>
9. Click the `Restart in firmware mode` button. This causes the Crazyflie to reboot and the new firmware is now running<br>
10. Turn off the Crazyflie<br>
11. Either click the `Cancel bootloading` button or simply close the `Crazyflie Service` window<br><br>

### Channel changing
This is also described on the page linked above. Use the following format: 0/__xx__/2M where __xx__ stands for the radio channel.<br>
The crazyflie has to be restared for the changes to take effect.<br>
![channel_config](https://wiki.bitcraze.io/_media/doc:crazyflie:client:pycfclient:cfclient_cf2_config.png?w=500&tok=74d1d3)


## Network
### Setting up the Vicon network
Inseret the Ethernet cable from Vicon into your computer and go to network settings in the top right corner. Click on _Edit Connections..._ <br>
Choose the Ethernet connection (probably named _wired connection 1_) and click on _edit_.
<br><img src="./pics/setup_pics/viconNetwork1.png" style="width: 500px;"/> <br>
Change the connection name to _Vicon_.
<br><img src="./pics/setup_pics/viconNetwork2.png" style="width: 500px;"/> <br>
Then go to the _IPv4 Settings_ and choose **Manual** as the _Method_ and then add your IP address provided by your teacher (Form 10.42.0.xx). Click tab twice and _save_.
<br><img src="./pics/setup_pics/viconNetwork3.png" style="width: 500px;"/> <br>
**Don't forget to choose the Vicon network manually if you choose to enable WiFi and Ethernet simultaneously as described below!**

<br>

### Vicon, teacher and students
During installation process is the IP address of the teacher set to 10.42.0.10. (This value is written to the /etc/hosts file such that this IP address is accessible through the keyword _teacher_) <br>
Have a look at `Config.sh` in `~/dfall_ws/src/dfall_pkg/launch/`
<br><img src="./pics/setup_pics/configsh2.png" style="width: 500px;"/> <br>
Here you see, that the ROS Master URI is set to be the teacher. This means that _roscore_ runs only on the teacher's computer. Your own IP address (_ROS IP_) is also set and taken from your Ethernet settings as defined in the section _Setting up the Vicon network_.
<br>

### IP-Addresses
Currently the teacher's IP is ``10.42.0.10`` and the student's IP are of the format ``10.42.0.xx``, where xx is an unused address.

### Using WLAN and Vicon simultaneously
Without some adjustments it is not possible to use an Ethernet and a wireless network at the same time. Vicon is connected via cable and therefore wouldn't allow a connection to the Internet. Because it's tedious to always unplug the cable just to be able to look something up on Google, we provide an explanation on how to enable simultaneous usage of WLAN and Vicon. <br>
**But careful: If you enable this setting you have to choose the Vicon network manually. This is a common mistake! You will get an error if the cable is inserted but not manually chosen as shown in the picture below.**
<br><img src="./pics/setup_pics/chooseVicon.jpg" style="width: 500px;"/> <br>
#### Step by Step:
Ubuntu allows multiple connections by default, but sometimes, we need to specify which one to use. Here we use LAN for the Intranet and WiFi for the Internet.

So, firstly search for Network Connections in the unity dash. Then, under the Ethernet section, click 'Add' button.
<br><img src="./pics/setup_pics/internetethernet1.png" style="width: 500px;"/> <br>

Then, we need to create a new Ethernet connection which we are going to enable manually. Uncheck the option of connecting automatically since this is your Intranet connection. **Choose _Vicon_ as a name instead of Intranet**
<br><img src="./pics/setup_pics/internetethernet2.png" style="width: 500px;"/> <br>

Then go to the IPv4/IPv6 settings (depending on your network) and then click on the Routes button. Check the 'Use this connection only for resources on its network' option. Click Save.
<br><img src="./pics/setup_pics/internetethernet3.png" style="width: 500px;"/> <br>
Now you can use both the LAN and WiFi simultaneously.

<br>
Source: <br> https://askubuntu.com/questions/639100/how-to-get-connection-to-both-wifi-as-well-as-lan-in-ubuntu-14-04-lts#639425
<br>
Some additional informations: <br> http://aleksz-programming.blogspot.ch/2013/01/using-wifi-and-network-cable-at-same.html
