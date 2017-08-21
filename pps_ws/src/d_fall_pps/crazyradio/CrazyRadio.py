#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('d_fall_pps')
import rospy
from d_fall_pps.msg import ControlCommand
from std_msgs.msg import Int32


# General import
import time, sys
import struct
import logging

import rosbag
from rospkg import RosPack
from std_msgs.msg import Float32
from std_msgs.msg import String

# Add library
#sys.path.append("lib")

# CrazyFlie client imports
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

import cflib.drivers.crazyradio

# Logging import(*
from cflib.crazyflie.log import LogConfig

# Logging settings
logging.basicConfig(level=logging.ERROR)

CONTROLLER_MOTOR = 2
CONTROLLER_ANGLE = 1
CONTROLLER_RATE = 0
RAD_TO_DEG = 57.296

# CrazyRadio states:
CONNECTED = 0
CONNECTING = 1
DISCONNECTED = 2

# Commands coming
CMD_RECONNECT = 0

rp = RosPack()
record_file = rp.get_path('d_fall_pps') + '/LoggingOnboard.bag'
rospy.loginfo('afdsasdfasdfsadfasdfasdfasdfasdfasdfasdf')
rospy.loginfo(record_file)
bag = rosbag.Bag(record_file, 'w')

class PPSRadioClient:
    """
       CrazyRadio client that recieves the commands from the controller and
       sends them in a CRTP package to the crazyflie with the specified
       address.
    """
    def __init__(self, link_uri):

        # Setpoints to be sent to the CrazyFlie
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.motor1cmd = 0.0
        self.motor2cmd = 0.0
        self.motor3cmd = 0.0
        self.motor4cmd = 0.0
        self.status = DISCONNECTED

        # Initialize the CrazyFlie and add callbacks
        self._cf = Crazyflie()

        # Add callbacks that get executed depending on the connection status.
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Connect to the Crazyflie
        print "Connecting to %s" % link_uri

        self.connect();

    def connect(self):
        self.status = CONNECTING
        self._cf.open_link(link_uri)
    def _data_received_callback(self, timestamp, data, logconf):
        #print "log of stabilizer and battery: [%d][%s]: %s" % (timestamp, logconf.name, data)
        batteryVolt = Float32()
        stabilizerYaw = Float32()
        stabilizerPitch = Float32()
        stabilizerRoll = Float32()
        batteryVolt.data = data["pm.vbat"]
        stabilizerYaw.data = data["stabilizer.yaw"]
        stabilizerPitch.data = data["stabilizer.pitch"]
        bag.write('batteryVoltage', batteryVolt)
        bag.write('stabilizerYaw', stabilizerYaw)
        bag.write('stabilizerPitch', stabilizerPitch)
        bag.write('stabilizerRoll', stabilizerRoll)

        #publish battery voltage for GUI
        #cfbattery_pub.publish(std_msgs.Float32(batteryVolt.data))
        cfbattery_pub.publish(batteryVolt)



    def _logging_error(self, logconf, msg):
        print "Error when logging %s" % logconf.name

    def _connected(self, link_uri):
        """
            This callback is executed as soon as the connection to the
            quadrotor is established.
        """
        self.status = CONNECTED
        rospy.loginfo("Connection to %s successful: " % link_uri)


        # Config for Logging
        logconf = LogConfig("LoggingTest", 100)
        logconf.add_variable("stabilizer.roll", "float");
        logconf.add_variable("stabilizer.pitch", "float");
        logconf.add_variable("stabilizer.yaw", "float");
        logconf.add_variable("pm.vbat", "float");

        self._cf.log.add_config(logconf)
        if logconf.valid:
            logconf.data_received_cb.add_callback(self._data_received_callback)
            logconf.error_cb.add_callback(self._logging_error)
            logconf.start()
            print "logconf valid"
        else:
            print "logconf invalid"


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        rospy.logerr("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        rospy.logerr("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""

        self.status = DISCONNECTED
        rospy.logwarn("Disconnected from %s" % link_uri)
        bag.close()
        rospy.loginfo("bag closed")


    def _send_to_commander(self,roll, pitch, yaw, thrust, cmd1, cmd2, cmd3, cmd4, mode):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffHHHHHH', roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG, thrust, cmd1, cmd2, cmd3, cmd4, mode)
        self._cf.send_packet(pk)

    def crazyRadioCommandCallback(data):
        """Callback to tell CrazyRadio to reconnect"""
        if data = CMD_RECONNECT:            # reconnect, check status first and then do whatever needs to be done
            if self.status = DISCONNECTED:
                self.connect()

def controlCommandCallback(data):
    """Callback for controller actions"""
    #rospy.loginfo("controller callback : %s, %s, %s", data.roll, data.pitch, data.yaw)

    #cmd1..4 must not be 0, as crazyflie onboard controller resets!
    #pitch and yaw are inverted on crazyflie controller
    cf_client._send_to_commander(data.roll, -data.pitch, -data.yaw, 0, data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.onboardControllerType)





if __name__ == '__main__':
    rospy.init_node('CrazyRadio', anonymous=True)
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    #wait until address parameter is set by PPSClient
    while not rospy.has_param("~crazyFlieAddress"):
        time.sleep(0.05)

    radio_address = "radio://" + rospy.get_param("~crazyFlieAddress")
    rospy.loginfo("Crazyradio connecting to %s" % radio_address)

    #use this following two lines to connect without data from CentralManager
    #radio_address = "radio://0/72/2M"
    #rospy.loginfo("manual address loaded")
    global cfbattery_pub
    cfbattery_pub = rospy.Publisher('cfbattery', Float32, queue_size=10)

    global cf_client

    cf_client = PPSRadioClient(radio_address)
    rospy.Subscriber("PPSClient/crazyRadioCommand", Int32, cf_client.crazyRadioCommandCallback)

    time.sleep(1.0)

    rospy.Subscriber("PPSClient/ControlCommand", ControlCommand, controlCommandCallback)

    rospy.spin()
    rospy.loginfo("Turning off crazyflie")

    cf_client._send_to_commander(0, 0, 0, 0, 0, 0, 0, 0, CONTROLLER_MOTOR)
    #wait for client to send its commands
    time.sleep(1.0)

    cf_client._cf.close_link()
    rospy.loginfo("Link closed")
