#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('d_fall_pps')
import rospy
from d_fall_pps.msg import ControlParameters


# General import
import time, sys
import struct
import logging

# Add library
#sys.path.append("lib")

# CrazyFlie client imports
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

import cflib.drivers.crazyradio

# Logging import
from cflib.crazyflie.log import LogConfig

# Logging settings
logging.basicConfig(level=logging.ERROR)

class SimpleClient:
    """
       Example script that runs several threads that read Vicon measurements
       from a file and send it together with the setpoints to the Crazyflie.
       It also employs a keyboard event detector that allows the user to
       manipulate the setpoints with keys.
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

        # Initialize the CrazyFlie and add callbacks
        self._cf = Crazyflie()

        # Add callbacks that get executed depending on the connection status.
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Connect to the Crazyflie
        print "Connecting to %s" % link_uri
        self._cf.open_link(link_uri)


    def _connected(self, link_uri):
        """
            This callback is executed as soon as the connection to the
            quadrotor is established.
        """
        #print "Connection to %s successful: " % link_uri
        rospy.loginfo("Connection to %s successful: " % link_uri)
        cf_client._send_to_commander(0, 0, 0, 1000,    0, 0, 0, 0, 0)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        #print "Connection to %s failed: %s" % (link_uri, msg)
        rospy.logerr("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        #print "Connection to %s lost: %s" % (link_uri, msg)
        rospy.logerr("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        #print "Disconnected from %s" % link_uri
        rospy.logwarn("Disconnected from %s" % link_uri)


    def _send_commands(self,cmd1,cmd2,cmd3,cmd4):
        # Send setpoints at the given frequency.
        # Fill the CRTP packet with the setpoints and send it to the stabilizer
        pk = CRTPPacket()
        pk.port = CRTPPort.STABILIZER
        pk.data = struct.pack('<ffff', cmd1,cmd2,cmd3,cmd4)
        self._cf.send_packet(pk)
        print('command')
        #print "Motor commands: %f, %f, %f, %f" % (cmd1,cmd2,cmd3,cmd4)

    def _send_to_commander(self,roll, pitch, yaw, thrust,cmd1,cmd2,cmd3,cmd4,mode):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        #pk.data = struct.pack('<fffHHHHHH', roll, pitch, yaw, thrust,cmd1,cmd2,cmd3,cmd4,mode)
        pk.data = struct.pack('<fffH', roll, pitch, yaw, thrust)
        self._cf.send_packet(pk)
        print(thrust)

        # self._cf.commander.send_setpoint (roll, pitch, yaw, thrust,cmd1,cmd2,cmd3,cmd4,mode)
        # print "Motor commands: %f, %f, %f, %f" % (cmd1,cmd2,cmd3,cmd4)

def subscriberControllerOutputCallback(data):
    #cf_client._send_commands(data.cmd1,data.cmd2,data.cmd3,data.cmd4)
    cf_client._send_to_commander(data.roll,data.pitch,data.yaw,data.thrust,data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.onboardControllerType)
    #rospy.loginfo(data.onboardControllerType);
    #rospy.loginfo(data.motorCmd1);
    #rospy.logdebug_throttle(2,"sending motor commands to crazyflie: ")




if __name__ == '__main__':
    rospy.init_node('CrazyRadio', anonymous=True)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    while not rospy.is_shutdown():

        # Initialize the low-level drivers (don't list the debug drivers)

        # Scan for Crazyflies and use the first one found
        rospy.loginfo("Scanning interfaces for Crazyflies...")
        available=[]
        available = cflib.crtp.scan_interfaces()
        rospy.loginfo("Crazyflies found:")
        for i in available:
            print i[0]
        if len(available) > 0:
            # uri would can be specified directly, as for example: radio://0/70/250K
            # instead of available[0][0]
            global cf_client
            #cf_client = SimpleClient('radio://0/111/250K')
            cf_client = SimpleClient(available[0][0])
            time.sleep(5.0)
            #rospy.Subscriber("FlightControl/topicControllerOutput", ControllerOutputPackage, subscriberControllerOutputCallback)

            while True:
                cf_client._send_to_commander(1, 1, 1, 1000,    100, 100, 100, 100, 100)

            rospy.spin()
        else:
            rospy.logerr("No Crazyflies found, cannot run example")

        #inp=raw_input('press any key');


        time.sleep(0.5)

    cf_client._cf.close_link()


