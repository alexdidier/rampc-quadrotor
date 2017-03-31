#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('d_fall_pps')
import rospy
from d_fall_pps.msg import AngleCommand
from d_fall_pps.msg import RateCommand
from d_fall_pps.msg import MotorCommand


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

CONTROLLER_MOTOR = 2
CONTROLLER_ANGLE = 1
CONTROLLER_RATE = 0

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
        rospy.loginfo("Connection to %s successful: " % link_uri)

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
        rospy.logwarn("Disconnected from %s" % link_uri)

    def _send_to_commander(self,roll, pitch, yaw, thrust,cmd1,cmd2,cmd3,cmd4,mode):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffHHHHHH', roll, pitch, yaw, thrust,cmd1,cmd2,cmd3,cmd4,mode)
        self._cf.send_packet(pk)

def motorCommandCallback(data):
    """Callback for motor controller actions"""
    rospy.loginfo("test motorCommandCallback")

def angleCommandCallback(data):
    """Callback for angle controller actions"""
    ###cf_client._send_to_commander(data.roll,data.pitch,data.yaw,data.thrust, 0, 0, 0, 0, CONTROLLER_ANGLE)
    ####rospy.loginfo("angle controller callback: %s, %s, %s", data.roll,data.pitch,data.yaw,data.thrust)
    rospy.loginfo("test angleCommandCallback")

def rateCommandCallback(data):
    """Callback for rate controller actions"""
    ###cf_client._send_to_commander(data.roll,data.pitch,data.yaw,data.thrust, 0, 0, 0, 0, CONTROLLER_RATE)
    ###rospy.loginfo("rate controller callback: %s, %s, %s", data.roll,data.pitch,data.yaw,data.thrust)
    rospy.loginfo("test rateCommandCallback")

def testCallback(data):
    """Callback used to test data receipt if no crazyfly was found"""
    rospy.loginfo("Crazyradio.py successfully subscribed and received testvalues: %f", data.cmd1)

if __name__ == '__main__':
    rospy.init_node('CrazyRadio', anonymous=True)
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)


    while not rospy.is_shutdown():

        # Scan for Crazyflies and use the first one found
        rospy.loginfo("Scanning interfaces for Crazyflies...")
        available=[]
        available = cflib.crtp.scan_interfaces()
        rospy.loginfo("Crazyflies found:")
        for i in available:
            print i[0]
        if len(available) > 0:
            global cf_client

            #TODO: load address from parameters
            cf_client = PPSRadioClient(available[0][0])
            time.sleep(3.0)
            #TODO: change publisher name if not correct
            rospy.Subscriber("PPSClient/topicMotorCommand", MotorCommand, motorCommandCallback)
            rospy.loginfo("trying to subscribe")
            #rospy.Subscriber("PPSClient/AngleCommands", AngleCommand, angleCommandCallback)
            #rospy.Subscriber("PPSClient/RateCommands", RateCommand, rateCommandCallback)

            rospy.spin()
        else:
            #rospy.logerr("No Crazyflies found, cannot run example")
            #for testing try to subscribe even if no crazyflie was found
            rospy.loginfo("No Crazyflies found, still trying to subscribe")
            rospy.Subscriber("PPSClient/topicMotorCommand", MotorCommand, testCallback)
            rospy.spin()



        time.sleep(0.5)

    cf_client._cf.close_link()


