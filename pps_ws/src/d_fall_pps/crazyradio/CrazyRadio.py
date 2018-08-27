#!/usr/bin/env python
# -*- coding: utf-8 -*-

#    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
#
#    This file is part of D-FaLL-System.
#    
#    D-FaLL-System is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#    
#    D-FaLL-System is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#    
#    You should have received a copy of the GNU General Public License
#    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
#    
#
#    ----------------------------------------------------------------------------------
#    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
#    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
#    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
#    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
#    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
#
#
#    DESCRIPTION:
#    The service that manages the context of the student groups.
#
#    ----------------------------------------------------------------------------------


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

# CONTROLLER_MOTOR = 2
# CONTROLLER_ANGLE = 1
# CONTROLLER_RATE = 0


TYPE_PPS_MOTORS = 6
TYPE_PPS_RATE =   7
TYPE_PPS_ANGLE =  8

RAD_TO_DEG = 57.296

# CrazyRadio states:
CONNECTED = 0
CONNECTING = 1
DISCONNECTED = 2

# Commands coming
CMD_RECONNECT = 0
CMD_DISCONNECT = 1

# Commands for PPSClient
#CMD_USE_SAFE_CONTROLLER =       1
#CMD_USE_DEMO_CONTROLLER =       2
#CMD_USE_STUDENT_CONTROLLER =    3
#CMD_USE_MPC_CONTROLLER =        4
#CMD_USE_REMOTE_CONTROLLER =     5
#CMD_USE_TUNING_CONTROLLER =     6

CMD_CRAZYFLY_TAKE_OFF =     11
CMD_CRAZYFLY_LAND =         12
CMD_CRAZYFLY_MOTORS_OFF =   13

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
    def __init__(self):

        # Setpoints to be sent to the CrazyFlie
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.motor1cmd = 0.0
        self.motor2cmd = 0.0
        self.motor3cmd = 0.0
        self.motor4cmd = 0.0
        self._status = DISCONNECTED
        self.link_uri = ""

        self.status_pub = rospy.Publisher(node_name + '/CrazyRadioStatus', Int32, queue_size=1)
        self.PPSClient_command_pub = rospy.Publisher('PPSClient/Command', Int32, queue_size=1)
        time.sleep(1.0)

        # Initialize the CrazyFlie and add callbacks
        self._init_cf()

        # Connect to the Crazyflie
        self.connect()

    def _init_cf(self):
        self._cf = Crazyflie()

        # Add callbacks that get executed depending on the connection _status.
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.change_status_to(DISCONNECTED)

    def change_status_to(self, new_status):
        print "status changed to: %s" % new_status
        self._status = new_status
        self.status_pub.publish(new_status)

    def get_status(self):
        return self._status

    def update_link_uri(self):
        self.link_uri = "radio://" + rospy.get_param("~crazyFlieAddress")

    def connect(self):
        # update link from ros params
        self.update_link_uri()

        print "Connecting to %s" % self.link_uri
        self.change_status_to(CONNECTING)
        rospy.loginfo("connecting...")
        self._cf.open_link(self.link_uri)

    def disconnect(self):
        print "Motors OFF"
        self._send_to_commander_motor(0, 0, 0, 0)
        # change state to motors OFF
        self.PPSClient_command_pub.publish(CMD_CRAZYFLY_MOTORS_OFF)
        time.sleep(0.1)
        print "Disconnecting from %s" % self.link_uri
        self._cf.close_link()
        self.change_status_to(DISCONNECTED)

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
        # print "batteryVolt: %s" % batteryVolt
        cfbattery_pub.publish(batteryVolt)



    def _logging_error(self, logconf, msg):
        print "Error when logging %s" % logconf.name

    # def _init_logging(self):

    def _start_logging(self):
        self.logconf = LogConfig("LoggingTest", battery_polling_period) # second variable is period in ms
        self.logconf.add_variable("stabilizer.roll", "float");
        self.logconf.add_variable("stabilizer.pitch", "float");
        self.logconf.add_variable("stabilizer.yaw", "float");
        self.logconf.add_variable("pm.vbat", "float");

        self._cf.log.add_config(self.logconf)
        if self.logconf.valid:
            self.logconf.data_received_cb.add_callback(self._data_received_callback)
            self.logconf.error_cb.add_callback(self._logging_error)
            print "logconf valid"
        else:
            print "logconf invalid"

        self.logconf.start()
        print "logconf start"

    def _connected(self, link_uri):
        """
            This callback is executed as soon as the connection to the
            quadrotor is established.
        """
        self.change_status_to(CONNECTED)
        # change state to motors OFF
        cf_client.PPSClient_command_pub.publish(CMD_CRAZYFLY_MOTORS_OFF)

        rospy.loginfo("Connection to %s successful: " % link_uri)
        # Config for Logging
        self._start_logging()




    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        self.change_status_to(DISCONNECTED)
        rospy.logerr("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        self.change_status_to(DISCONNECTED)
        rospy.logerr("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.change_status_to(DISCONNECTED)
        rospy.logwarn("Disconnected from %s" % link_uri)

        # change state to motors OFF
        self.PPSClient_command_pub.publish(CMD_CRAZYFLY_MOTORS_OFF)

        self.logconf.stop()
        rospy.loginfo("logconf stopped")
        self.logconf.delete()
        rospy.loginfo("logconf deleted")

    def _send_to_commander_motor(self, cmd1, cmd2, cmd3, cmd4):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHH', TYPE_PPS_MOTORS, cmd1, cmd2, cmd3, cmd4)
        self._cf.send_packet(pk)

    def _send_to_commander_rate(self, cmd1, cmd2, cmd3, cmd4, roll_rate, pitch_rate, yaw_rate):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHfff', TYPE_PPS_RATE, cmd1, cmd2, cmd3, cmd4, roll_rate * RAD_TO_DEG, pitch_rate * RAD_TO_DEG, yaw_rate * RAD_TO_DEG)
        self._cf.send_packet(pk)

    def _send_to_commander_angle(self, cmd1, cmd2, cmd3, cmd4, roll, pitch, yaw):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHfff', TYPE_PPS_ANGLE, cmd1, cmd2, cmd3, cmd4, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG)
        self._cf.send_packet(pk)

    # def _send_to_commander(self,roll, pitch, yaw, thrust, cmd1, cmd2, cmd3, cmd4, mode):
    #     pk = CRTPPacket()
    #     pk.port = CRTPPort.COMMANDER
    #     pk.data = struct.pack('<fffHHHHHH', roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG, thrust, cmd1, cmd2, cmd3, cmd4, mode)
    #     self._cf.send_packet(pk)

    def crazyRadioCommandCallback(self, msg):
        """Callback to tell CrazyRadio to reconnect"""
        print "crazyRadio command received %s" % msg.data

        if msg.data == CMD_RECONNECT:            # reconnect, check _status first and then do whatever needs to be done
            print "entered reconnect"
            print "_status: %s" % self._status
            if self.get_status() == DISCONNECTED:
                print "entered disconnected"
                self.connect()
            if self.get_status() == CONNECTED:
                self.status_pub.publish(CONNECTED)

        elif msg.data == CMD_DISCONNECT:
            print "disconnect received"
            if self.get_status() != DISCONNECTED: # what happens if we disconnect while we are in connecting state?
                self.disconnect()
            else:
                self.status_pub.publish(DISCONNECTED)

def controlCommandCallback(data):
    """Callback for controller actions"""
    #rospy.loginfo("controller callback : %s, %s, %s", data.roll, data.pitch, data.yaw)

    #cmd1..4 must not be 0, as crazyflie onboard controller resets!
    #pitch and yaw are inverted on crazyflie controller
    if data.onboardControllerType == TYPE_PPS_MOTORS:
        cf_client._send_to_commander_motor(data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4)

    elif data.onboardControllerType == TYPE_PPS_RATE:
        cf_client._send_to_commander_rate(data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.roll, -data.pitch, -data.yaw)

    elif data.onboardControllerType == TYPE_PPS_ANGLE:
        cf_client._send_to_commander_angle(data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.roll, -data.pitch, -data.yaw)
        # cf_client._send_to_commander(data.roll, -data.pitch, -data.yaw, 0, data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.onboardControllerType)




if __name__ == '__main__':
    global node_name
    node_name = "CrazyRadio"
    rospy.init_node(node_name, anonymous=True)

    global ros_namespace
    ros_namespace = rospy.get_namespace()
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    #wait until address parameter is set by PPSClient
    while not rospy.has_param("~crazyFlieAddress"):
        time.sleep(0.05)

    #use this following two lines to connect without data from CentralManager
    # radio_address = "radio://0/72/2M"
    # rospy.loginfo("manual address loaded")
    global battery_polling_period
    battery_polling_period = rospy.get_param(ros_namespace + "/CrazyRadio/battery_polling_period")

    global cfbattery_pub
    cfbattery_pub = rospy.Publisher(node_name + '/CFBattery', Float32, queue_size=10)

    global cf_client

    cf_client = PPSRadioClient()
    rospy.Subscriber("PPSClient/crazyRadioCommand", Int32, cf_client.crazyRadioCommandCallback) # allows commands from scripts

    time.sleep(1.0)

    rospy.Subscriber("PPSClient/ControlCommand", ControlCommand, controlCommandCallback)

    rospy.spin()
    rospy.loginfo("Turning off crazyflie")

    cf_client._send_to_commander_motor(0, 0, 0, 0)
    # change state to motors OFF
    cf_client.PPSClient_command_pub.publish(CMD_CRAZYFLY_MOTORS_OFF)
    #wait for client to send its commands
    time.sleep(1.0)


    bag.close()
    rospy.loginfo("bag closed")

    cf_client._cf.close_link()
    rospy.loginfo("Link closed")
