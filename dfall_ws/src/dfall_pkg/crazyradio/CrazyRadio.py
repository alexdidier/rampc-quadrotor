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


import roslib; roslib.load_manifest('dfall_pkg')
import rospy
from std_msgs.msg import Int32
from dfall_pkg.msg import ControlCommand
from dfall_pkg.msg import IntWithHeader
from dfall_pkg.srv import IntIntService


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

CF_COMMAND_TYPE_MOTORS = 6
CF_COMMAND_TYPE_RATE =   7
CF_COMMAND_TYPE_ANGLE =  8

RAD_TO_DEG = 57.296

# CrazyRadio states:
CONNECTED = 0
CONNECTING = 1
DISCONNECTED = 2

# Commands coming
CMD_RECONNECT = 0
CMD_DISCONNECT = 1

# Commands for FlyingAgentClient
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
record_file = rp.get_path('dfall_pkg') + '/LoggingOnboard.bag'
rospy.loginfo('afdsasdfasdfsadfasdfasdfasdfasdfasdfasdf')
rospy.loginfo(record_file)
bag = rosbag.Bag(record_file, 'w')

class CrazyRadioClient:
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
        self.FlyingAgentClient_command_pub = rospy.Publisher('FlyingAgentClient/Command', IntWithHeader, queue_size=1)
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
        print "[CRAZY RADIO] status changed to: %s" % new_status
        self._status = new_status
        self.status_pub.publish(new_status)

    def get_status(self):
        return self._status

    def update_link_uri(self):
        self.link_uri = "radio://" + rospy.get_param("~crazyFlieAddress")

    def connect(self):
        # update link from ros params
        self.update_link_uri()

        print "[CRAZY RADIO] Connecting to %s" % self.link_uri
        self.change_status_to(CONNECTING)
        rospy.loginfo("[CRAZY RADIO] connecting...")
        self._cf.open_link(self.link_uri)

    def disconnect(self):
        print "[CRAZY RADIO] sending Motors OFF command before disconnecting"
        self._send_to_commander_motor(0, 0, 0, 0)
        # change state to motors OFF
        msg = IntWithHeader()
        msg.shouldCheckForAgentID = False
        msg.data = CMD_CRAZYFLY_MOTORS_OFF
        self.FlyingAgentClient_command_pub.publish(msg)
        time.sleep(0.1)
        print "[CRAZY RADIO] Disconnecting from %s" % self.link_uri
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
        print "[CRAZY RADIO] Error when logging %s" % logconf.name

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
            print "[CRAZY RADIO] logconf valid"
        else:
            print "[CRAZY RADIO] logconf invalid"

        self.logconf.start()
        print "[CRAZY RADIO] logconf start"

    def _connected(self, link_uri):
        """
            This callback is executed as soon as the connection to the
            quadrotor is established.
        """
        self.change_status_to(CONNECTED)
        # change state to motors OFF
        msg = IntWithHeader()
        msg.shouldCheckForAgentID = False
        msg.data = CMD_CRAZYFLY_MOTORS_OFF
        cf_client.FlyingAgentClient_command_pub.publish(msg)

        rospy.loginfo("[CRAZY RADIO] Connection to %s successful: " % link_uri)
        # Config for Logging
        self._start_logging()




    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        self.change_status_to(DISCONNECTED)
        rospy.logerr("[CRAZY RADIO] Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        self.change_status_to(DISCONNECTED)
        rospy.logerr("[CRAZY RADIO] Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.change_status_to(DISCONNECTED)
        rospy.logwarn("[CRAZY RADIO] Disconnected from %s" % link_uri)

        # change state to motors OFF
        msg = IntWithHeader()
        msg.shouldCheckForAgentID = False
        msg.data = CMD_CRAZYFLY_MOTORS_OFF
        self.FlyingAgentClient_command_pub.publish(msg)

        self.logconf.stop()
        rospy.loginfo("logconf stopped")
        self.logconf.delete()
        rospy.loginfo("logconf deleted")

    def _send_to_commander_motor(self, cmd1, cmd2, cmd3, cmd4):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHH', CF_COMMAND_TYPE_MOTORS, cmd1, cmd2, cmd3, cmd4)
        self._cf.send_packet(pk)

    def _send_to_commander_rate(self, cmd1, cmd2, cmd3, cmd4, roll_rate, pitch_rate, yaw_rate):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHfff', CF_COMMAND_TYPE_RATE, cmd1, cmd2, cmd3, cmd4, roll_rate * RAD_TO_DEG, pitch_rate * RAD_TO_DEG, yaw_rate * RAD_TO_DEG)
        self._cf.send_packet(pk)

    def _send_to_commander_angle(self, cmd1, cmd2, cmd3, cmd4, roll, pitch, yaw):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.data = struct.pack('<BHHHHfff', CF_COMMAND_TYPE_ANGLE, cmd1, cmd2, cmd3, cmd4, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG)
        self._cf.send_packet(pk)

    # def _send_to_commander(self,roll, pitch, yaw, thrust, cmd1, cmd2, cmd3, cmd4, mode):
    #     pk = CRTPPacket()
    #     pk.port = CRTPPort.COMMANDER
    #     pk.data = struct.pack('<fffHHHHHH', roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG, thrust, cmd1, cmd2, cmd3, cmd4, mode)
    #     self._cf.send_packet(pk)

    def crazyRadioCommandCallback(self, msg):
        """Callback to tell CrazyRadio to reconnect"""

        # Initialise a boolean flag that the command is NOT relevant
        command_is_relevant = False

        # Check the header details of the message for it relevance
        if (msg.shouldCheckForAgentID == False):
            command_is_relevant = True
        else:
            for this_ID in msg.agentIDs:
                if (this_ID == m_agentID):
                    command_is_relevant = True
                    break

        # Only consider the command if it is relevant
        if (command_is_relevant):
            #print "[CRAZY RADIO] received command to: %s" % msg.data
            if msg.data == CMD_RECONNECT:
                if self.get_status() == DISCONNECTED:
                    print "[CRAZY RADIO] received command to CONNECT (current status is DISCONNECTED)"
                    self.connect()
                elif self.get_status() == CONNECTING:
                    print "[CRAZY RADIO] received command to CONNECT (current status is CONNECTING)"
                    #self.status_pub.publish(CONNECTING)
                elif self.get_status() == CONNECTED:
                    print "[CRAZY RADIO] received command to CONNECT (current status is CONNECTED)"
                    #self.status_pub.publish(CONNECTED)

            elif msg.data == CMD_DISCONNECT:
                if self.get_status() == CONNECTED:
                    print "[CRAZY RADIO] received command to DISCONNECT (current status is CONNECTED)"
                    self.disconnect()
                elif self.get_status() == CONNECTING:
                    print "[CRAZY RADIO] received command to DISCONNECT (current status is CONNECTING)"
                    #self.status_pub.publish(CONNECTING)
                elif self.get_status() == DISCONNECTED:
                    print "[CRAZY RADIO] received command to DISCONNECT (current status is DISCONNECTED)"
                    #self.status_pub.publish(DISCONNECTED)



    def getCurrentCrazyRadioStatusServiceCallback(self, req):
        """Callback to service the request for the connection status"""
        # Directly return the current status
        return self._status




def controlCommandCallback(data):
    """Callback for controller actions"""
    #rospy.loginfo("controller callback : %s, %s, %s", data.roll, data.pitch, data.yaw)

    #cmd1..4 must not be 0, as crazyflie onboard controller resets!
    #pitch and yaw are inverted on crazyflie controller
    if data.onboardControllerType == CF_COMMAND_TYPE_MOTORS:
        cf_client._send_to_commander_motor(data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4)

    elif data.onboardControllerType == CF_COMMAND_TYPE_RATE:
        cf_client._send_to_commander_rate(data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.roll, -data.pitch, -data.yaw)

    elif data.onboardControllerType == CF_COMMAND_TYPE_ANGLE:
        cf_client._send_to_commander_angle(data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.roll, -data.pitch, -data.yaw)
        # cf_client._send_to_commander(data.roll, -data.pitch, -data.yaw, 0, data.motorCmd1, data.motorCmd2, data.motorCmd3, data.motorCmd4, data.onboardControllerType)






if __name__ == '__main__':

    # Starting the ROS-node
    global node_name
    node_name = "CrazyRadio"
    rospy.init_node(node_name, anonymous=True)

    # Get the namespace of this node
    global ros_namespace
    ros_namespace = rospy.get_namespace()


    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    #wait until address parameter is set by FlyingAgentClient
    while not rospy.has_param("~crazyFlieAddress"):
        time.sleep(0.05)

    #use this following two lines to connect without data from CentralManager
    # radio_address = "radio://0/72/2M"
    # rospy.loginfo("manual address loaded")

    # Fetch the YAML paramter "battery polling period"
    global battery_polling_period
    battery_polling_period = rospy.get_param(ros_namespace + "/CrazyRadio/battery_polling_period")

    # Fetch the YAML paramter "agentID" and "coordID"
    global m_agentID
    m_agentID = rospy.get_param(ros_namespace + "/FlyingAgentClient/agentID")
    coordID   = rospy.get_param(ros_namespace + "/FlyingAgentClient/coordID")
    # Convert the coordinator ID to a zero-padded string
    coordID_as_string = format(coordID, '03')


    # Initialise a publisher for the battery voltage
    global cfbattery_pub
    cfbattery_pub = rospy.Publisher(node_name + '/CFBattery', Float32, queue_size=10)

    # Initialise a "CrazyRadioClient" type variable that handles
    # all communication over the CrazyRadio
    global cf_client
    cf_client = CrazyRadioClient()

    print "[CRAZY RADIO] DEBUG 2"

    # Subscribe to the commands for when to (dis-)connect the
    # CrazyRadio connection with the Crazyflie
    # > For the radio commands from the FlyingAgentClient of this agent
    rospy.Subscriber("FlyingAgentClient/crazyRadioCommand", IntWithHeader, cf_client.crazyRadioCommandCallback)
    # > For the radio command from the coordinator
    rospy.Subscriber("/dfall/coord" + coordID_as_string + "/FlyingAgentClient/crazyRadioCommand", IntWithHeader, cf_client.crazyRadioCommandCallback)


    # Advertise a Serice for the current status
    # of the Crazy Radio connect
    s = rospy.Service(node_name + "/getCurrentCrazyRadioStatus", IntIntService , cf_client.getCurrentCrazyRadioStatusServiceCallback)



    time.sleep(1.0)

    rospy.Subscriber("FlyingAgentClient/ControlCommand", ControlCommand, controlCommandCallback)

    print "[CRAZY RADIO] Node READY :-)"
    rospy.spin()
    rospy.loginfo("[CRAZY RADIO] Turning off crazyflie")

    cf_client._send_to_commander_motor(0, 0, 0, 0)
    # change state to motors OFF
    msg = IntWithHeader()
    msg.shouldCheckForAgentID = False
    msg.data = CMD_CRAZYFLY_MOTORS_OFF
    cf_client.FlyingAgentClient_command_pub.publish(msg)
    #wait for client to send its commands
    time.sleep(1.0)


    bag.close()
    rospy.loginfo("[CRAZY RADIO] bag closed")

    cf_client._cf.close_link()
    rospy.loginfo("[CRAZY RADIO] Link closed")
