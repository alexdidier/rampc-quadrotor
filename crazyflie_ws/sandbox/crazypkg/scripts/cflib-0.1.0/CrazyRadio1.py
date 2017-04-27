#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import roslib; roslib.load_manifest('crazypkg')

# General import
import time, sys
import struct
#import logging

# Add library
#sys.path.append("lib")

# CrazyFlie client imports
import cflib

from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

import cflib.drivers.crazyradio

# Logging import
#from cflib.crazyflie.log import LogConfig

# Logging settings
#logging.basicConfig(level=logging.ERROR)

class SimpleClient:
    """
       Example script that runs several threads that read Vicon measurements
       from a file and send it together with the setpoints to the Crazyflie.
       It also employs a keyboard event detector that allows the user to
       manipulate the setpoints with keys.
    """
    def __init__(self, link_uri):

        # Setpoints to be sent to the CrazyFlie
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
        print "Connection to %s successful: " % link_uri

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri

    def _send_setpoints(self, frequency = 1):
        if frequency is 0:
            print "Frequency was set to 0. Will be reset to 1 Hz."
            frequency = 1
        try:
            # Send setpoints at the given frequency.
            while 1:
                # Fill the CRTP packet with the setpoints and send it to the stabilizer
                pk = CRTPPacket()
                pk.port = CRTPPort.STABILIZER
                pk.data = struct.pack('<fff', self.targetX, self.targetY, self.targetZ)
                self._cf.send_packet(pk)
                print "Setpoints: %f, %f, %f" % (self.targetX, self.targetY, self.targetZ)

                # maintain the frequency
                time.sleep(1.0 / frequency)
        finally:
            # Close the link, therefore, terminate the application.
            self._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]
    if len(available) > 0:
        # uri would can be specified directly, as for example: radio://0/70/250K
        # instead of available[0][0]
        cf_client = SimpleClient('radio://0/80/250K')
        #cf_client = SimpleClient(available[0][0])
    else:
        print "No Crazyflies found, cannot run example"
    #inp=raw_input('press any key')

