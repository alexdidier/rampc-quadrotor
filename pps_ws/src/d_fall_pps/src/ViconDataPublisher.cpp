//    ROS node that publishes the data from the Vicon system
//    Copyright (C) 2017  Dusan Zikovic, Cyrill Burgener, Marco Mueller, Philipp Friedli
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.




//TODO:
//CentralManager: extract data about room from vicon data
//CentralManager: assign area for each group and those coordinates to PPSClients
//ViconDataPublisher: extract data about room from vicon data in and send also to PPSClient
//PPSClient: Compare data received from CentralManager and ViconDataPublisher and determine in which area you are
//PPSClient: Choose correct controller accoring to current area



#include <string.h>
#include "DataStreamClient.h"
#include "ros/ros.h"
#include "d_fall_pps/ViconData.h"

using namespace ViconDataStreamSDK::CPP;
using namespace d_fall_pps;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ViconDataPublisher");

    ros::NodeHandle nodeHandle("~");
    ros::Time::init();

    ros::Publisher viconDataPublisher =
            nodeHandle.advertise<ViconData>("ViconData", 1);

    //publish something random if no viconData is available for testing
    /*
    ViconData viconData; 
    double i = 1; 
    while(true)
    {
    	viconData.roll  = i;
    	 viconDataPublisher.publish(viconData);
    	 ++i;
    }
    //the code will not go further than here if testing without real ViconData
    */

    Client client;

    //connect client to Vicon computer
    std::string hostName = "10.42.00.15:801";
    ROS_INFO_STREAM("Connecting to " << hostName << " ...");
    while (!client.IsConnected().Connected) {
        bool ok = (client.Connect(hostName).Result == Result::Success);

        if (!ok) {
            ROS_ERROR("Error - connection failed...");
            ros::Duration(1.0).sleep();
        } else {
            ROS_INFO("Connected successfully");
        }
    }

    //set data stream parameters
    client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull); //phfriedl: maybe ServerPush instead of ClientPull for less latency?

    client.EnableSegmentData();
    client.EnableMarkerData();
    client.EnableUnlabeledMarkerData();
    client.EnableDeviceData();

    // Set the global up axis, such that Z is up
    client.SetAxisMapping(Direction::Forward,
            Direction::Left,
            Direction::Up);

    //TODO
    //maybe we need a loop rate?---------e.g. 0.5 MHz -----------------------------
    //ros::Rate loop_rate(500000)


    int iterations = 0;
    while (ros::ok()) {
    	//if you want to see at least some output in the terminal
    	//to see that you are still publishing
    	if(iterations % 1000 == 0){
        	ROS_INFO("iteration #%d",iterations);
    	}
    	iterations++;




        // Get a frame
        while (client.GetFrame().Result != Result::Success) {
            // Sleep a little so that we don't lumber the CPU with a busy poll
            ros::Duration(0.001).sleep();
        }

        unsigned int subjectCount = client.GetSubjectCount().SubjectCount;

        // //Process the data and publish on topic
        for (int index = 0; index < subjectCount; index++) {
       		std::string subjectName = client.GetSubjectName(index).SubjectName;
            std::string segmentName = client.GetSegmentName(subjectName, 0).SegmentName; //last value had to be changed to 0 instead of index, 27.03.17


            Output_GetSegmentGlobalTranslation outputTranslation =
                    client.GetSegmentGlobalTranslation(subjectName, segmentName);

            Output_GetSegmentGlobalRotationQuaternion outputRotation =
                    client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName);


            //calculate position and rotation of Crazyflie
            double quat_x = outputRotation.Rotation[0];
            double quat_y = outputRotation.Rotation[1];
            double quat_z = outputRotation.Rotation[2];
            double quat_w = outputRotation.Rotation[3];

            //TODO check whether this transformation is correct
            double roll = atan2(2 * (quat_w * quat_x + quat_y * quat_z), 1 - 2 * (quat_x * quat_x + quat_y * quat_y));
            double pitch = asin(2 * (quat_w * quat_y - quat_z * quat_x));
            double yaw = atan2(2 * (quat_w * quat_z + quat_x * quat_y), 1 - 2 * (quat_y * quat_y + quat_z * quat_z));

            //calculate time until frame data was received
            Output_GetLatencyTotal outputLatencyTotal = client.GetLatencyTotal();
            double totalViconLatency;
            if (outputLatencyTotal.Result == Result::Success) {
                totalViconLatency = outputLatencyTotal.Total;
            } else {
                totalViconLatency = 0;
            }

            //build and publish message
            ViconData viconData;
            viconData.crazyflieName = subjectName;

            viconData.x = outputTranslation.Translation[0];
            viconData.y = outputTranslation.Translation[1];
            viconData.z = outputTranslation.Translation[2];
            viconData.roll = roll;
            viconData.pitch = pitch;
            viconData.yaw = yaw;
            viconData.acquiringTime = totalViconLatency;

            //finally publish
            viconDataPublisher.publish(viconData);

        }

    }

    client.DisableSegmentData();
    client.DisableMarkerData();
    client.DisableUnlabeledMarkerData();
    client.DisableDeviceData();

    client.Disconnect();
}
