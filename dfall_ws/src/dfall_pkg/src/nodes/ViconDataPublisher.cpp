//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero, Dusan Zikovic, Cyrill Burgener, Marco Mueller, Philipp Friedli
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    ROS node that publishes the data from the Vicon system
//
//    ----------------------------------------------------------------------------------


#include <string.h>
#include "DataStreamClient.h"
#include "ros/ros.h"
#include "dfall_pkg/ViconData.h"
#include "dfall_pkg/UnlabeledMarker.h"

// #define TESTING_FAKE_DATA

// notice that unit here are in milimeters
using namespace ViconDataStreamSDK::CPP;
using namespace dfall_pkg;

int main(int argc, char* argv[]) {



    // Starting the ROS-node
    ros::init(argc, argv, "ViconDataPublisher");



    // Create a "ros::NodeHandle" type local variable "nodeHandle"
    // as the current node, the "~" indcates that "self" is the
    // node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");



    // Initialise ROS time
    ros::Time::init();



    // PUBLISHER FOR THE MOTION CAPTURE DATA
    // > Created as a local variable becuase the "looping" of
    //   ros is handled directly in the "while (ros::ok())"
    //   loop below, i.e., this variable will not
    //   go out-of-scope
    ros::Publisher viconDataPublisher =
        nodeHandle.advertise<ViconData>("ViconData", 1);



#ifdef TESTING_FAKE_DATA
    testFakeData(viconDataPublisher);
#else



    // CLIENT FOR GETTING DATA FROM THE VICON DATA STREAM SDK
    // > The "Client" variable type is defined in the header
    //   "DataStreamClient.h"
    Client vicon_client;


    // HOSTNAME (IP ADDRESS) OF THE "VICON COMPUTER"
    // > This is specified in the "ViconConfig.yaml" file
    // > That yaml file is added to this node during launch
    // > The "Vicon Computer" runs the "Tacker" software that
    //   is the heart of the Vicon Motion Capture system
    std::string hostName;
    if(!nodeHandle.getParam("hostName", hostName)) {
        ROS_ERROR("Failed to get hostName");
        return 1;
    }



    // CONNECT TO THE HOST (i.e., TO THE "VICON COMPUTER")

    // Inform the user this is about to be attempted
    ROS_INFO_STREAM("[VICON DATA PUBLISHER] Connecting to Vicon host with name: " << hostName );

    // Attempt to connect in a while loop
    while (!vicon_client.IsConnected().Connected)
    {
        // Get the connection status
        bool ok = (vicon_client.Connect(hostName).Result == Result::Success);

        if (!ok)
        {
            // If failed: inform the user and wait a bit
            ROS_ERROR("[VICON DATA PUBLISHER] ERROR - connection failed... attempting again in 1 second");
            ros::Duration(1.0).sleep();
        }
        else
        {
            // If successful: inform the user
            ROS_INFO("[VICON DATA PUBLISHER] Connected successfully to host");
        }
    }



    // SPECIFY THE SETTING OF THE VICON CLIENT

    // Set "stream mode" parameter
    // > OPTIONS: { ServerPush , ClientPull }
    // > The "ServerPush" option should have the lowest latency
    vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush); 

    // Set what type of data should be provided
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableDeviceData();

    // Set the inertial frame convention such that positive
    // Z points upwards
    vicon_client.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);



    // 
    while (ros::ok())
    {
        // Get a frame
        while (vicon_client.GetFrame().Result != Result::Success) {
            // Sleep a little so that we don't lumber the CPU with a busy poll
            ros::Duration(0.001).sleep();
        }

        // Initilise a "ViconData" struct
        // > This is defined in the "ViconData.msg" file
        ViconData viconData;

        // Unlabeled markers, for GUI
        unsigned int unlabeledMarkerCount = vicon_client.GetUnlabeledMarkerCount().MarkerCount;

        UnlabeledMarker marker;
        for(int unlabeledMarkerIndex = 0; unlabeledMarkerIndex < unlabeledMarkerCount; unlabeledMarkerIndex++)
        {

            Output_GetUnlabeledMarkerGlobalTranslation OutputTranslation =
                vicon_client.GetUnlabeledMarkerGlobalTranslation(unlabeledMarkerIndex);

            marker.index = unlabeledMarkerIndex;
            marker.x = OutputTranslation.Translation[0]/1000.0f;
            marker.y = OutputTranslation.Translation[1]/1000.0f;
            marker.z = OutputTranslation.Translation[2]/1000.0f;

            viconData.markers.push_back(marker);
        }

        unsigned int subjectCount = vicon_client.GetSubjectCount().SubjectCount;

        // //Process the data and publish on topic
        for (int index = 0; index < subjectCount; index++) {
       		std::string subjectName = vicon_client.GetSubjectName(index).SubjectName;
            std::string segmentName = vicon_client.GetSegmentName(subjectName, 0).SegmentName;


            //continue only if the received frame is for the correct crazyflie
            Output_GetSegmentGlobalTranslation outputTranslation =
                    vicon_client.GetSegmentGlobalTranslation(subjectName, segmentName);
            //ROS_INFO_STREAM("translation occluded: " << outputTranslation.Occluded);

            Output_GetSegmentGlobalRotationQuaternion outputRotation =
                    vicon_client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName);
            //ROS_INFO_STREAM("translation occluded: " << outputRotation.Occluded);

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
            Output_GetLatencyTotal outputLatencyTotal = vicon_client.GetLatencyTotal();
            double totalViconLatency;
            if (outputLatencyTotal.Result == Result::Success) {
                totalViconLatency = outputLatencyTotal.Total;
            } else {
                totalViconLatency = 0;
            }

            
            //build message
            CrazyflieData cfData;
            cfData.crazyflieName = subjectName;

            cfData.occluded = outputTranslation.Occluded;

            cfData.x = outputTranslation.Translation[0] / 1000.0f;
            cfData.y = outputTranslation.Translation[1] / 1000.0f;
            cfData.z = outputTranslation.Translation[2] / 1000.0f;
            cfData.roll = roll;
            cfData.pitch = pitch;
            cfData.yaw = yaw;
            cfData.acquiringTime = totalViconLatency;
            // if(!outputTranslation.Occluded)
            viconData.crazyflies.push_back(cfData);
        }
        viconDataPublisher.publish(viconData);
    }
    // END OF "while (ros::ok())"

    // The code only reaches this point if the while loop is
    // broken, hence disable and diconnect the Vicon client
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();

    vicon_client.Disconnect();
#endif
}





void testFakeData(ros::Publisher viconDataPublisher)
{
    // Test faking data part
    float f = 0;
    int i = 0;

    ROS_INFO("[VICON DATA PUBLISHER] TESTING_FAKE_DATA");
    while(ros::ok())
    {
        if(i % 1000 == 0)
        {
            ROS_INFO("iteration #%d",i);
        }

        // Testing piece of code
        ViconData viconData;
        UnlabeledMarker marker;

        marker.index = 0;
        marker.x = f;
        marker.y = 0;
        marker.z = 0;

        viconData.markers.push_back(marker);


        marker.index = 1;
        marker.x = 0;
        marker.y = f;
        marker.z = 0;

        viconData.markers.push_back(marker);

        if(i > 50 && i < 100)
        {
            marker.index = 2;
            marker.x = f;
            marker.y = f;
            marker.z = 0;
            viconData.markers.push_back(marker);
        }

        ros::Duration(0.1).sleep();
        f += 10/1000.0f;
        i++;
        // TODO: Fake CF data
        CrazyflieData crazyfly;

        crazyfly.occluded = false;

        crazyfly.crazyflieName = "CF1";
        crazyfly.x = 0;
        crazyfly.y = 0;
        crazyfly.z = 0;
        crazyfly.yaw = 3.14159 * f;
        viconData.crazyflies.push_back(crazyfly);

        crazyfly.crazyflieName = "CF2";
        crazyfly.x = 1;
        crazyfly.y = 1;
        crazyfly.z = 0;
        crazyfly.yaw = -3.14159 * f;
        viconData.crazyflies.push_back(crazyfly);

        crazyfly.crazyflieName = "CF3";
        crazyfly.x = 1;
        crazyfly.y = -1;
        crazyfly.z = 0;
        crazyfly.yaw = -3.14159 * f;


        if(i > 50 && i < 200)
        {
            crazyfly.occluded = true;
        }

        viconData.crazyflies.push_back(crazyfly);

        viconDataPublisher.publish(viconData); // testing data
    }
}