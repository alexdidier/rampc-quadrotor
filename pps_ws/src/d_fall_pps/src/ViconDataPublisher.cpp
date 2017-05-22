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

#include <string.h>
#include "DataStreamClient.h"
#include "ros/ros.h"
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/UnlabeledMarker.h"


// notice that unit here are in milimeters
using namespace ViconDataStreamSDK::CPP;
using namespace d_fall_pps;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ViconDataPublisher");

    ros::NodeHandle nodeHandle("~");
    ros::Time::init();

    ros::Publisher viconDataPublisher =
        nodeHandle.advertise<ViconData>("ViconData", 1);

    #ifdef TESTING_FAKE_DATA
    // Test faking data part
    float f = 0;
    int i = 0;

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

        if(i > 50 && i < 100)
        {
            crazyfly.crazyflieName = "CF3";
            crazyfly.x = 1;
            crazyfly.y = -1;
            crazyfly.z = 0;
            crazyfly.yaw = -3.14159 * f;
            viconData.crazyflies.push_back(crazyfly);
        }

        viconDataPublisher.publish(viconData); // testing data
    }
    #else


    Client client;

    std::string hostName;
    if(!nodeHandle.getParam("hostName", hostName)) {
        ROS_ERROR("Failed to get hostName");
        return 1;
    }

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
    client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush); //maybe ServerPush instead of ClientPull for less latency

    client.EnableSegmentData();
    client.EnableMarkerData();
    client.EnableUnlabeledMarkerData();
    client.EnableDeviceData();

    // Set the global up axis, such that Z is up
    client.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);

    while (ros::ok()) {
        // Get a frame
        while (client.GetFrame().Result != Result::Success) {
            // Sleep a little so that we don't lumber the CPU with a busy poll
            ros::Duration(0.001).sleep();
        }

        ViconData viconData;

        // Unlabeled markers, for GUI
        unsigned int unlabeledMarkerCount = client.GetUnlabeledMarkerCount().MarkerCount;

        UnlabeledMarker marker;
        ROS_INFO_STREAM("unlabeledMarkerCount: " << unlabeledMarkerCount);

        for(int unlabeledMarkerIndex = 0; unlabeledMarkerIndex < unlabeledMarkerCount; unlabeledMarkerIndex++)
        {

            Output_GetUnlabeledMarkerGlobalTranslation OutputTranslation =
                client.GetUnlabeledMarkerGlobalTranslation(unlabeledMarkerIndex);

            marker.index = unlabeledMarkerIndex;
            marker.x = OutputTranslation.Translation[0]/1000.0f;
            marker.y = OutputTranslation.Translation[1]/1000.0f;
            marker.z = OutputTranslation.Translation[2]/1000.0f;

            viconData.markers.push_back(marker);
        }

        unsigned int subjectCount = client.GetSubjectCount().SubjectCount;

        // //Process the data and publish on topic
        for (int index = 0; index < subjectCount; index++) {
       		std::string subjectName = client.GetSubjectName(index).SubjectName;
            std::string segmentName = client.GetSegmentName(subjectName, 0).SegmentName;


            //continue only if the received frame is for the correct crazyflie
            Output_GetSegmentGlobalTranslation outputTranslation =
                    client.GetSegmentGlobalTranslation(subjectName, segmentName);
            //ROS_INFO_STREAM("translation occluded: " << outputTranslation.Occluded);

            Output_GetSegmentGlobalRotationQuaternion outputRotation =
                    client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName);
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
            Output_GetLatencyTotal outputLatencyTotal = client.GetLatencyTotal();
            double totalViconLatency;
            if (outputLatencyTotal.Result == Result::Success) {
                totalViconLatency = outputLatencyTotal.Total;
            } else {
                totalViconLatency = 0;
            }

            //build message
            CrazyflieData cfData;
            cfData.crazyflieName = subjectName;

            cfData.x = outputTranslation.Translation[0] / 1000.0f;
            cfData.y = outputTranslation.Translation[1] / 1000.0f;
            cfData.z = outputTranslation.Translation[2] / 1000.0f;
            cfData.roll = roll;
            cfData.pitch = pitch;
            cfData.yaw = yaw;
            cfData.acquiringTime = totalViconLatency;

            viconData.crazyflies.push_back(cfData);
        }
        viconDataPublisher.publish(viconData);
    }

    client.DisableSegmentData();
    client.DisableMarkerData();
    client.DisableUnlabeledMarkerData();
    client.DisableDeviceData();

    client.Disconnect();

    #endif
}
