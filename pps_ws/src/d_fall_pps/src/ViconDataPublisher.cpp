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

using namespace ViconDataStreamSDK::CPP;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ViconDataPublisher");

    ros::NodeHandle nodeHandle("~");
    ros::Time::init();

    ros::Publisher viconDataPublisher =
            nodeHandle.advertise<d_fall_pps::ViconData>("ViconData", 1);

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
    client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);

    client.EnableSegmentData();
    client.EnableMarkerData();
    client.EnableUnlabeledMarkerData();
    client.EnableDeviceData();

    // Set the global up axis
    client.SetAxisMapping(Direction::Forward,
            Direction::Left,
            Direction::Up);

    while (ros::ok()) {
        // Get a frame
        while (client.GetFrame().Result != Result::Success) {
            // Sleep a little so that we don't lumber the CPU with a busy poll
            ros::Duration(0.001).sleep();
        }

        unsigned int subjectCount = client.GetSubjectCount().SubjectCount;

        // //Process the data and publish on topic
        for (int index = 0; index < subjectCount; index++) {
            std::string subjectName = client.GetSubjectName(index).SubjectName;
            std::string segmentName = client.GetSegmentName(subjectName, index).SegmentName;

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
            d_fall_pps::ViconData viconData;
            viconData.crazyflieName = subjectName;
            viconData.x = outputTranslation.Translation[0];
            viconData.y = outputTranslation.Translation[1];
            viconData.z = outputTranslation.Translation[2];
            viconData.roll = roll;
            viconData.pitch = pitch;
            viconData.yaw = yaw;
            viconData.acquiringTime = totalViconLatency;

            viconDataPublisher.publish(viconData);
        }

    }

    client.DisableSegmentData();
    client.DisableMarkerData();
    client.DisableUnlabeledMarkerData();
    client.DisableDeviceData();

    client.Disconnect();
}
