///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include <DataStreamClient.h>

#include <iostream>
#include <sstream>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "crazypkg/ViconData.h"
#include <time.h>


using namespace ViconDataStreamSDK::CPP;

#define output_stream  std::cout


int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "ViconDataStreamSDK");
    ros::NodeHandle nodeHandle("~");
    ros::Time::init();
    ros::Publisher publisherViconData=
            nodeHandle.advertise<crazypkg::ViconData>("topicViconData",1);
    crazypkg::ViconData msgViconData;

    ros::Time loopStartTime;
    ros::Time dataAcquiringTime;
    ros::Duration loopDuration;
    double maxLoopDuration=0;

    // Program options
    std::string HostName;
    std::string StreamMode;

    nodeHandle.param<std::string>("HostName",HostName,"10.42.0.15:801");
    nodeHandle.param<std::string>("StreamMode",StreamMode,"ServerPush");

    //variables used to get and process the vicon datastream
    unsigned int SubjectCount;
    unsigned int SubjectIndex;
    std::string SubjectName;
    std::string SegmentName;
    Output_GetSegmentGlobalTranslation OutputTranslation;
    Output_GetSegmentGlobalRotationQuaternion OutputRotation;
    Output_GetSegmentLocalRotationQuaternion OutputRotationLocal;
    Output_GetLatencyTotal OutputLatencyTotal;
    double totalViconLatency,avgViconLatency=0,measurementCount=0,maxViconLatency=0;
    double quat_x, quat_y, quat_z, quat_w, roll, pitch, yaw;
    double quat_x_local, quat_y_local, quat_z_local, quat_w_local, roll_local, pitch_local, yaw_local;


    // Make a new client
    Client MyClient;

    for(int i=0; i != 3; ++i) // repeat to check disconnecting doesn't wreck next connect
    {
        // Connect to a server
        ROS_INFO_STREAM("Connecting to " << HostName << " ...");
        while( !MyClient.IsConnected().Connected )
        {
            // Direct connection

            bool ok = false;
            ok =( MyClient.Connect( HostName ).Result == Result::Success );

            if(!ok)
            {
                ROS_ERROR("Warning - connection failed...");
                ros::Duration(1.0).sleep();
            }
            else
            {
                ROS_INFO("Connected successfully");
            }

        }
        ROS_INFO_STREAM(std::endl);

        // Enable some different data types
        MyClient.EnableSegmentData();
        MyClient.EnableMarkerData();
        MyClient.EnableUnlabeledMarkerData();
        MyClient.EnableDeviceData();


        // Set the streaming mode
        if(StreamMode=="ClientPull")
        {
            MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
            ROS_INFO_STREAM("Client Pull stream mode enabled");
        }
        else if(StreamMode=="ClientPullPreFetch")
        {
            MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
            ROS_INFO_STREAM("Client Pull Pre Fetch stream mode enabled");
        }
        else if(StreamMode=="ServerPush")
        {
            MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );
            ROS_INFO_STREAM("Server Push stream mode enabled");
        }
        else
        {
            ROS_ERROR_STREAM("stream mode not reckognized");
        }

        // Set the global up axis
        MyClient.SetAxisMapping( Direction::Forward,
                                 Direction::Left,
                                 Direction::Up ); // Z-up
        // MyClient.SetGlobalUpAxis( Direction::Forward,
        //                           Direction::Up,
        //                           Direction::Right ); // Y-up


        ros::Duration(0.1).sleep();
        loopStartTime=ros::Time::now();


        //loop infinitely
        ROS_INFO_STREAM("Starting Vicon data transfer..."<<std::endl);

        while( ros::ok())
        {
            // Get a frame
            while( MyClient.GetFrame().Result != Result::Success )
            {
                // Sleep a little so that we don't lumber the CPU with a busy poll
                ros::Duration(0.001).sleep();
                output_stream << ".";
            }

            loopStartTime=ros::Time::now();

            //Get frame rate
            //            Output_GetFrameRate OutputFrameRate = MyClient.GetFrameRate ();
            //            if(OutputFrameRate.Result==Result::Success)
            //                std::cout<<"Frame Rate= "<<OutputFrameRate.FrameRateHz<<std::endl;
            //            else
            //                std::cout<<"Frame Rate not fetched ERROR"<<std::endl;

            //Get latency
            OutputLatencyTotal = MyClient.GetLatencyTotal();
            if(OutputLatencyTotal.Result==Result::Success)
                totalViconLatency=OutputLatencyTotal.Total;
            else
                totalViconLatency=0;
            //std::cout<<"Total latency: "<<totalViconLatency;
            dataAcquiringTime=loopStartTime-ros::Duration(totalViconLatency);


            // Count the number of subjects
            SubjectCount = MyClient.GetSubjectCount().SubjectCount;

            //Process the data and publish on topic
            for( SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
            {

                // Get the subject's name
                SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
                SegmentName = MyClient.GetSegmentName( SubjectName, SubjectIndex ).SegmentName;

                // Get global translations
                OutputTranslation =
                        MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                // Get global rotations
                OutputRotation =
                        MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );

                OutputRotationLocal =
                        MyClient.GetSegmentLocalRotationQuaternion( SubjectName, SegmentName );

                quat_x = OutputRotation.Rotation [ 0 ];
                quat_y = OutputRotation.Rotation [ 1 ];
                quat_z = OutputRotation.Rotation [ 2 ];
                quat_w = OutputRotation.Rotation [ 3 ];

                quat_x_local = OutputRotationLocal.Rotation [ 0 ];
                quat_y_local = OutputRotationLocal.Rotation [ 1 ];
                quat_z_local = OutputRotationLocal.Rotation [ 2 ];
                quat_w_local = OutputRotationLocal.Rotation [ 3 ];

                //TODO check whether this transformation is correct
                roll = atan2(2 * (quat_w * quat_x + quat_y * quat_z), 1 - 2 * (quat_x * quat_x + quat_y * quat_y));
                pitch = asin(2 * (quat_w * quat_y - quat_z * quat_x));
                yaw = atan2(2 * (quat_w * quat_z + quat_x * quat_y), 1 - 2 * (quat_y * quat_y + quat_z * quat_z));

                roll_local = atan2(2 * (quat_w_local * quat_x_local + quat_y_local * quat_z_local), 1 - 2 * (quat_x_local * quat_x_local + quat_y_local * quat_y_local));
                pitch_local = asin(2 * (quat_w_local * quat_y_local - quat_z_local * quat_x_local));
                yaw_local = atan2(2 * (quat_w_local * quat_z_local + quat_x_local * quat_y_local), 1 - 2 * (quat_y_local * quat_y_local + quat_z_local * quat_z_local));



                msgViconData.x=OutputTranslation.Translation[ 0 ];
                msgViconData.y=OutputTranslation.Translation[ 1 ];
                msgViconData.z=OutputTranslation.Translation[ 2 ];
                msgViconData.roll=roll;
                msgViconData.pitch=pitch;
                msgViconData.yaw=yaw;
//                msgViconData.rollLocal=roll_local;
//                msgViconData.pitchLocal=pitch_local;
//                msgViconData.yawLocal=yaw_local;
                msgViconData.acquiringTime=dataAcquiringTime.toSec();


                //compute the loop time
                //            loopDuration=ros::Time::now()-loopStartTime;
                //            if(loopDuration.toSec()>maxLoopDuration)
                //                maxLoopDuration=loopDuration.toSec();
                //                std::cout<<"loop execution time: "<<loopDuration.toSec()<<",  max loop duration: "<<
                //                           maxLoopDuration<<std::endl;
//                msgViconData.latencyTotal=totalViconLatency+
//                        (ros::Time::now()-loopStartTime).toSec();

                //                std::cout<<"total vicon latency: "<<totalViconLatency<<"  loop time: "
                //                        <<(ros::Time::now()-loopStartTime).toSec()<<std::endl;

                publisherViconData.publish(msgViconData);

//                int countLat=MyClient.GetLatencySampleCount().Count;
//                ROS_DEBUG_STREAM("vicon latency count: "<<countLat);
//                for(int i=0;i<countLat;i++)
//                    ROS_DEBUG_STREAM("vicon latency #"<<i<<": "<<
//                                     MyClient.GetLatencySampleName( i ).Name
//                                     <<" latency value: "<<
//                                     MyClient.GetLatencySampleValue(MyClient.GetLatencySampleName( i ).Name).Value*1000);


//                ROS_DEBUG_STREAM_THROTTLE_NAMED(0.5,"vicon_data_latencyTotal",
//                                                "acquiring time est: "<<dataAcquiringTime<<
//                                                " total Vicon Latency: "<<
//                                                totalViconLatency<<std::endl);


//                Output_GetSegmentGlobalRotationEulerXYZ Output11 =
//                        MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );

//                Output_GetSegmentLocalRotationEulerXYZ Output12 =
//                        MyClient.GetSegmentLocalRotationEulerXYZ( SubjectName, SegmentName );

//                std::cout<<OutputTranslation.Translation[ 0 ]  << ",";
//                std::cout << OutputTranslation.Translation[ 1 ]  << ",";
//                std::cout << OutputTranslation.Translation[ 2 ]  << ",";
//                std::cout << roll  << ",";
//                std::cout << pitch  << ",";
//                std::cout << yaw << ",";
//                std::cout << roll_local  << ",";
//                std::cout << pitch_local  << ",";
//                std::cout << yaw_local << ",";
//                std::cout << std::endl;

//                std::cout<< Output11.Rotation [ 0 ] << ",";
//                std::cout << Output11.Rotation [ 1 ]  << ",";
//                std::cout << Output11.Rotation [ 2 ]  << ",";
//                std::cout << Output12.Rotation [ 0 ]  << ",";
//                std::cout << Output12.Rotation [ 1 ] << ",";
//                std::cout << Output12.Rotation [ 2 ]  << ",";
//                std::cout << std::endl;
            }
        }

        MyClient.DisableSegmentData();
        MyClient.DisableMarkerData();
        MyClient.DisableUnlabeledMarkerData();
        MyClient.DisableDeviceData();


        // Disconnect and dispose
        ROS_WARN_STREAM(" Disconnecting..." << std::endl);
        MyClient.Disconnect();
        ROS_WARN_STREAM(" Disconnected" << std::endl);

    }
}
