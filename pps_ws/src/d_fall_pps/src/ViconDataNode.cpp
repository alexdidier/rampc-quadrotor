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

#include <string.h>
#include "DataStreamClient.h"
#include "ros/ros.h"

using namespace ViconDataStreamSDK::CPP;

void connect(Client* client) {
    std::string hostName = "10.42.00.15:801";
	ROS_INFO_STREAM("Connecting to " << hostName << " ...");
    while( !client->IsConnected().Connected )
    {
        bool ok =( client->Connect( hostName ).Result == Result::Success );

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
}

void setupDataStream(Client* client) {
	// Enable some different data types
        ROS_INFO_STREAM("ClientPull stream mode enabled3");
        client->EnableSegmentData();
        client->EnableMarkerData();
        client->EnableUnlabeledMarkerData();
        client->EnableDeviceData();
        ROS_INFO_STREAM("ClientPull stream mode enabled4");

        // Set the streaming mode
        client->SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
        ROS_INFO_STREAM("ClientPull stream mode enabled");

        // Set the global up axis
        client->SetAxisMapping( Direction::Forward,
                                 Direction::Left,
                                 Direction::Up ); // Z-up
        // client->SetGlobalUpAxis( Direction::Forward,
        //                           Direction::Up,
        //                           Direction::Right ); // Y-up
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "ViconDataStreamSDK");
    ROS_INFO_STREAM("Okili dokili");

    ros::NodeHandle nodeHandle("~");
    ros::Time::init();

    Client client;
    connect(&client);
    setupDataStream(&client);

    while(ros::ok()) {
    	            // Get a frame
            while( client.GetFrame().Result != Result::Success )
            {
                // Sleep a little so that we don't lumber the CPU with a busy poll
                ros::Duration(0.001).sleep();
            }
            
            // Count the number of subjects
            unsigned int subjectCount = client.GetSubjectCount().SubjectCount;

            // //Process the data and publish on topic
             for(int index = 0 ; index < subjectCount ; index++ )
             {
             	std::string subjName = client.GetSubjectName( index ).SubjectName;
             	std::string segName = client.GetSegmentName( subjName, index ).SegmentName;

                 ROS_INFO_STREAM(subjName);
                 ROS_INFO_STREAM(segName);
                 ROS_INFO_STREAM("------");
             }

    }
/*
        while( ros::ok())
        {
            // Get a frame
            while( client.GetFrame().Result != Result::Success )
            {
                // Sleep a little so that we don't lumber the CPU with a busy poll
                ros::Duration(0.001).sleep();
            }

            //Get frame rate
            //            Output_GetFrameRate OutputFrameRate = client.GetFrameRate ();
            //            if(OutputFrameRate.Result==Result::Success)
            //                std::cout<<"Frame Rate= "<<OutputFrameRate.FrameRateHz<<std::endl;
            //            else
            //                std::cout<<"Frame Rate not fetched ERROR"<<std::endl;

            // //Get latency
            // OutputLatencyTotal = client.GetLatencyTotal();
            // if(OutputLatencyTotal.Result==Result::Success)
            //     totalViconLatency=OutputLatencyTotal.Total;
            // else
            //     totalViconLatency=0;
            // //std::cout<<"Total latency: "<<totalViconLatency;
            // dataAcquiringTime=loopStartTime-ros::Duration(totalViconLatency);


            // Count the number of subjects
            int subjectCount = client.GetSubjectCount().SubjectCount;
            ROS_INFO_STREAM();

            // //Process the data and publish on topic
            // for( SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
            // {

            //     // Get the subject's name
            //     subjectName = client.GetSubjectName( subjectIndex ).SubjectName;
            //     segmentName = client.GetSegmentName( subjectName, subjectIndex ).SegmentName;
            //     ROS_INFO_STREAM();
            // }
        }

        client.DisableSegmentData();
        client.DisableMarkerData();
        client.DisableUnlabeledMarkerData();
        client.DisableDeviceData();


        // Disconnect and dispose
        ROS_WARN_STREAM(" Disconnecting..." << std::endl);
        client.Disconnect();
        ROS_WARN_STREAM(" Disconnected" << std::endl);
*/
}
