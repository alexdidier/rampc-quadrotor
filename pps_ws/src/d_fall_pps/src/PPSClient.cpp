//    ROS node that manages the student's setup.
//    Copyright (C) 2017  Cyrill Burgener, Marco Mueller, Philipp Friedli
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


#include "ros/ros.h"
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/RateController.h"
#include "d_fall_pps/AngleCommand.h"
#include "d_fall_pps/RateCommand.h"
#include "d_fall_pps/MotorCommand.h"

using namespace d_fall_pps;

//the teamname and the assigned crazyflie, will be extracted from studentParams.yaml
std::string team; //is this needed here? maybe for room asignment received from CentralManager?
std::string cflie;

//global sevices
ros::ServiceClient rateClient;

ros::Publisher angleCommandPublisher;
ros::Publisher rateCommandPublisher;
ros::Publisher motorCommandPublisher;

//uncommenting the next line causes FATAL Error at runtime: "You must call ros::init() before creating the first NodeHandle"
//ros::NodeHandle nodeHandle;


//extract data from "data" and publish/add to service for controller
//not void: sould give back controlldata
void ppsClientToController(ViconData data){
	if(data.crazyflieName == cflie){
		//TODO:
		//Some way of choosing the correct controller: Safe or Custom
		//using the area data

		//TODO:
		//communicating with Controller
		RateController srvRate;
		Setpoint goalLocation;

		goalLocation.x = 9; //testvalue
		goalLocation.y = 8; //testvalue
		goalLocation.z = 7; //testvalue

		srvRate.request.crazyflieLocation = data;
		srvRate.request.setpoint = goalLocation;

		//TODO:
		//return control commands
		if(rateClient.call(srvRate)){
			ROS_INFO("Service gave response");
			ROS_INFO("Received control input");
			ROS_INFO_STREAM(srvRate.response.controlOutput);
		}
		else{
			ROS_ERROR("Failed to call SafeControllerService");
			//return 1; //return some useful stuff
		}
	}
	else {
		ROS_INFO("ViconData from other crazyflie received");
	}
}


//debugging
//int callbackCalls = 0;

//is called upon every new arrival of data in main
void viconCallback(const d_fall_pps::ViconData& data){
	//debugging
	//++callbackCalls;
	//ROS_INFO("Callback called #%d",callbackCalls);
	//ROS_INFO("Received Pitch in this callback: %f", data.pitch);
	//ROS_INFO("received data:"); ROS_INFO_STREAM(data);
	//ROS_INFO("My teamname is:"); ROS_INFO_STREAM(team);
	//ROS_INFO("My crazyflie is:"); ROS_INFO_STREAM(cflie);

	//extract data from "data" and publish/add to service for controller
	ppsClientToController(data);

}



//callback method to publish d_fall_pps::AngleCommand
void callbackAngleCommand(const ros::TimerEvent&)
{
	d_fall_pps::AngleCommand angleCommandPkg;
	angleCommandPkg.rollAngle = 1;
	angleCommandPkg.pitchAngle = 1;
	angleCommandPkg.yawAngle = 1;
	angleCommandPkg.thrust = 50;
	
	angleCommandPublisher.publish(angleCommandPkg);
	ROS_INFO_STREAM("AngleCommandTimer pubslishes: " << angleCommandPkg.rollAngle << ", " << angleCommandPkg.pitchAngle << ", " << angleCommandPkg.yawAngle);
}

//callback method to publish d_fall_pps::RateCommand
void callbackRateCommand(const ros::TimerEvent&)
{
	d_fall_pps::RateCommand rateCommandPkg;
	rateCommandPkg.rollRate = 2;
	rateCommandPkg.pitchRate = 2;
	rateCommandPkg.yawRate = 2;
	rateCommandPkg.thrust = 50;
	
	rateCommandPublisher.publish(rateCommandPkg);
	ROS_INFO_STREAM("RateCommandTimer pubslishes: " << rateCommandPkg.rollRate << ", " << rateCommandPkg.pitchRate << ", " << rateCommandPkg.yawRate);
}

//callback method to publish d_fall_pps::MotorCommand
void callbackMotorCommand(const ros::TimerEvent&)
{
	d_fall_pps::MotorCommand motorCommandPkg;
	motorCommandPkg.cmd1 = 3;
	motorCommandPkg.cmd2 = 3;
	motorCommandPkg.cmd3 = 3;
	motorCommandPkg.cmd4 = 3;
	
	motorCommandPublisher.publish(motorCommandPkg);
	ROS_INFO_STREAM("MotorCommandTimer pubslishes: " << motorCommandPkg.cmd1 << ", " << motorCommandPkg.cmd2 << ", " << motorCommandPkg.cmd3 << ", " << motorCommandPkg.cmd4);
}




int main(int argc, char* argv[]){
	ROS_INFO_STREAM("PPSClient started");

	ros::init(argc, argv, "PPSClient");
	ros::NodeHandle nodeHandle("~");

	//get the params defined in studentParams.yaml
	if(!nodeHandle.getParam("TeamName",team)){
		ROS_ERROR("Failed to get TeamName");
	}

	if(!nodeHandle.getParam("CrazyFlieName",cflie)){
		ROS_ERROR("Failed to get CrazyFlieName");
	}
	
	ros::Subscriber ViconSubscriber = nodeHandle.subscribe("/ViconDataPublisher/ViconData", 1, viconCallback);
	ROS_INFO_STREAM("successfully subscribed to ViconData");
	
	
	//ros::Timers to call method that publishes controller outputs for crayzradio node
	/*
	Timers let you schedule a callback to happen at a specific rate through the same callback queue mechanism used by subscription, service, etc. callbacks. 
	Timers are not a realtime thread/kernel replacement, rather they are useful for things that do not have hard realtime requirements. 
	Reference: http://wiki.ros.org/roscpp/Overview/Timers
	*/
    ROS_INFO("creating publishers for package_for_crazyradio");
	ros::Timer angleCommandTimer = nodeHandle.createTimer(ros::Duration(0.1), callbackAngleCommand);
	ros::Timer rateCommandTimer = nodeHandle.createTimer(ros::Duration(0.1), callbackRateCommand);
	ros::Timer motorCommandTimer = nodeHandle.createTimer(ros::Duration(0.1), callbackMotorCommand);
	
	
	//ros::Publishers to advertise on the three command type topics
	angleCommandPublisher = nodeHandle.advertise <d_fall_pps::AngleCommand>("AngleCommand", 1);
	rateCommandPublisher = nodeHandle.advertise<d_fall_pps::RateCommand>("RateCommand", 1);
	motorCommandPublisher = nodeHandle.advertise <d_fall_pps::MotorCommand>("MotorCommand", 1);


	//service: now only one available: to add several services depending on controller
	rateClient = nodeHandle.serviceClient<d_fall_pps::RateController>("/SafeControllerService/RateController");



    ros::spin();
    return 0;
}
