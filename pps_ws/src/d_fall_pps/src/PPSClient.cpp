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

//the teamname and the assigned crazyflie, will be extracted from studentParams.yaml
std::string team; //is this needed here? maybe for room asignment received from CentralManager?
std::string cflie;

//global sevices
ros::ServiceClient rateClient;

//extract data from "data" and publish/add to service for controller
//not void: sould give back controlldata
void PPSClientToController(data){
	if(data.crazyflieName == cflie){
		/* unnecessairy: just send data!!!!!
		d_fall_pps::ViconData myDataToPublish;
		myDataToPublish.crazyflieName = data.crazyflieName;
		myDataToPublish.x = data.x;
		myDataToPublish.y = data.y;
		myDataToPublish.z = data.z;
		myDataToPublish.roll = data.roll;
		myDataToPublish.pitch = data.pitch;
		myDataToPublish.yaw = data.yaw;
		myDataToPublish.acquiringTime = data.acquiringTime;
		*/


		//TODO:
		//Some way of choosing the correct controller: Safe or Custom
		//using the area data

		//TODO:
		//communicating with Controller


		//TODO:
		//return control commands
	}
	else {
		ROS_INFO("ViconData from other crazyflie received");
	}
}


//debugging
int callbackCalls = 0;

//is called upon every new arrival of data in main
void viconCallback(const d_fall_pps::ViconData& data){
	//debugging
	++callbackCalls;
	//ROS_INFO("Callback called #%d",callbackCalls);
	//ROS_INFO("Recived Pitch in this callback: %f", data.pitch);
	//ROS_INFO("received data:"); ROS_INFO_STREAM(data);
	//ROS_INFO("My teamname is:"); ROS_INFO_STREAM(team);
	//ROS_INFO("My crazyflie is:"); ROS_INFO_STREAM(cflie);

	//extract data from "data" and publish/add to service for controller
	PPSClientToController(data);

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
	
	ROS_INFO_STREAM("about to subscribe");
	ros::Subscriber ViconSubscriber = nodeHandle.subscribe("/ViconDataPublisher/ViconData", 1, viconCallback);
	ROS_INFO_STREAM("subscribed");





	//service: now only one available: to add several services depending on controller
	rateClient = nodeHandle.serviceClient<d_fall_pps::RateController>("/SafeControllerService/RateCommand");






    ros::spin();
    return 0;
}
