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
//CentralManager: assign localArea for each group and those coordinates to PPSClients
//ViconDataPublisher: extract data about room from vicon data in and send also to PPSClient
//PPSClient: Compare data received from CentralManager and ViconDataPublisher and determine in which localArea you are
//PPSClient: Choose correct controller accoring to current localArea


#include "ros/ros.h"
#include <stdlib.h>

//include autogenerated headers from srv files
#include "d_fall_pps/Controller.h"
#include "d_fall_pps/CentralManager.h"

//include autogenerated headers from msg files
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/CrazyflieContext.h"

#include "d_fall_pps/ControlCommand.h"

using namespace d_fall_pps;

//the teamname and the assigned crazyflie, will be extracted from studentParams.yaml
std::string team; //is this needed here? maybe for room asignment received from CentralManager?
std::string cflie;

//global sevices
ros::ServiceClient safeController;
ros::ServiceClient centralClient;

ros::Publisher controlCommandPublisher;

AreaBounds localArea;

void ppsClientToController(ViconData data, bool autocontrolOn){
		//call safecontroller if autocontrol is true

	if(autocontrolOn){
		//TBD: call safecontroller here

		//for the moment just switch the motor off
		ROS_INFO_STREAM("AutocontrolOn >>>>>> SWITCHED OFF");
		ControlCommand switchOffControls;
		switchOffControls.roll = 0;
		switchOffControls.pitch = 0;
		switchOffControls.yaw = 0;
		switchOffControls.motorCmd1 = 0;
		switchOffControls.motorCmd2 = 0;
		switchOffControls.motorCmd3 = 0;
		switchOffControls.motorCmd4 = 0;
		switchOffControls.onboardControllerType = 0;
		controlCommandPublisher.publish(switchOffControls);
	}
	else {
		//student controller is called here
			//for the moment use safecontroller for TESTING
		
		Controller srvRate;
		Setpoint goalLocation;

		goalLocation.x = 0; //testvalue
		goalLocation.y = 0.5; //testvalue
		goalLocation.z = 0.4; //testvalue
		goalLocation.yaw = 0;

		srvRate.request.crazyflieLocation = data;
		srvRate.request.setpoint = goalLocation;

		//TODO:
		//return control commands
		if(safeController.call(srvRate)){
			//ROS_INFO("Received control output");
			//ROS_INFO_STREAM(srvRate.response.controlOutput);
			
			//check attitude an 

			controlCommandPublisher.publish(srvRate.response.controlOutput);
				//onboardControllerType = ??????????????????????
			
			
		} else {
			ROS_ERROR("Failed to call SafeControllerService");
			//return 1; //return some useful stuff
		}
	}
	
}

//acceptance test for crazyflie position
bool safetyCheck(ViconData data){
	CrazyflieContext CrazyflieContext;
	
	//position check
	if((data.x < localArea.xmin) or (data.x > localArea.xmax)){
		return true;
	}
	if((data.y < localArea.ymin) or (data.y > localArea.ymax)){
		return true;
	}
	if((data.z < localArea.zmin) or (data.z > localArea.zmax)){
		return true;
	}
	
	//all checks passed
	return false;
}

//is called upon every new arrival of data in main
void viconCallback(const ViconData& data){
	//ROS_INFO("My teamname is:"); ROS_INFO_STREAM(team);
	//ROS_INFO("My crazyflie is:"); ROS_INFO_STREAM(cflie);

	if(data.crazyflieName == cflie){	
		ROS_INFO_STREAM(data);
		//forward data to safety check
		bool autocontrolOn = safetyCheck(data);
		ppsClientToController(data, autocontrolOn);
	}
	else {
		ROS_INFO("ViconData from other crazyflie received");
	}


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
	
	//ros::Publishers to advertise the control output
	controlCommandPublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);


	//service 
		//to be expanded with additional services depending on controller (currently only one available)
	ros::service::waitForService("/SafeControllerService/RateController");
	safeController = nodeHandle.serviceClient<Controller>("/SafeControllerService/RateController", true);
	
	//safeController = nodeHandle.serviceClient<d_fall_pps::RateController>("/SafeControllerService/RateController", true);
	//http://wiki.ros.org/roscpp/Overview/Services 
	//2.1 Persistenct Connection: ROS also allows for persistent connections to services. With a persistent connection, a client stays connected to a service. 
	// Otherwise, a client normally does a lookup and reconnects to a service each time.


	//service 
	centralClient = nodeHandle.serviceClient<CentralManager>("/CentralManagerService/CentralManager");
	
	
	//TBD: some sort of init procedure to get data from CentralManager upfront
	//this is only for testing>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	CentralManager ManagerSettings;
	if(centralClient.call(ManagerSettings)){

		localArea = ManagerSettings.response.context.localArea;
		ROS_INFO("CentralManager responded");
		ROS_INFO("localAreaBoundaries Set");
		

	}
	else{
		ROS_ERROR("Failed to call CentralManagerService. Callback is aborted");
		//return some useful stuff
		return 0;
	}
	//<<<<<<<<<<<<<<<<<<<<<<<this is only for testing

    ros::spin();
    return 0;
}
