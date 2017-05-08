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

#include "ros/ros.h"
#include <stdlib.h>

#include "d_fall_pps/Controller.h"
#include "d_fall_pps/CentralManager.h"

#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "std_msgs/Int32.h"

#include "d_fall_pps/ControlCommand.h"

#define CMD_USE_SAFE_CONTROLLER 1
#define CMD_USE_CUSTOM_CONTROLLER 2

using namespace d_fall_pps;

//name of the student team
std::string teamName;
//name of the crazyflie, as specified in Vicon
std::string crazyflieName;

//the safe controller specified in the ClientConfig.yaml, is considered trusted
ros::ServiceClient safeController;
//the custom controller specified in the ClientConfig.yaml, is considered untrusted
ros::ServiceClient customController;

ros::ServiceClient centralManager;
ros::Publisher controlCommandPublisher;

//describes the area of the crazyflie and other parameters
CrazyflieContext context;
//wheter to use safe of custom controller
bool usingSafeController;

//checks if crazyflie is within allowed area and if custom controller returns no data
bool safetyCheck(ViconData data, ControlCommand controlCommand) {
	CrazyflieContext CrazyflieContext;
	
	//position check
	if((data.x < context.localArea.xmin) or (data.x > context.localArea.xmax)) {
		return true;
	}
	if((data.y < context.localArea.ymin) or (data.y > context.localArea.ymax)) {
		return true;
	}
	if((data.z < context.localArea.zmin) or (data.z > context.localArea.zmax)) {
		return true;
	}

	return false;
}

//is called when new data from Vicon arrives
void viconCallback(const ViconData& data) {
	if(data.crazyflieName == crazyflieName) {	
		Controller controllerCall;
		controllerCall.request.crazyflieLocation = data;

		if(!usingSafeController) {
			bool success = customController.call(controllerCall);
			if(!success) {
				ROS_ERROR("Failed to call custom controller, switching to safe controller");
				usingSafeController = true;
			}

			//usingSafeController = !safetyCheck(data, controllerCall.response.controlOutput);
			usingSafeController = true; //debug
		}

		if(usingSafeController) {
			bool success = safeController.call(controllerCall);
			if(!success) {
				ROS_ERROR("Failed to call safe controller");
			}
		}

		controlCommandPublisher.publish(controllerCall.response.controlOutput);
	}
}

void loadParameters(ros::NodeHandle& nodeHandle) {
	if(!nodeHandle.getParam("teamName", teamName)) {
		ROS_ERROR("Failed to get teamName");
	}

	if(!nodeHandle.getParam("crazyFlieName", crazyflieName)) {
		ROS_ERROR("Failed to get crazyFlieName");
	}
}

void loadCrazyflieContext() {
	CentralManager centralManagerCall;
	if(centralManager.call(centralManagerCall)) {
		context = centralManagerCall.response.context;
		ROS_INFO("CrazyflieContext obtained");
	} else {
		ROS_ERROR("Failed to call CentralManagerService");
	}
}

void loadSafeController() {
	ros::NodeHandle nodeHandle("~");

	std::string safeControllerName;
	if(!nodeHandle.getParam("safeController", safeControllerName)) {
		ROS_ERROR("Failed to get safe controller name");
		return;
	}

	ros::service::waitForService(safeControllerName);
	safeController = nodeHandle.serviceClient<Controller>(safeControllerName, true);
    ROS_INFO_STREAM("loaded safe controller " << safeControllerName);
}

void loadCustomController() {
	ros::NodeHandle nodeHandle("~");

	std::string customControllerName;
	if(!nodeHandle.getParam("customController", customControllerName)) {
		ROS_ERROR("Failed to get custom controller name");
		return;
	}

	customController = nodeHandle.serviceClient<Controller>(customControllerName, true);
    ROS_INFO_STREAM("loaded custom controller " << customControllerName);
}

void commandCallback(const std_msgs::Int32& commandMsg) {
	int cmd = commandMsg.data;
	switch(cmd) {
    	case CMD_USE_SAFE_CONTROLLER:
    		loadSafeController();
    		usingSafeController = true;
    		break;

    	case CMD_USE_CUSTOM_CONTROLLER:
    		loadCustomController();
    		usingSafeController = false;
    		break;

    	default:
    		ROS_ERROR_STREAM("unexpected command number: " << cmd);
    		break;
	}
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "PPSClient");
	ros::NodeHandle nodeHandle("~");
	loadParameters(nodeHandle);
	
	//keeps 100 messages because otherwise ViconDataPublisher would override the data immediately
	ros::Subscriber viconSubscriber = nodeHandle.subscribe("/ViconDataPublisher/ViconData", 100, viconCallback);
	ROS_INFO_STREAM("successfully subscribed to ViconData");
	
	//ros::Publishers to advertise the control output
	controlCommandPublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);

	//this topic lets the PPSClient listen to the terminal commands
    ros::Publisher commandPublisher = nodeHandle.advertise<std_msgs::Int32>("Command", 1);
    ros::Subscriber commandSubscriber = nodeHandle.subscribe("/PPSClient/Command", 1, commandCallback);

	//start with safe controller
	usingSafeController = true;
	loadSafeController();

	//ros::service::waitForService("/CentralManagerService/CentralManager");
	centralManager = nodeHandle.serviceClient<CentralManager>("/CentralManagerService/CentralManager");
	loadCrazyflieContext();

    ros::spin();
    return 0;
}
