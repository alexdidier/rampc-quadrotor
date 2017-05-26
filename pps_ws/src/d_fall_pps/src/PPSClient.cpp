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
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <ros/package.h>

#include "d_fall_pps/Controller.h"
#include "d_fall_pps/CMQuery.h"

#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "std_msgs/Int32.h"


#include "d_fall_pps/ControlCommand.h"

#define CMD_USE_SAFE_CONTROLLER 1
#define CMD_USE_CUSTOM_CONTROLLER 2
#define CMD_USE_CRAZYFLY_ENABLE 3
#define CMD_USE_CRAZYFLY_DISABLE 4

using namespace d_fall_pps;

//name of the crazyflie, as specified in Vicon
std::string crazyflieName;
//studentID, gives namespace and identifier in CentralManagerService
int studentID;

//the safe controller specified in the ClientConfig.yaml, is considered trusted
ros::ServiceClient safeController;
//the custom controller specified in the ClientConfig.yaml, is considered untrusted
ros::ServiceClient customController;

ros::ServiceClient centralManager;
ros::Publisher controlCommandPublisher;

rosbag::Bag bag;

//describes the area of the crazyflie and other parameters
CrazyflieContext context;

//gather information about other crazyflies --------------------------------------------------------------------------------
/*bool getOtherCrazyflies;
bool getAllCrazyflies;
std::vector<Setpoint> otherSetpoints;
*/
//------------------------------------------------------------------------------------

//wheter to use safe of custom controller
bool usingSafeController;
//wheter crazyflie is enabled (ready to fly) or disabled (motors off)
bool crazyflieEnabled;

int safetyDelay;

//checks if crazyflie is within allowed area and if custom controller returns no data
bool safetyCheck(CrazyflieData data, ControlCommand controlCommand) {
	
	//position check
	if((data.x < context.localArea.xmin) or (data.x > context.localArea.xmax)) {
		safetyDelay--;
		return false;
	}
	if((data.y < context.localArea.ymin) or (data.y > context.localArea.ymax)) {
		safetyDelay--;
		return false;
	}
	if((data.z < context.localArea.zmin) or (data.z > context.localArea.zmax)) {
		safetyDelay--;
		return false;
	}

	
	return true;
}

void coordinatesToLocal(CrazyflieData& cf) {
	AreaBounds area = context.localArea;
	float originX = (area.xmin + area.xmax) / 2.0;
	float originY = (area.ymin + area.ymax) / 2.0;
	float originZ = (area.zmin + area.zmax) / 2.0;

	cf.x -= originX;
	cf.y -= originY;
	cf.z -= originZ;
}

//is called when new data from Vicon arrives
void viconCallback(const ViconData& viconData) {
	for(std::vector<CrazyflieData>::const_iterator it = viconData.crazyflies.begin(); it != viconData.crazyflies.end(); ++it) {
		CrazyflieData global = *it;
		
		if(global.crazyflieName == context.crazyflieName) {
			Controller controllerCall;
			
			CrazyflieData local = global;
			coordinatesToLocal(local);
			controllerCall.request.ownCrazyflie = local;
			
			if(crazyflieEnabled){
				if(!usingSafeController) {
					bool success = customController.call(controllerCall);
					
					if(!success) {
						ROS_ERROR("Failed to call custom controller, switching to safe controller");
						ROS_ERROR_STREAM("custom controller status: valid: " << customController.isValid() << ", exists: " << customController.exists());
						ROS_ERROR_STREAM("custom controller name: " << customController.getService());
						usingSafeController = true;
					} else {
						usingSafeController = !safetyCheck(global, controllerCall.response.controlOutput);
						ROS_INFO_STREAM("safety check failed, switching to safe controller");
					}
				}

				
				if(usingSafeController) {
					bool success = safeController.call(controllerCall);
					if(!success) {
						ROS_ERROR_STREAM("Failed to call safe controller, valid: " << safeController.isValid() << ", exists: " << safeController.exists());
					}
				}
				
				//ROS_INFO_STREAM("safe controller active: " << usingSafeController);

				controlCommandPublisher.publish(controllerCall.response.controlOutput);

				bag.write("ViconData", ros::Time::now(), local);
				bag.write("ControlOutput", ros::Time::now(), controllerCall.response.controlOutput);
			
			} else { //crazyflie disabled
				ControlCommand zeroOutput = ControlCommand(); //everything set to zero
				zeroOutput.onboardControllerType = 2; //set to motor_mode
				controlCommandPublisher.publish(zeroOutput);
				bag.write("ViconData", ros::Time::now(), local);
				bag.write("ControlOutput", ros::Time::now(), zeroOutput);
			}
		}
	}
}

void loadParameters(ros::NodeHandle& nodeHandle) {
	if(!nodeHandle.getParam("studentID", studentID)) {
		ROS_ERROR("Failed to get studentID");
		studentID = 5;
	}
}

void loadCrazyflieContext() {
	CMQuery contextCall;
	contextCall.request.studentID = studentID;
	studentID = 5;
	ROS_INFO_STREAM("myID:" << studentID);
	
	centralManager.waitForExistence(ros::Duration(-1));

	if(centralManager.call(contextCall)) {
		context = contextCall.response.crazyflieContext;
		ROS_INFO_STREAM("CrazyflieContext:\n" << context);
	} else {
		ROS_ERROR("Failed to load context");
	}

	ros::NodeHandle nh("CrazyRadio");
	nh.setParam("crazyFlieAddress", context.crazyflieAddress);
}

void loadSafeController() {
	ros::NodeHandle nodeHandle("~");

	std::string safeControllerName;
	if(!nodeHandle.getParam("safeController", safeControllerName)) {
		ROS_ERROR("Failed to get safe controller name");
		return;
	}

	ros::service::waitForService(safeControllerName);
	safeController = ros::service::createClient<Controller>(safeControllerName, true);
    ROS_INFO_STREAM("loaded safe controller: " << safeController.getService());
}

void loadCustomController() {
	ros::NodeHandle nodeHandle("~");

	std::string customControllerName;
	if(!nodeHandle.getParam("customController", customControllerName)) {
		ROS_ERROR("Failed to get custom controller name");
		return;
	}

	customController = ros::service::createClient<Controller>(customControllerName, true);
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

    	case CMD_USE_CRAZYFLY_ENABLE:
    		crazyflieEnabled = true;
    		break;

    	case CMD_USE_CRAZYFLY_DISABLE:
    		crazyflieEnabled = false;
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
	
	
	//ros::service::waitForService("/CentralManagerService/CentralManager");
	centralManager = nodeHandle.serviceClient<CMQuery>("/CentralManagerService/Query", false);
	loadCrazyflieContext();
	
	//keeps 100 messages because otherwise ViconDataPublisher would override the data immediately
	ros::Subscriber viconSubscriber = nodeHandle.subscribe("/ViconDataPublisher/ViconData", 100, viconCallback);
	ROS_INFO_STREAM("successfully subscribed to ViconData");
	
	//ros::Publishers to advertise the control output
	controlCommandPublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);

	//this topic lets the PPSClient listen to the terminal commands
    ros::Publisher commandPublisher = nodeHandle.advertise<std_msgs::Int32>("Command", 1);
    ros::Subscriber commandSubscriber = nodeHandle.subscribe("Command", 1, commandCallback);

	//start with safe controller
	crazyflieEnabled = true;
	usingSafeController = true;
	loadSafeController();
	
	std::string package_path;
	package_path = ros::package::getPath("d_fall_pps") + "/";
	ROS_INFO_STREAM(package_path);
	std::string record_file = package_path + "LoggingPPSClient.bag";
	bag.open(record_file, rosbag::bagmode::Write);

    ros::spin();
	bag.close();
    return 0;
}
