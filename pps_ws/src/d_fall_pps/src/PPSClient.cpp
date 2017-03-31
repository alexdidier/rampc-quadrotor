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

using namespace d_fall_pps;

//the teamname and the assigned crazyflie, will be extracted from studentParams.yaml
std::string team; //is this needed here? maybe for room asignment received from CentralManager?
std::string cflie;

//global sevices
ros::ServiceClient rateClient;

//extract data from "data" and publish/add to service for controller
//not void: sould give back controlldata
void ppsClientToController(ViconData data){
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
	//ROS_INFO("Recived Pitch in this callback: %f", data.pitch);
	//ROS_INFO("received data:"); ROS_INFO_STREAM(data);
	//ROS_INFO("My teamname is:"); ROS_INFO_STREAM(team);
	//ROS_INFO("My crazyflie is:"); ROS_INFO_STREAM(cflie);

	//extract data from "data" and publish/add to service for controller
	ppsClientToController(data);

}





//Send to Crayzradio>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/*
void CControlMgr::callbackRunRateController(const ros::TimerEvent&)
{
    if(m_pRateController!=NULL)
        if(m_pRateController->fIsInit())
        {
            DistributePowerAndSendToCrazyflie(m_pRateController->computeOutput());
        }
}

void CControlMgr::DistributePowerAndSendToCrazyflie(ControllerOutput rateControllerOutput)
{
    assert(rateControllerOutput.onboardControllerType=eOnboardMotorCmdController);
    assert(m_isRateOffboard==true);


    rateControllerOutput.motorCmd1=m_CrazyControllerOutput.motorCmd1-rateControllerOutput.roll/2.0-rateControllerOutput.pitch/2.0-rateControllerOutput.yaw;
    rateControllerOutput.motorCmd2=m_CrazyControllerOutput.motorCmd2-rateControllerOutput.roll/2.0+rateControllerOutput.pitch/2.0+rateControllerOutput.yaw;
    rateControllerOutput.motorCmd3=m_CrazyControllerOutput.motorCmd3+rateControllerOutput.roll/2.0+rateControllerOutput.pitch/2.0-rateControllerOutput.yaw;
    rateControllerOutput.motorCmd4=m_CrazyControllerOutput.motorCmd4+rateControllerOutput.roll/2.0-rateControllerOutput.pitch/2.0+rateControllerOutput.yaw;

    SendToCrazyflie(rateControllerOutput);
}

void CControlMgr::SendToCrazyflie(ControllerOutput package)
{
    if(m_isStopped)
    {
        m_packageToSend.motorCmd1=0;
        m_packageToSend.motorCmd2=0;
        m_packageToSend.motorCmd3=0;
        m_packageToSend.motorCmd4=0;
        m_packageToSend.onboardControllerType=eOnboardMotorCmdController;
        m_pPublisherControllerOutput->publish(m_packageToSend);
        return;
    }
    else
    {
        m_packageToSend.roll=package.roll*RAD2DEG;
        m_packageToSend.pitch=package.pitch*RAD2DEG;
        m_packageToSend.yaw=package.yaw*RAD2DEG;

        m_packageToSend.thrust=SaturateToUINT16(package.thrust);
        m_packageToSend.motorCmd1=SaturateToUINT16(package.motorCmd1);
        m_packageToSend.motorCmd2=SaturateToUINT16(package.motorCmd2);
        m_packageToSend.motorCmd3=SaturateToUINT16(package.motorCmd3);
        m_packageToSend.motorCmd4=SaturateToUINT16(package.motorCmd4);

        m_packageToSend.onboardControllerType=package.onboardControllerType;

        m_pPublisherControllerOutput->publish(m_packageToSend);
    }
}


*/
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

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
	
	/*
	//publish package_for_crazyradio>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	    //m_pNodeHandle=nodeHandle;
    	//m_pCallbackQueueControlMgr=new ros::CallbackQueue();
    	//m_pNodeHandle->setCallbackQueue(m_pCallbackQueueControlMgr);


    ROS_INFO_STREAM("creating publishers for package_for_crazyradio");
	ros::Pubslisher AngleCommandsPublisher = nodeHandle.advertise <d_fall_pps::AngleCommandsPackage>("AngleCommands", 10));
	ros::Pubslisher AngleCommandsPublisher = nodeHandle.advertise <d_fall_pps::RateCommandsPackage>("RateCommands", 10));
	ros::Pubslisher AngleCommandsPublisher = nodeHandle.advertise <d_fall_pps::MotorCommandsPackage>("MotorCommands", 10));

	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	*/




	//service: now only one available: to add several services depending on controller
	rateClient = nodeHandle.serviceClient<d_fall_pps::RateController>("/SafeControllerService/RateCommand");






    ros::spin();
    return 0;
}
