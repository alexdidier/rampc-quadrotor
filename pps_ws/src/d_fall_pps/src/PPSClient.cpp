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
#include "d_fall_pps/ViconData.h"

//the teamname and the assigned crazyflie, will be extracted from studentParams.yaml
std::string team; //is this needed?
std::string cflie;



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
	if(data.crazyflieName == cflie){
		d_fall_pps::ViconData myDataToPublish;
		myDataToPublish.crazyflieName = data.crazyflieName;
		myDataToPublish.x = data.x;
		myDataToPublish.y = data.y;
		myDataToPublish.z = data.z;
		myDataToPublish.roll = data.roll;
		myDataToPublish.pitch = data.pitch;
		myDataToPublish.yaw = data.yaw;
		myDataToPublish.acquiringTime = data.acquiringTime;
		//ROS_INFO("data to share with right controller:");
		//ROS_INFO_STREAM(myDataToPublish);


		//TODO:
		//Some way of choosing the correct controller: Safe or Custom
	}
	else {
		ROS_INFO("ViconData from other crazyflie received");
	}


}





//Send to Crayzradio>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void callbackRunRateController(const ros::TimerEvent&)
{
      DistributePowerAndSendToCrazyflie(m_pRateController->computeOutput());

}

void DistributePowerAndSendToCrazyflie(ControllerOutput rateControllerOutput)
{
    assert(rateControllerOutput.onboardControllerType=eOnboardMotorCmdController);
    assert(m_isRateOffboard==true);


    rateControllerOutput.motorCmd1=m_CrazyControllerOutput.motorCmd1-rateControllerOutput.roll/2.0-rateControllerOutput.pitch/2.0-rateControllerOutput.yaw;
    rateControllerOutput.motorCmd2=m_CrazyControllerOutput.motorCmd2-rateControllerOutput.roll/2.0+rateControllerOutput.pitch/2.0+rateControllerOutput.yaw;
    rateControllerOutput.motorCmd3=m_CrazyControllerOutput.motorCmd3+rateControllerOutput.roll/2.0+rateControllerOutput.pitch/2.0-rateControllerOutput.yaw;
    rateControllerOutput.motorCmd4=m_CrazyControllerOutput.motorCmd4+rateControllerOutput.roll/2.0-rateControllerOutput.pitch/2.0+rateControllerOutput.yaw;

    SendToCrazyflie(rateControllerOutput);
}

void SendToCrazyflie(ControllerOutput package)
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
	
	
	//publish package_for_crazyradio>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	    //m_pNodeHandle=nodeHandle;
    	//m_pCallbackQueueControlMgr=new ros::CallbackQueue();
    	//m_pNodeHandle->setCallbackQueue(m_pCallbackQueueControlMgr);


    ROS_INFO_STREAM("creating publishers for package_for_crazyradio");
	ros::Pubslisher AngleCommandsPublisher = nodeHandle.advertise <d_fall_pps::AngleCommandsPackage>("AngleCommands", 10));
	ros::Pubslisher AngleCommandsPublisher = nodeHandle.advertise <d_fall_pps::RateCommandsPackage>("RateCommands", 10));
	ros::Pubslisher AngleCommandsPublisher = nodeHandle.advertise <d_fall_pps::MotorCommandsPackage>("MotorCommands", 10));

	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    ros::spin();
    return 0;
}
