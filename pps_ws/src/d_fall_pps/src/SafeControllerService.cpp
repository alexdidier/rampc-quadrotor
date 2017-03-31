//    Alternate controller that is expected to work.
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
#include "d_fall_pps/Setpoint.h"
#include "d_fall_pps/RateCommand.h"
#include "d_fall_pps/RateController.h"

using namespace d_fall_pps;

bool calculateControlOutput(RateController::Request &request, RateController::Response &response) {
    ROS_INFO("calculate control output");
    
    ViconData vicon = request.crazyflieLocation;
    Setpoint goal = request.setpoint;
    ROS_INFO("request received with following ViconData");
    ROS_INFO_STREAM(vicon);
    ROS_INFO("the goal setpoint is:");
    ROS_INFO_STREAM(request.setpoint);

    //add/calculate safeController
    response.controlOutput.rollRate = 1; //testvalue
	response.controlOutput.pitchRate = 2; //testvalue
	response.controlOutput.yawRate = 3; //testvalue

	return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SafeControllerService");

    ros::NodeHandle nodeHandle("~");

    ros::ServiceServer service = nodeHandle.advertiseService("RateController", calculateControlOutput);
    ROS_INFO("SafeControllerService ready");
    ros::spin();

    return 0;
}
