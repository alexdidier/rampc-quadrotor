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


ViconData previousLocation;



bool calculateControlOutput(RateController::Request &request, RateController::Response &response) {
    ROS_INFO("calculate control output");

    //Philipp: I have put this here, because in the first call, we wouldnt have previousLocation initialized
    //save previous data for calculating velocities in next step
    previousLocation = request.crazyflieLocation;
    
    ViconData vicon = request.crazyflieLocation;
    Setpoint goal = request.setpoint;


    ROS_INFO("request received with following ViconData");
    ROS_INFO_STREAM(vicon);
    ROS_INFO("the goal setpoint is:");
    ROS_INFO_STREAM(request.setpoint);




    //add/calculate safeController
    //K matrix for kLqrOuter
    const float k[] = {
        0,            -1.714330725,            0,             0,   -1.337107465,              0,      5.115369735,             0,            0,
        1.714330725,             0,            0,   1.337107465,              0,              0,                0,   5.115369735,            0,
        0,                       0,            0,             0,              0,              0,                0,             0,  3.843099534,
        0,                       0,   0.22195826,             0,              0,     0.12362477,                0,             0,            0
    };

    float px = request.crazyflieLocation.x - request.setpoint.x;
    float py = request.crazyflieLocation.y - request.setpoint.y;
    float pz = request.crazyflieLocation.z - request.setpoint.z;

    //linear approximation of derivative of position
    float vx = (request.crazyflieLocation.x - previousLocation.x) / request.crazyflieLocation.acquiringTime;
    float vy = (request.crazyflieLocation.y - previousLocation.y) / request.crazyflieLocation.acquiringTime;
    float vz = (request.crazyflieLocation.z - previousLocation.z) / request.crazyflieLocation.acquiringTime;

    float roll = request.crazyflieLocation.roll;
    float pitch = request.crazyflieLocation.pitch;
    float yaw = request.crazyflieLocation.yaw - request.setpoint.yaw;

    response.controlOutput.rollRate = -(k[0] * px + k[1] * py + k[2] * pz + k[3] * vx + k[4] * vy + k[5] * vz + k[6] * roll + k[7] * pitch + k[8] * yaw);
    response.controlOutput.pitchRate = -(k[9] * px + k[10] * py + k[11] * pz + k[12] * vx + k[13] * vy + k[14] * vz + k[15] * roll + k[16] * pitch + k[17] * yaw);
    response.controlOutput.yawRate = -(k[18] * px + k[19] * py + k[20] * pz + k[21] * vx + k[22] * vy + k[23] * vz + k[24] * roll + k[25] * pitch + k[26] * yaw);
    float thrustIntermediate = -(k[27] * px + k[28] * py + k[29] * pz + k[30] * vx + k[31] * vy + k[32] * vz + k[33] * roll + k[34] * pitch + k[35] * yaw);
    //idea: linerazie plot and apply on sum of thrust instead of on each motor
    response.controlOutput.thrust = 20000;


    /*cmd1Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd1Thrust))))/(2*m_a2);
    cmd2Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd2Thrust))))/(2*m_a2);
    cmd3Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd3Thrust))))/(2*m_a2);
    cmd4Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd4Thrust))))/(2*m_a2);
    */


	return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SafeControllerService");

    ros::NodeHandle nodeHandle("~");

    ros::ServiceServer service = nodeHandle.advertiseService("RateController", calculateControlOutput);
    ROS_INFO("SafeControllerService ready to send");
    ros::spin();

    return 0;
}
