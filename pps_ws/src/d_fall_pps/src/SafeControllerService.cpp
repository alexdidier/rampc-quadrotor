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

#include <math.h>
#include "ros/ros.h"
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/Setpoint.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/Controller.h"

using namespace d_fall_pps;


ViconData previousLocation;
ControlCommand previousCommand;
float ffThrust1; float ffThrust2; float ffThrust3; float ffThrust4;

const float a2=2.130295e-11;
const float a1=1.032633e-6;
const float a0=5.484560e-4;
const float saturationThrust = a2*12000*12000 + a1*12000 + a0;

const float feedforwardm1 = 37000;
const float feedforwardm2 = 37000;
const float feedforwardm3 = 37000;
const float feedforwardm4 = 37000;


void ffSetup()
{
    ffThrust1 = a2*feedforwardm1*feedforwardm1+a1*feedforwardm1+a0;
    ffThrust2 = a2*feedforwardm2*feedforwardm2+a1*feedforwardm2+a0;
    ffThrust3 = a2*feedforwardm3*feedforwardm3+a1*feedforwardm3+a0;
    ffThrust4 = a2*feedforwardm4*feedforwardm4+a1*feedforwardm4+a0;
}

bool calculateControlOutput(Controller::Request &request, Controller::Response &response) {
    ROS_INFO("calculate control output");
    ffSetup(); //to change that it only does this once at intialization
    
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
        0,                       0,            0,             0,              0,              0,                0,             0,  2.843099534,
        0,                       0,   0.22195826,             0,              0,     0.12362477,                0,             0,            0
    };

    float px = request.crazyflieLocation.x - request.setpoint.x;
    float py = request.crazyflieLocation.y - request.setpoint.y;
    float pz = request.crazyflieLocation.z - request.setpoint.z;


    //linear approximation of derivative of position (no Estimator implemented...)
    float vx = (request.crazyflieLocation.x - previousLocation.x) / request.crazyflieLocation.acquiringTime;
    float vy = (request.crazyflieLocation.y - previousLocation.y) / request.crazyflieLocation.acquiringTime;
    float vz = (request.crazyflieLocation.z - previousLocation.z) / request.crazyflieLocation.acquiringTime;

    ROS_INFO_STREAM("px: " << px);
    ROS_INFO_STREAM("py: " << py);
    ROS_INFO_STREAM("pz: " << pz);
    ROS_INFO_STREAM("vx: " << vx);
    ROS_INFO_STREAM("vy: " << vy);
    ROS_INFO_STREAM("vz: " << vz);

    float roll = request.crazyflieLocation.roll;
    float pitch = request.crazyflieLocation.pitch;
    float yaw = request.crazyflieLocation.yaw - request.setpoint.yaw;

    const float PI = 3.141592535f;
    while(yaw > PI) yaw -= 2 * PI;
    while(yaw < -PI) yaw += 2 * PI;


    ROS_INFO_STREAM("yaw: " << yaw);

    //convert into body frame
    float sinYaw = sin(request.crazyflieLocation.yaw);
    float cosYaw = cos(request.crazyflieLocation.yaw);

    float state[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    state[0] = px * cosYaw + py * sinYaw;
    state[1] = -px * sinYaw + py * cosYaw;
    state[2] = pz;

    state[3] = vx * cosYaw + vy * sinYaw;
    state[4] = -vx * sinYaw + vy * cosYaw;
    state[5] = vz;

    state[6] = roll;
    state[7] = pitch;
    state[8] = yaw;

    //roll,pitch,yaw calculations
    float resRoll = 0;
    float resPitch = 0;
    float resYaw = 0;
    float thrustIntermediate = 0;
    for(int i = 0; i < 9; ++i) {
    	resRoll -= k[i + 0] * state[i];
    	resPitch -= k[i + 9] * state[i];
    	resYaw -= k[i + 18] * state[i];
    	thrustIntermediate -= k[i + 27] * state[i];
    }

    response.controlOutput.roll = resRoll;
    response.controlOutput.pitch = resPitch;
    response.controlOutput.yaw = resYaw;

    ROS_INFO_STREAM("thrustIntermediate before: " << thrustIntermediate);
    if(thrustIntermediate > saturationThrust)
        thrustIntermediate = saturationThrust;
    else if(thrustIntermediate < -saturationThrust)
        thrustIntermediate = -saturationThrust;

    response.controlOutput.motorCmd1 = (-a1+sqrt(a1*a1-4*a2*(a0-(thrustIntermediate+ffThrust1))))/(2*a2);
    response.controlOutput.motorCmd2 = (-a1+sqrt(a1*a1-4*a2*(a0-(thrustIntermediate+ffThrust2))))/(2*a2);
    response.controlOutput.motorCmd3 = (-a1+sqrt(a1*a1-4*a2*(a0-(thrustIntermediate+ffThrust3))))/(2*a2);
    response.controlOutput.motorCmd4 = (-a1+sqrt(a1*a1-4*a2*(a0-(thrustIntermediate+ffThrust4))))/(2*a2);
    previousCommand = response.controlOutput;

    response.controlOutput.onboardControllerType = 0; //RATE

    previousLocation = request.crazyflieLocation;
    
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
