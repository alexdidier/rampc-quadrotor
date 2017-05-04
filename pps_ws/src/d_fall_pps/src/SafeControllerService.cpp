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
#include <stdlib.h>
#include "ros/ros.h"
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/Setpoint.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/Controller.h"

#define PI 3.1415926535
#define RATE_CONTROLLER 0
#define RAD2DEG 57.3
#define VEL_AVERAGE_SIZE 10

using namespace d_fall_pps;

std::vector<float>  ffThrust(4);
std::vector<float>  feedforwardMotor(4);
std::vector<float>  motorPoly(3);

std::vector<float>  gainMatrixRoll(9);
std::vector<float>  gainMatrixPitch(9);
std::vector<float>  gainMatrixYaw(9);
std::vector<float>  gainMatrixThrust(9);

std::vector<float>  setpoint(4);
float saturationThrust;

//averaging test, will probably be replaced by Kalman filter
//float prevVelocities[VEL_AVERAGE_SIZE * 3];
//int velocityIndex = 0;

ViconData previousLocation;

void loadParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length) {
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

void loadParameters(ros::NodeHandle& nodeHandle) {
    loadParameterFloatVector(nodeHandle, "feedforwardMotor", feedforwardMotor, 4);
    loadParameterFloatVector(nodeHandle, "motorPoly", motorPoly, 3);

    for(int i = 0; i < 4; ++i) {
        ffThrust[i] = motorPoly[2] * feedforwardMotor[i] * feedforwardMotor[i] + motorPoly[1] * feedforwardMotor[i] + motorPoly[0];
    }
    saturationThrust = motorPoly[2] * 12000 * 12000 + motorPoly[1] * 12000 + motorPoly[0];

    loadParameterFloatVector(nodeHandle, "gainMatrixRoll", gainMatrixRoll, 9);
    loadParameterFloatVector(nodeHandle, "gainMatrixPitch", gainMatrixPitch, 9);
    loadParameterFloatVector(nodeHandle, "gainMatrixYaw", gainMatrixYaw, 9);
    loadParameterFloatVector(nodeHandle, "gainMatrixThrust", gainMatrixThrust, 9);
    loadParameterFloatVector(nodeHandle, "setpoint", setpoint, 4);
}

float computeMotorPolyBackward(float thrust) {
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
}

void estimateState(Controller::Request &request, float (&est)[9]) {
    est[0] = request.crazyflieLocation.x - setpoint[0];
    est[1] = request.crazyflieLocation.y - setpoint[1];
    est[2] = request.crazyflieLocation.z - setpoint[2];

    //linear approximation of derivative of position (no Estimator implemented...)
    /*prevVelocities[3 * velocityIndex + 0] = (request.crazyflieLocation.x - previousLocation.x) / request.crazyflieLocation.acquiringTime;
    prevVelocities[3 * velocityIndex + 1] = (request.crazyflieLocation.y - previousLocation.y) / request.crazyflieLocation.acquiringTime;
    prevVelocities[3 * velocityIndex + 2] = (request.crazyflieLocation.z - previousLocation.z) / request.crazyflieLocation.acquiringTime;
    velocityIndex = (velocityIndex + 1) % VEL_AVERAGE_SIZE;

    est[3] = 0;
    est[4] = 0;
    est[5] = 0;
    for(int i = 0; i < VEL_AVERAGE_SIZE; ++i) {
        est[3] += prevVelocities[3 * i + 0] / ((float) VEL_AVERAGE_SIZE);
        est[4] += prevVelocities[3 * i + 1] / ((float) VEL_AVERAGE_SIZE);
        est[5] += prevVelocities[3 * i + 2] / ((float) VEL_AVERAGE_SIZE);
    }*/

    est[3] = (request.crazyflieLocation.x - previousLocation.x) / request.crazyflieLocation.acquiringTime;
    est[4] = (request.crazyflieLocation.y - previousLocation.y) / request.crazyflieLocation.acquiringTime;
    est[5] = (request.crazyflieLocation.z - previousLocation.z) / request.crazyflieLocation.acquiringTime;

    //ROS_INFO_STREAM("velocityIndex: " << velocityIndex);
    ROS_INFO_STREAM("vx: " << est[3]);
    ROS_INFO_STREAM("vy: " << est[4]);
    ROS_INFO_STREAM("vz: " << est[5]);

    est[6] = request.crazyflieLocation.roll;
    est[7] = request.crazyflieLocation.pitch;
    //ROS_INFO_STREAM("crazyflieyaw: " << request.crazyflieLocation.yaw);
    //ROS_INFO_STREAM("setpointyaw: " << request.setpoint.yaw);
    float yaw = request.crazyflieLocation.yaw - setpoint[3];
    //ROS_INFO_STREAM("differenceyaw: " << yaw);

    while(yaw > PI) yaw -= 2 * PI;
    while(yaw < -PI) yaw += 2 * PI;
    est[8] = yaw;
}

void convertIntoBodyFrame(Controller::Request &request, float est[9], float (&state)[9]) {
    float sinYaw = sin(request.crazyflieLocation.yaw);
    float cosYaw = cos(request.crazyflieLocation.yaw);

    state[0] = est[0] * cosYaw + est[1] * sinYaw;
    state[1] = -est[0] * sinYaw + est[1] * cosYaw;
    state[2] = est[2];

    state[3] = est[3] * cosYaw + est[4] * sinYaw;
    state[4] = -est[3] * sinYaw + est[4] * cosYaw;
    state[5] = est[5];

    state[6] = est[6];
    state[7] = est[7];
    state[8] = est[8];
}

bool calculateControlOutput(Controller::Request &request, Controller::Response &response) {
    ViconData vicon = request.crazyflieLocation;

    float est[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    estimateState(request, est);

    float state[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    convertIntoBodyFrame(request, est, state);

    //calculate feedback
    float outRoll = 0;
    float outPitch = 0;
    float outYaw = 0;
    float thrustIntermediate = 0;
    for(int i = 0; i < 9; ++i) {
    	outRoll -= gainMatrixRoll[i] * state[i];
    	outPitch -= gainMatrixPitch[i] * state[i];
    	outYaw -= gainMatrixYaw[i] * state[i];
    	thrustIntermediate -= gainMatrixThrust[i] * state[i];
    }

    //HINWEIS: übersteuern beim outYaw wenn man 180 Grad zum yaw-Setpoint startet
    //nach Multiplikation mit 0.5 gibt es den Effekt nicht mehr -> mit Paul besprechen....
    outYaw = outYaw * 0.5;

    //multiply by RAD2DEG as OnBoard Controller expects degrees
    response.controlOutput.roll = outRoll*RAD2DEG;
    response.controlOutput.pitch = outPitch* RAD2DEG;
    response.controlOutput.yaw = outYaw * RAD2DEG;

    if(thrustIntermediate > saturationThrust)
        thrustIntermediate = saturationThrust;
    else if(thrustIntermediate < -saturationThrust)
        thrustIntermediate = -saturationThrust;

    response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustIntermediate + ffThrust[0]);
    response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustIntermediate + ffThrust[1]);
    response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustIntermediate + ffThrust[2]);
    response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustIntermediate + ffThrust[3]);

    response.controlOutput.onboardControllerType = RATE_CONTROLLER;

    previousLocation = request.crazyflieLocation;
    
	return true;
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SafeControllerService");

    ros::NodeHandle nodeHandle("~");
    loadParameters(nodeHandle);

    ros::ServiceServer service = nodeHandle.advertiseService("RateController", calculateControlOutput);
    ROS_INFO("SafeControllerService ready");
    ros::spin();

    return 0;
}
