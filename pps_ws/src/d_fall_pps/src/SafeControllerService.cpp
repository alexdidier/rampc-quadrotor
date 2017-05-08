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
#define VEL_AVERAGE_SIZE 10

using namespace d_fall_pps;

std::vector<float>  ffThrust(4);
std::vector<float>  feedforwardMotor(4);
std::vector<float>  motorPoly(3);

std::vector<float>  gainMatrixRoll(9);
std::vector<float>  gainMatrixPitch(9);
std::vector<float>  gainMatrixYaw(9);
std::vector<float>  gainMatrixThrust(9);

//K_infinite of feedback
std::vector<float> filterGain(6);
//only for velocity calculation
std::vector<float> estimatorMatrix(2);
float prevEstimate[9];

std::vector<float>  setpoint(4);
float saturationThrust;

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

    loadParameterFloatVector(nodeHandle, "filterGain", filterGain, 6);
    loadParameterFloatVector(nodeHandle, "estimatorMatrix", estimatorMatrix, 2);

    loadParameterFloatVector(nodeHandle, "defaultSetpoint", setpoint, 4);
}

float computeMotorPolyBackward(float thrust) {
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
}


//Kalman
void estimateState(Controller::Request &request, float (&est)[9]) {
    // attitude
    est[6] = request.crazyflieLocation.roll;
    est[7] = request.crazyflieLocation.pitch;
    est[8] = request.crazyflieLocation.yaw;

    //velocity & filtering
    float ahat_x[6]; //estimator matrix times state (x, y, z, vx, vy, vz)
    ahat_x[0] = 0; ahat_x[1]=0; ahat_x[2]=0;
    ahat_x[3] = estimatorMatrix[0] * prevEstimate[0] + estimatorMatrix[1] * prevEstimate[3];
    ahat_x[4] = estimatorMatrix[0] * prevEstimate[1] + estimatorMatrix[1] * prevEstimate[4];
    ahat_x[5] = estimatorMatrix[0] * prevEstimate[2] + estimatorMatrix[1] * prevEstimate[5];

    
    ROS_INFO_STREAM("est prevEstimate[0]: " << prevEstimate[0]);
    ROS_INFO_STREAM("est prevEstimate[3]: " << prevEstimate[3]);
    ROS_INFO_STREAM("est prevEstimate[1]: " << prevEstimate[1]);
    ROS_INFO_STREAM("est prevEstimate[4]: " << prevEstimate[4]);
    ROS_INFO_STREAM("est prevEstimate[2]: " << prevEstimate[2]);
    ROS_INFO_STREAM("est prevEstimate[5]: " << prevEstimate[5]);

    ROS_INFO_STREAM("est request.crazyflieLocation.x: " << request.crazyflieLocation.x);
    ROS_INFO_STREAM("est request.crazyflieLocation.y: " << request.crazyflieLocation.y);
    ROS_INFO_STREAM("est request.crazyflieLocation.z: " << request.crazyflieLocation.z);
    

    float k_x[6]; //filterGain times state
    k_x[0] = request.crazyflieLocation.x * filterGain[0];
    k_x[1] = request.crazyflieLocation.y * filterGain[1];
    k_x[2] = request.crazyflieLocation.z * filterGain[2];
    k_x[3] = request.crazyflieLocation.x * filterGain[3];
    k_x[4] = request.crazyflieLocation.y * filterGain[4];
    k_x[5] = request.crazyflieLocation.z * filterGain[5];

    
    ROS_INFO_STREAM("est k_x x: " << k_x[0]);
    ROS_INFO_STREAM("est k_x y: " << k_x[1]);
    ROS_INFO_STREAM("est k_x z: " << k_x[2]);
    ROS_INFO_STREAM("est k_x vx: " << k_x[3]);
    ROS_INFO_STREAM("est k_x vy: " << k_x[4]);
    ROS_INFO_STREAM("est k_x vz: " << k_x[5]);

    ROS_INFO_STREAM("est ahat_x x: " << ahat_x[0]);
    ROS_INFO_STREAM("est ahat_x y: " << ahat_x[1]);
    ROS_INFO_STREAM("est ahat_x z: " << ahat_x[2]);
    ROS_INFO_STREAM("est ahat_x vx: " << ahat_x[3]);
    ROS_INFO_STREAM("est ahat_x vy: " << ahat_x[4]);
    ROS_INFO_STREAM("est ahat_x vz: " << ahat_x[5]);
    

    est[0] = ahat_x[0] + k_x[0];
    est[1] = ahat_x[1] + k_x[1];
    est[2] = ahat_x[2] + k_x[2];
    est[3] = ahat_x[3] + k_x[3];
    est[4] = ahat_x[4] + k_x[4];
    est[5] = ahat_x[5] + k_x[5];

    memcpy(prevEstimate, est, 9 * sizeof(float));

    
    ROS_INFO_STREAM("est x: " << est[0]);
    ROS_INFO_STREAM("est y: " << est[1]);
    ROS_INFO_STREAM("est z: " << est[2]);

    ROS_INFO_STREAM("est vx: " << est[3]);
    ROS_INFO_STREAM("est vy: " << est[4]);
    ROS_INFO_STREAM("est vz: " << est[5]);

    ROS_INFO_STREAM("est y: " << est[8]);
    ROS_INFO_STREAM("est r: " << est[6]);
    ROS_INFO_STREAM("est p: " << est[7]);
    
}


//simple derivative
/*
void estimateState(Controller::Request &request, float (&est)[9]) {
    est[0] = request.crazyflieLocation.x;
    est[1] = request.crazyflieLocation.y;
    est[2] = request.crazyflieLocation.z;

    est[3] = (request.crazyflieLocation.x - previousLocation.x) / request.crazyflieLocation.acquiringTime;
    est[4] = (request.crazyflieLocation.y - previousLocation.y) / request.crazyflieLocation.acquiringTime;
    est[5] = (request.crazyflieLocation.z - previousLocation.z) / request.crazyflieLocation.acquiringTime;

    ROS_INFO_STREAM("vx: " << est[3]);
    ROS_INFO_STREAM("vy: " << est[4]);
    ROS_INFO_STREAM("vz: " << est[5]);

    est[6] = request.crazyflieLocation.roll;
    est[7] = request.crazyflieLocation.pitch;
    est[8] = request.crazyflieLocation.yaw;
}
*/

void convertIntoBodyFrame(Controller::Request &request, float est[9], float (&state)[9], int yaw_measured) {
    //float sinYaw = sin(request.crazyflieLocation.yaw);
    //float cosYaw = cos(request.crazyflieLocation.yaw);
	float sinYaw = sin(yaw_measured);
    float cosYaw = cos(yaw_measured);

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
	
	//trial>>>>>>>
	int yaw_measured = request.crazyflieLocation.yaw;
	//<<<<<<

    //move coordinate system to make setpoint origin
    request.crazyflieLocation.x -= setpoint[0];
    request.crazyflieLocation.y -= setpoint[1];
    request.crazyflieLocation.z -= setpoint[2];
    float yaw = request.crazyflieLocation.yaw - setpoint[3];

    while(yaw > PI) yaw -= 2 * PI;
    while(yaw < -PI) yaw += 2 * PI;
    request.crazyflieLocation.yaw = yaw;

    float est[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    estimateState(request, est);

    float state[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    convertIntoBodyFrame(request, est, state, yaw_measured);
	//convertIntoBodyFrame(request, est, state, yaw);

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
    //outYaw *= 0.5;

    response.controlOutput.roll = outRoll;
    response.controlOutput.pitch = outPitch;
    response.controlOutput.yaw = outYaw;

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

void setpointCallback(const Setpoint& newSetpoint) {
    setpoint[0] = newSetpoint.x;
    setpoint[1] = newSetpoint.y;
    setpoint[2] = newSetpoint.z;
    setpoint[3] = newSetpoint.yaw;
}


//ros::Publisher pub;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SafeControllerService");

    ros::NodeHandle nodeHandle("~");
    loadParameters(nodeHandle);

    ros::Publisher setpointPublisher = nodeHandle.advertise<Setpoint>("Setpoint", 1);
    ros::Subscriber setpointSubscriber = nodeHandle.subscribe("/SafeControllerService/Setpoint", 1, setpointCallback);

    ros::ServiceServer service = nodeHandle.advertiseService("RateController", calculateControlOutput);
    ROS_INFO("SafeControllerService ready");
    ros::spin();

    return 0;
}
