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
#include <std_msgs/String.h>
#include <ros/package.h>
#include "std_msgs/Float32.h"

#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/Setpoint.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/Controller.h"

#define PI 3.1415926535
#define RATE_CONTROLLER 0

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

float saturationThrust;

CrazyflieData previousLocation;

//circle stuff
float currentTime;
const float OMEGA = 0.25*2*PI;
const float RADIUS = 0.25;

//point publisher for FollowCrazyflieService
ros::Publisher followPublisher;


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

}

float computeMotorPolyBackward(float thrust) {
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
}


//Kalman
void estimateState(Controller::Request &request, float (&est)[9]) {
    // attitude
    est[6] = request.ownCrazyflie.roll;
    est[7] = request.ownCrazyflie.pitch;
    est[8] = request.ownCrazyflie.yaw;

    //velocity & filtering
    float ahat_x[6]; //estimator matrix times state (x, y, z, vx, vy, vz)
    ahat_x[0] = 0; ahat_x[1]=0; ahat_x[2]=0;
    ahat_x[3] = estimatorMatrix[0] * prevEstimate[0] + estimatorMatrix[1] * prevEstimate[3];
    ahat_x[4] = estimatorMatrix[0] * prevEstimate[1] + estimatorMatrix[1] * prevEstimate[4];
    ahat_x[5] = estimatorMatrix[0] * prevEstimate[2] + estimatorMatrix[1] * prevEstimate[5];

    
    float k_x[6]; //filterGain times state
    k_x[0] = request.ownCrazyflie.x * filterGain[0];
    k_x[1] = request.ownCrazyflie.y * filterGain[1];
    k_x[2] = request.ownCrazyflie.z * filterGain[2];
    k_x[3] = request.ownCrazyflie.x * filterGain[3];
    k_x[4] = request.ownCrazyflie.y * filterGain[4];
    k_x[5] = request.ownCrazyflie.z * filterGain[5];
   
    est[0] = ahat_x[0] + k_x[0];
    est[1] = ahat_x[1] + k_x[1];
	est[2] = ahat_x[2] + k_x[2];
    est[3] = ahat_x[3] + k_x[3];
    est[4] = ahat_x[4] + k_x[4];
    est[5] = ahat_x[5] + k_x[5];

    memcpy(prevEstimate, est, 9 * sizeof(float));
    
}

void convertIntoBodyFrame(float est[9], float (&state)[9], float yaw_measured) {
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

void calculateCircle(Setpoint &circlePoint){
    circlePoint.x = 0;
    circlePoint.y = RADIUS*sin(OMEGA*currentTime);
    circlePoint.z = RADIUS*cos(OMEGA*currentTime);
    circlePoint.yaw = 0;//OMEGA*currentTime;

}

bool calculateControlOutput(Controller::Request &request, Controller::Response &response) {
    CrazyflieData vicon = request.ownCrazyflie;
	
    currentTime += request.ownCrazyflie.acquiringTime;

    //publish setpoint for FollowController of another student group
    Setpoint pubSetpoint;
    pubSetpoint.x = request.ownCrazyflie.x;
    pubSetpoint.y = request.ownCrazyflie.y;
    pubSetpoint.z = request.ownCrazyflie.z;
    pubSetpoint.yaw = request.ownCrazyflie.yaw;
    followPublisher.publish(pubSetpoint);

    //actual Circle Controller stuff follows
	Setpoint circlePoint;
    calculateCircle(circlePoint);

	float yaw_measured = request.ownCrazyflie.yaw;

    //move coordinate system to make setpoint origin
    request.ownCrazyflie.x -= circlePoint.x;
    request.ownCrazyflie.y -= circlePoint.y;
    request.ownCrazyflie.z -= circlePoint.z;
    float yaw = request.ownCrazyflie.yaw - circlePoint.yaw;
	
    while(yaw > PI) {yaw -= 2 * PI;}
    while(yaw < -PI) {yaw += 2 * PI;}
    request.ownCrazyflie.yaw = yaw;

    float est[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    estimateState(request, est);
	
    float state[9]; //px, py, pz, vx, vy, vz, roll, pitch, yaw
    convertIntoBodyFrame(est, state, yaw_measured);

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

    previousLocation = request.ownCrazyflie;


    
	return true;
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "CircleControllerService");

    currentTime = 0;

    ros::NodeHandle nodeHandle("~");
    loadParameters(nodeHandle);

    //publisher for Follow Controller of another student group
    followPublisher = nodeHandle.advertise<Setpoint>("FollowTopic", 1);

    ros::ServiceServer service = nodeHandle.advertiseService("CircleController", calculateControlOutput);
    ROS_INFO("CircleControllerService ready");
    
    ros::spin();

    return 0;
}
