//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Cyrill Burgener, Marco Mueller, Philipp Friedli
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    The fall-back controller that keeps the Crazyflie safe
//
//    ----------------------------------------------------------------------------------




//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These various headers need to be included so that this controller service can be
// connected with the D-FaLL system.

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

#include <std_msgs/Int32.h>




//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.

// Universal constants
#define PI 3.1415926535

// The constants define the modes that can be used for controller the Crazyflie 2.0,
// the constants defined here need to be in agreement with those defined in the
// firmware running on the Crazyflie 2.0.
// The following is a short description about each mode:
// MOTOR_MODE    In this mode the Crazyflie will apply the requested 16-bit per motor
//               command directly to each of the motors
// RATE_MODE     In this mode the Crazyflie will apply the requested 16-bit per motor
//               command directly to each of the motors, and additionally request the
//               body frame roll, pitch, and yaw angular rates from the PID rate
//               controllers implemented in the Crazyflie 2.0 firmware.
// ANGE_MODE     In this mode the Crazyflie will apply the requested 16-bit per motor
//               command directly to each of the motors, and additionally request the
//               body frame roll, pitch, and yaw angles from the PID attitude
//               controllers implemented in the Crazyflie 2.0 firmware.
#define MOTOR_MODE 6
#define RATE_MODE  7
#define ANGLE_MODE 8

// Constants for feching the yaml paramters
#define FETCH_YAML_SAFE_CONTROLLER_AGENT          1
#define FETCH_YAML_CUSTOM_CONTROLLER_AGENT        2
#define FETCH_YAML_SAFE_CONTROLLER_COORDINATOR    3
#define FETCH_YAML_CUSTOM_CONTROLLER_COORDINATOR  4

// Namespacing the package
using namespace d_fall_pps;




//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

std::vector<float>  ffThrust(4);
std::vector<float>  feedforwardMotor(4);
float cf_mass;
float gravity_force;
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
std::vector<float> defaultSetpoint(4);
float saturationThrust;

CrazyflieData previousLocation;


// Variable for the namespaces for the paramter services
// > For the paramter service of this agent
std::string namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string namespace_to_coordinator_parameter_service;



//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    PPPP   RRRR    OOO   TTTTT   OOO   TTTTT  Y   Y  PPPP   EEEEE   SSSS
//    P   P  R   R  O   O    T    O   O    T     Y Y   P   P  E      S
//    PPPP   RRRR   O   O    T    O   O    T      Y    PPPP   EEE     SSS
//    P      R  R   O   O    T    O   O    T      Y    P      E          S
//    P      R   R   OOO     T     OOO     T      Y    P      EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These function prototypes are not strictly required for this code to complile, but
// adding the function prototypes here means the the functions can be written below in
// any order. If the function prototypes are not included then the function need to
// written below in an order that ensure each function is implemented before it is
// called from another function, hence why the "main" function is at the bottom.

// > For the CONTROL LOOP
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);
void convertIntoBodyFrame(float est[9], float (&state)[9], float yaw_measured);
float computeMotorPolyBackward(float thrust);
void estimateState(Controller::Request &request, float (&est)[9]);

// > For the LOAD PARAMETERS
void yamlReadyForFetchCallback(const std_msgs::Int32& msg);
void fetchYamlParameters(ros::NodeHandle& nodeHandle);
void processFetchedParameters();

// > For the GETPARAM()
float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name);
void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length);
int getParameterInt(ros::NodeHandle& nodeHandle, std::string name);
void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length);
int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val);
bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name);



//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    III M   M PPPP  L     EEEEE M   M EEEEE N   N TTTTT   A   TTTTT III  OOO  N   N
//     I  MM MM P   P L     E     MM MM E     NN  N   T    A A    T    I  O   O NN  N
//     I  M M M PPPP  L     EEE   M M M EEE   N N N   T   A   A   T    I  O   O N N N
//     I  M   M P     L     E     M   M E     N  NN   T   AAAAA   T    I  O   O N  NN
//    III M   M P     LLLLL EEEEE M   M EEEEE N   N   T   A   A   T   III  OOO  N   N
//    ----------------------------------------------------------------------------------




//    ------------------------------------------------------------------------------
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L           L       OOO    OOO   PPPP
//    C      O   O  NN  N    T    R   R  O   O  L           L      O   O  O   O  P   P
//    C      O   O  N N N    T    RRRR   O   O  L           L      O   O  O   O  PPPP
//    C      O   O  N  NN    T    R  R   O   O  L           L      O   O  O   O  P
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL       LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------

//simple derivative
/*
void estimateState(Controller::Request &request, float (&est)[9]) {
    est[0] = request.ownCrazyflie.x;
    est[1] = request.ownCrazyflie.y;
    est[2] = request.ownCrazyflie.z;

    est[3] = (request.ownCrazyflie.x - previousLocation.x) / request.ownCrazyflie.acquiringTime;
    est[4] = (request.ownCrazyflie.y - previousLocation.y) / request.ownCrazyflie.acquiringTime;
    est[5] = (request.ownCrazyflie.z - previousLocation.z) / request.ownCrazyflie.acquiringTime;

    est[6] = request.ownCrazyflie.roll;
    est[7] = request.ownCrazyflie.pitch;
    est[8] = request.ownCrazyflie.yaw;
}
*/


bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{
    
    float yaw_measured = request.ownCrazyflie.yaw;

    //move coordinate system to make setpoint origin
    request.ownCrazyflie.x -= setpoint[0];
    request.ownCrazyflie.y -= setpoint[1];
    request.ownCrazyflie.z -= setpoint[2];
    float yaw = request.ownCrazyflie.yaw - setpoint[3];

    
    //bag.write("Offset", ros::Time::now(), request.ownCrazyflie);

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
    // ROS_INFO_STREAM("thrustIntermediate = " << thrustIntermediate);
    //INFORMATION: this ugly fix was needed for the older firmware
    //outYaw *= 0.5;

    response.controlOutput.roll = outRoll;
    response.controlOutput.pitch = outPitch;
    response.controlOutput.yaw = outYaw;

    if(thrustIntermediate > saturationThrust)
        thrustIntermediate = saturationThrust;
    else if(thrustIntermediate < -saturationThrust)
        thrustIntermediate = -saturationThrust;

    response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustIntermediate + gravity_force);

    // ROS_INFO_STREAM("ffThrust" << ffThrust[0]);
    // ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
    // ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
    // ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
    // ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);

    response.controlOutput.onboardControllerType = RATE_MODE;

    previousLocation = request.ownCrazyflie;

    return true;
}

void convertIntoBodyFrame(float est[9], float (&state)[9], float yaw_measured)
{
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



float computeMotorPolyBackward(float thrust)
{
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
}


//Kalman
void estimateState(Controller::Request &request, float (&est)[9])
{
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





//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//    PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
//    P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
//    PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
//    P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
//    P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
//    ----------------------------------------------------------------------------------


// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void yamlReadyForFetchCallback(const std_msgs::Int32& msg)
{
    // Extract from the "msg" for which controller the and from where to fetch the YAML
    // parameters
    int controller_to_fetch_yaml = msg.data;

    // Switch between fetching for the different controllers and from different locations
    switch(controller_to_fetch_yaml)
    {

        // > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
        case FETCH_YAML_SAFE_CONTROLLER_AGENT:
        {
            // Let the user know that this message was received
            ROS_INFO("The SafeControllerService received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from this machine.");
            // Create a node handle to the parameter service running on this agent's machine
            ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);
            // Call the function that fetches the parameters
            fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);
            break;
        }

        // > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
        case FETCH_YAML_SAFE_CONTROLLER_COORDINATOR:
        {
            // Let the user know that this message was received
            ROS_INFO("The SafeControllerService received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from the coordinator.");
            // Create a node handle to the parameter service running on the coordinator machine
            ros::NodeHandle nodeHandle_to_coordinator_parameter_service(namespace_to_coordinator_parameter_service);
            // Call the function that fetches the parameters
            fetchYamlParameters(nodeHandle_to_coordinator_parameter_service);
            break;
        }

        default:
        {
            // Let the user know that the command was not relevant
            //ROS_INFO("The SafeControllerService received the message that YAML parameters were (re-)loaded.\r> However the parameters do not relate to this controller, hence nothing will be fetched.");
            break;
        }
    }
}

void fetchYamlParameters(ros::NodeHandle& nodeHandle)
{

    // Here we load the parameters that are specified in the SafeController.yaml file

    // Add the "CustomController" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_safeController(nodeHandle, "SafeController");

    // > The mass of the crazyflie
    cf_mass = getParameterFloat(nodeHandle_for_safeController, "mass");

    // > The co-efficients of the quadratic conversation from 16-bit motor command to
    //   thrust force in Newtons
    getParameterFloatVector(nodeHandle_for_safeController, "motorPoly", motorPoly, 3);
    
    
    // > The row of the LQR matrix that commands body frame roll rate
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixRoll", gainMatrixRoll, 9);
    // > The row of the LQR matrix that commands body frame pitch rate
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixPitch", gainMatrixPitch, 9);
    // > The row of the LQR matrix that commands body frame yaw rate
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixYaw", gainMatrixYaw, 9);
    // > The row of the LQR matrix that commands thrust adjustment
    getParameterFloatVector(nodeHandle_for_safeController, "gainMatrixThrust", gainMatrixThrust, 9);

    // > The gains for the point-mass Kalman filter
    getParameterFloatVector(nodeHandle_for_safeController, "filterGain", filterGain, 6);
    getParameterFloatVector(nodeHandle_for_safeController, "estimatorMatrix", estimatorMatrix, 2);

    // > The defailt setpoint of the controller
    getParameterFloatVector(nodeHandle_for_safeController, "defaultSetpoint", defaultSetpoint, 4);

    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("DEBUGGING: the fetched SafeController/mass = " << cf_mass);

    // Call the function that computes details an values that are needed from these
    // parameters loaded above
    processFetchedParameters();
}

void processFetchedParameters()
{
    // force that we need to counteract gravity (mg)
    gravity_force = cf_mass * 9.81/(1000*4); // in N
    // The maximum possible thrust
    saturationThrust = motorPoly[2] * 12000 * 12000 + motorPoly[1] * 12000 + motorPoly[0];
}




//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------


float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}

void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}

void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val.size();
}

bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name)
{
    bool val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     GGG   U   U  III        CCCC    A    L      L      BBBB     A     CCCC  K   K
//    G   G  U   U   I        C       A A   L      L      B   B   A A   C      K  K
//    G      U   U   I        C      A   A  L      L      BBBB   A   A  C      KKK
//    G   G  U   U   I        C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     GGGG   UUU   III        CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------

void setpointCallback(const Setpoint& newSetpoint) {
    setpoint[0] = newSetpoint.x;
    setpoint[1] = newSetpoint.y;
    setpoint[2] = newSetpoint.z;
    setpoint[3] = newSetpoint.yaw;
}





//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "SafeControllerService");

    ros::NodeHandle nodeHandle("~");


    // *********************************************************************************
    // EVERYTHING THAT RELATES TO FETCHING PARAMETERS FROM A YAML FILE


    // EVERYTHING FOR THE CONNECTION TO THIS AGENT's OWN PARAMETER SERVICE:

    // Get the namespace of this "SafeControllerService" node
    std::string m_namespace = ros::this_node::getNamespace();

    // Set the class variable "namespace_to_own_agent_parameter_service" to be a the
    // namespace string for the parameter service that is running on the machine of this
    // agent
    namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";
    
    // Create a node handle to the parameter service running on this agent's machine
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    ros::Subscriber controllerYamlReadyForFetchSubscriber_to_agent = nodeHandle_to_own_agent_parameter_service.subscribe("controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);


    // EVERYTHING FOR THE CONNECTION THE COORDINATOR'S PARAMETER SERVICE:

    // Set the class variable "nodeHandle_to_coordinator_parameter_service" to be a node handle
    // for the parameter service that is running on the coordinate machine
    // NOTE: the backslash here (i.e., "/") before the name of the node ("ParameterService")
    //       is very important because it specifies that the name is global
    namespace_to_coordinator_parameter_service = "/ParameterService";

    // Create a node handle to the parameter service running on the coordinator machine
    ros::NodeHandle nodeHandle_to_coordinator = ros::NodeHandle();
    //ros::NodeHandle nodeHandle_to_coordinator_parameter_service = ros::NodeHandle(namespace_to_own_agent_parameter_service);
    

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator.subscribe("/ParameterService/controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);
    //ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator_parameter_service.subscribe("controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);


    // PRINT OUT SOME INFORMATION

    // Let the user know what namespaces are being used for linking to the parameter service
    ROS_INFO_STREAM("The namespace string for accessing the Paramter Services are:");
    ROS_INFO_STREAM("namespace_to_own_agent_parameter_service    =  " << namespace_to_own_agent_parameter_service);
    ROS_INFO_STREAM("namespace_to_coordinator_parameter_service  =  " << namespace_to_coordinator_parameter_service);


    // FINALLY, FETCH ANY PARAMETERS REQUIRED FROM THESE "PARAMETER SERVICES"

    // Call the class function that loads the parameters for this class.
    fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);

    // *********************************************************************************

    
    setpoint = defaultSetpoint; // only first time setpoint is equal to default setpoint

    ros::Subscriber setpointSubscriber = nodeHandle.subscribe("Setpoint", 1, setpointCallback);

    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());


    ros::ServiceServer service = nodeHandle.advertiseService("RateController", calculateControlOutput);
    ROS_INFO("SafeControllerService ready");
    
	// std::string package_path;
	// package_path = ros::package::getPath("d_fall_pps") + "/";
	// ROS_INFO_STREAM(package_path);
	// std::string record_file = package_path + "LoggingSafeController.bag";
	// bag.open(record_file, rosbag::bagmode::Write);


    ros::spin();
	// bag.close();
	
	

    return 0;
}
