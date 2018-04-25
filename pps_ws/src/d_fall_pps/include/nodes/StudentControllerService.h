//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
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
//    Place for students to implement their controller
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

//some useful libraries
#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <ros/package.h>

//the generated structs from the msg-files have to be included
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/Setpoint.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/Controller.h"
#include "d_fall_pps/DebugMsg.h"
#include "d_fall_pps/CustomControllerYAML.h"

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

// These constants define the modes that can be used for controller the Crazyflie 2.0,
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

// These constants define the controller used for computing the response in the
// "calculateControlOutput" function
// The following is a short description about each mode:
// LQR_RATE_MODE      LQR controller based on the state vector:
//                    [position,velocity,angles]
//
// LQR_ANGLE_MODE     LQR controller based on the state vector:
//                    [position,velocity]
//
#define LQR_RATE_MODE   1   // (DEFAULT)
#define LQR_ANGLE_MODE  2

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

// Variables for controller
float cf_mass;                       // Crazyflie mass in grams
std::vector<float> motorPoly(3);     // Coefficients of the 16-bit command to thrust conversion
float control_frequency;             // Frequency at which the controller is running
float gravity_force;                 // The weight of the Crazyflie in Newtons, i.e., mg

float previous_stateErrorInertial[9];     // The location error of the Crazyflie at the "previous" time step

std::vector<float>  setpoint{0.0,0.0,0.4,0.0};     // The setpoints for (x,y,z) position and yaw angle, in that order


// The LQR Controller parameters for "LQR_RATE_MODE"
const float gainMatrixRollRate[9]    =  { 0.00,-1.72, 0.00, 0.00,-1.34, 0.00, 5.12, 0.00, 0.00};
const float gainMatrixPitchRate[9]   =  { 1.72, 0.00, 0.00, 1.34, 0.00, 0.00, 0.00, 5.12, 0.00};
const float gainMatrixYawRate[9]     =  { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.84};
const float gainMatrixThrust[9]  =  { 0.00, 0.00, 0.25, 0.00, 0.00, 0.14, 0.00, 0.00, 0.00};


// ROS Publisher for debugging variables
ros::Publisher debugPublisher;


// Variable for the namespaces for the paramter services
// > For the paramter service of this agent
std::string namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string namespace_to_coordinator_parameter_service;

// The ID of this agent, i.e., the ID of this compute
int my_agentID = 0;


// RELEVANT NOTES ABOUT THE VARIABLES DECLARE HERE:
// The "CrazyflieData" type used for the "request" variable is a
// structure as defined in the file "CrazyflieData.msg" which has the following
// properties:
//     string crazyflieName              The name given to the Crazyflie in the Vicon software
//     float64 x                         The x position of the Crazyflie [metres]
//     float64 y                         The y position of the Crazyflie [metres]
//     float64 z                         The z position of the Crazyflie [metres]
//     float64 roll                      The roll component of the intrinsic Euler angles [radians]
//     float64 pitch                     The pitch component of the intrinsic Euler angles [radians]
//     float64 yaw                       The yaw component of the intrinsic Euler angles [radians]
//     float64 acquiringTime #delta t    The time elapsed since the previous "CrazyflieData" was received [seconds]
//     bool occluded                     A boolean indicted whether the Crazyflie for visible at the time of this measurement





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

// CONTROLLER COMPUTATIONS
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured);

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);

// SETPOINT CHANGE CALLBACK
void setpointCallback(const Setpoint& newSetpoint);

// LOAD PARAMETERS
float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name);
void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length);
int getParameterInt(ros::NodeHandle& nodeHandle, std::string name);
void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length);
int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val);
bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name);

void yamlReadyForFetchCallback(const std_msgs::Int32& msg);
void fetchYamlParameters(ros::NodeHandle& nodeHandle);
void processFetchedParameters();
//void customYAMLasMessageCallback(const CustomControllerYAML& newCustomControllerParameters);