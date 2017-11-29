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

// Variables for controller
float cf_mass;                       // Crazyflie mass in grams
std::vector<float> motorPoly(3);     // Coefficients of the 16-bit command to thrust conversion
float control_frequency;             // Frequency at which the controller is running
float gravity_force;                 // The weight of the Crazyflie in Newtons, i.e., mg

CrazyflieData previous_stateErrorInertial;     // The location error of the Crazyflie at the "previous" time step

std::vector<float>  setpoint{0.0,0.0,0.4,0.0};     // The setpoints for (x,y,z) position and yaw angle, in that order

// The LQR Controller parameters
const float gainMatrixRoll[9] = {0, -1.714330725, 0, 0, -1.337107465, 0, 5.115369735, 0, 0};
const float gainMatrixPitch[9] = {1.714330725, 0, 0, 1.337107465, 0, 0, 0, 5.115369735, 0};
const float gainMatrixYaw[9] = {0, 0, 0, 0, 0, 0, 0, 0, 2.843099534};
const float gainMatrixThrust[9] = {0, 0, 0.22195826, 0, 0, 0.12362477, 0, 0, 0};

// ROS Publisher for debugging variables
ros::Publisher debugPublisher;


// Variable for the node handle to the paramter services
// > For the paramter service of this agent
ros::NodeHandle nodeHandle_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
ros::NodeHandle nodeHandle_to_coordinator_parameter_service;


// Boolean whether to execute the convert into body frame function
bool shouldPerformConvertIntoBodyFrame = false;


// VARIABLES RELATING TO PUBLISHING CURRENT POSITION AND FOLLOWING ANOTHER AGENT'S
// POSITION

// The ID of this agent, i.e., the ID of this compute
int my_agentID = 0;

// Boolean indicating whether the (x,y,z,yaw) of this agent should be published or not
// > The default behaviour is: do not publish,
// > This varaible is changed based on parameters loaded from the YAML file
bool shouldPublishCurrent_xyz_yaw = false;

// Boolean indicating whether the (x,y,z,yaw) setpoint of this agent should adapted in
// an attempt to follow the "my_current_xyz_yaw_topic" from another agent
// > The default behaviour is: do not follow,
// > This varaible is changed based on parameters loaded from the YAML file
bool shouldFollowAnotherAgent = false;

// The order in which agents should follow eachother
// > This parameter is a vector of integers that specifies  agent ID's
// > The order of the agent ID's is the ordering of the line formation
// > i.e., the first ID is the leader, the second ID should follow the first ID, and so on
std::vector<int> follow_in_a_line_agentIDs(100);

// Integer specifying the ID of the agent that will be followed by this agent
// > The default behaviour not to follow any agent, indicated by ID zero
// > This varaible is changed based on parameters loaded from the YAML file
int agentID_to_follow = 0;

// ROS Publisher for my current (x,y,z,yaw) position
ros::Publisher my_current_xyz_yaw_publisher;

// ROS Subscriber for my position
ros::Subscriber xyz_yaw_to_follow_subscriber;


// RELEVANT NOTES ABOUT THE VARIABLES DECLARE HERE:
// The "CrazyflieData" type used for the "previous_stateErrorInertial" variable is a
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

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured);

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);

// SETPOINT CHANGE CALLBACK
void setpointCallback(const Setpoint& newSetpoint);
void xyz_yaw_to_follow_callback(const Setpoint& newSetpoint);

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
//     OOO   U   U  TTTTT  EEEEE  RRRR 
//    O   O  U   U    T    E      R   R
//    O   O  U   U    T    EEE    RRRR
//    O   O  U   U    T    E      R  R
//     OOO    UUU     T    EEEEE  R   R
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L           L       OOO    OOO   PPPP
//    C      O   O  NN  N    T    R   R  O   O  L           L      O   O  O   O  P   P
//    C      O   O  N N N    T    RRRR   O   O  L           L      O   O  O   O  PPPP
//    C      O   O  N  NN    T    R  R   O   O  L           L      O   O  O   O  P
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL       LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------

// This function is the callback that is linked to the "CustomController" service that
// is advertised in the main function. This must have arguments that match the
// "input-output" behaviour defined in the "Controller.srv" file (located in the "srv"
// folder)
//
// The arument "request" is a structure provided to this service with the following two
// properties:
// request.ownCrazyflie
// This property is itself a structure of type "CrazyflieData",  which is defined in the
// file "CrazyflieData.msg", and has the following properties
// string crazyflieName
//     float64 x                         The x position of the Crazyflie [metres]
//     float64 y                         The y position of the Crazyflie [metres]
//     float64 z                         The z position of the Crazyflie [metres]
//     float64 roll                      The roll component of the intrinsic Euler angles [radians]
//     float64 pitch                     The pitch component of the intrinsic Euler angles [radians]
//     float64 yaw                       The yaw component of the intrinsic Euler angles [radians]
//     float64 acquiringTime #delta t    The time elapsed since the previous "CrazyflieData" was received [seconds]
//     bool occluded                     A boolean indicted whether the Crazyflie for visible at the time of this measurement
// The values in these properties are directly the measurement taken by the Vicon
// motion capture system of the Crazyflie that is to be controlled by this service
//
// request.otherCrazyflies
// This property is an array of "CrazyflieData" structures, what allows access to the
// Vicon measurements of other Crazyflies.
//
// The argument "response" is a structure that is expected to be filled in by this
// service by this function, it has only the following property
// response.ControlCommand
// This property is iteself a structure of type "ControlCommand", which is defined in
// the file "ControlCommand.msg", and has the following properties:
//     float32 roll                      The command sent to the Crazyflie for the body frame x-axis
//     float32 pitch                     The command sent to the Crazyflie for the body frame y-axis
//     float32 yaw                       The command sent to the Crazyflie for the body frame z-axis
//     uint16 motorCmd1                  The command sent to the Crazyflie for motor 1
//     uint16 motorCmd2                  The command sent to the Crazyflie for motor 1
//     uint16 motorCmd3                  The command sent to the Crazyflie for motor 1
//     uint16 motorCmd4                  The command sent to the Crazyflie for motor 1
//     uint8 onboardControllerType       The flag sent to the Crazyflie for indicating how to implement the command
// 
// IMPORTANT NOTES FOR "onboardControllerType"  AND AXIS CONVENTIONS
// > The allowed values for "onboardControllerType" are in the "Defines" section at the
//   top of this file, they are MOTOR_MODE, RATE_MODE, OR ANGLE_MODE.
// > In the PPS exercise we will only use the RATE_MODE.
// > In RATE_MODE the ".roll", ".ptich", and ".yaw" properties of "response.ControlCommand"
//   specify the angular rate in [radians/second] that will be requested from the
//   PID controllers running in the Crazyflie 2.0 firmware.
// > In RATE_MODE the ".motorCmd1" to ".motorCmd4" properties of "response.ControlCommand"
//   are the baseline motor commands requested from the Crazyflie, with the adjustment
//   for body rates being added on top of this in the firmware (i.e., as per the code
//   of the "distribute_power" function provided in exercise sheet 2).
// > In RATE_MODE the axis convention for the roll, pitch, and yaw body rates returned
//   in "response.ControlCommand" should use right-hand coordinate axes with x-forward
//   and z-upwards (i.e., the positive z-axis is aligned with the direction of positive
//   thrust). To assist, teh following is an ASCII art of this convention:
//
// ASCII ART OF THE CRAZYFLIE 2.0 LAYOUT
//
//  > This is a top view,
//  > M1 to M4 stand for Motor 1 to Motor 4,
//  > "CW"  indicates that the motor rotates Clockwise,
//  > "CCW" indicates that the motor rotates Counter-Clockwise,
//  > By right-hand axis convention, the positive z-direction points our of the screen,
//  > This being a "top view" means tha the direction of positive thrust produced
//    by the propellers is also out of the screen.
//
//        ____                         ____
//       /    \                       /    \
//  (CW) | M4 |           x           | M1 | (CCW)
//       \____/\          ^          /\____/
//            \ \         |         / /
//             \ \        |        / /
//              \ \______ | ______/ /
//               \        |        /
//                |       |       |
//        y <-------------o       |
//                |               |
//               / _______________ \
//              / /               \ \
//             / /                 \ \
//        ____/ /                   \ \____
//       /    \/                     \/    \
// (CCW) | M3 |                       | M2 | (CW)
//       \____/                       \____/
//
//   
//
// This function WILL NEED TO BE edited for successful completion of the PPS exercise
bool calculateControlOutput(Controller::Request &request, Controller::Response &response) {

    // This is the "start" of the outer loop controller, add all your control
    // computation here, or you may find it convienient to create functions
    // to keep you code cleaner
    
    
    // Define a local array to fill in with the state error
    float stateErrorInertial[9];

    // Fill in the (x,y,z) position error
    stateErrorInertial[0] = request.ownCrazyflie.x - setpoint[0];
    stateErrorInertial[1] = request.ownCrazyflie.y - setpoint[1];
    stateErrorInertial[2] = request.ownCrazyflie.z - setpoint[2];

    // Compute an estimate of the velocity
    // > As simply the derivative between the current and previous position
    stateErrorInertial[3] = (stateErrorInertial[0] - previous_stateErrorInertial.x) * control_frequency;
    stateErrorInertial[4] = (stateErrorInertial[1] - previous_stateErrorInertial.y) * control_frequency;
    stateErrorInertial[5] = (stateErrorInertial[2] - previous_stateErrorInertial.z) * control_frequency;

    // Fill in the roll and pitch angle measurements directly
    stateErrorInertial[6] = request.ownCrazyflie.roll;
    stateErrorInertial[7] = request.ownCrazyflie.pitch;

    // Fill in the yaw angle error
    // > This error should be "unwrapped" to be in the range
    //   ( -pi , pi )
    // > First, get the yaw error into a local variable
    float yawError = request.ownCrazyflie.yaw - setpoint[3];
    // > Second, "unwrap" the yaw error to the interval ( -pi , pi )
    while(yawError > PI) {yawError -= 2 * PI;}
    while(yawError < -PI) {yawError += 2 * PI;}
    // > Third, put the "yawError" into the "stateError" variable
    stateErrorInertial[8] = yawError;

    
    // CONVERSION INTO BODY FRAME
    // Conver the state erorr from the Inertial frame into the Body frame
    // > Note: the function "convertIntoBodyFrame" is implemented in this file
    //   and by default does not perform any conversion. The equations to convert
    //   the state error into the body frame should be implemented in that function
    //   for successful completion of the PPS exercise
    float stateErrorBody[9];
    convertIntoBodyFrame(stateErrorInertial, stateErrorBody, request.ownCrazyflie.yaw);




    //  **********************
    //  Y   Y    A    W     W
    //   Y Y    A A   W     W
    //    Y    A   A  W     W
    //    Y    AAAAA   W W W
    //    Y    A   A    W W
    //
    // YAW CONTROLLER

    // Instantiate the local variable for the yaw rate that will be requested
    // from the Crazyflie's on-baord "inner-loop" controller
    float yawRate_forResponse = 0;

    // Perform the "-Kx" LQR computation for the yaw rate to respoond with
    for(int i = 0; i < 9; ++i)
    {
        yawRate_forResponse -= gainMatrixYaw[i] * stateErrorBody[i];
    }

    // Put the computed yaw rate into the "response" variable
    response.controlOutput.yaw = yawRate_forResponse;




    //  **************************************
    //  BBBB    OOO   DDDD   Y   Y       ZZZZZ
    //  B   B  O   O  D   D   Y Y           Z
    //  BBBB   O   O  D   D    Y           Z
    //  B   B  O   O  D   D    Y          Z
    //  BBBB    OOO   DDDD     Y         ZZZZZ
    //
    // ALITUDE CONTROLLER (i.e., z-controller)

    // Instantiate the local variable for the thrust adjustment that will be
    // requested from the Crazyflie's on-baord "inner-loop" controller
    float thrustAdjustment = 0;

    // Perform the "-Kx" LQR computation for the thrust adjustment to respoond with
    for(int i = 0; i < 9; ++i)
    {
        thrustAdjustment -= gainMatrixThrust[i] * stateErrorBody[i];
    }

    // Put the computed thrust adjustment into the "response" variable,
    // as well as adding the feed-forward thrust to counter-act gravity.
    // > NOTE: remember that the thrust is commanded per motor, so you sohuld
    //         consider whether the "thrustAdjustment" computed by your
    //         controller needed to be divided by 4 or not.
    // > NOTE: the "gravity_force" value was already divided by 4 when is was
    //         loaded from the .yaml file via the "fetchYamlParameters"
    //         class function in this file.
    response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustAdjustment + gravity_force);
    response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustAdjustment + gravity_force);
    response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustAdjustment + gravity_force);
    response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustAdjustment + gravity_force);




    //  **************************************
    //  BBBB    OOO   DDDD   Y   Y       X   X
    //  B   B  O   O  D   D   Y Y         X X
    //  BBBB   O   O  D   D    Y           X
    //  B   B  O   O  D   D    Y          X X
    //  BBBB    OOO   DDDD     Y         X   X
    //
    // BODY FRAME X CONTROLLER

    // Instantiate the local variable for the pitch rate that will be requested
    // from the Crazyflie's on-baord "inner-loop" controller
    float pitchRate_forResponse = 0;
    
    // Perform the "-Kx" LQR computation for the pitch rate to respoond with
    for(int i = 0; i < 9; ++i)
    {
        pitchRate_forResponse -= gainMatrixPitch[i] * stateErrorBody[i];
    }

    // Put the computed pitch rate into the "response" variable
    response.controlOutput.pitch = pitchRate_forResponse;
    



	//  **************************************
    //  BBBB    OOO   DDDD   Y   Y       Y   Y
    //  B   B  O   O  D   D   Y Y         Y Y
    //  BBBB   O   O  D   D    Y           Y
    //  B   B  O   O  D   D    Y           Y
    //  BBBB    OOO   DDDD     Y           Y
    //
    // BODY FRAME Y CONTROLLER

    // Instantiate the local variable for the roll rate that will be requested
    // from the Crazyflie's on-baord "inner-loop" controller
    float rollRate_forResponse = 0;
    
    // Perform the "-Kx" LQR computation for the roll rate to respoond with
    for(int i = 0; i < 9; ++i)
    {
        rollRate_forResponse -= gainMatrixRoll[i] * stateErrorBody[i];
    }

    // Put the computed roll rate into the "response" variable
    response.controlOutput.roll = rollRate_forResponse;
    
    



    // PREPARE AND RETURN THE VARIABLE "response"

    /*choosing the Crazyflie onBoard controller type.
    it can either be Motor, Rate or Angle based */
    // response.controlOutput.onboardControllerType = MOTOR_MODE;
    response.controlOutput.onboardControllerType = RATE_MODE;
    // response.controlOutput.onboardControllerType = ANGLE_MODE;

    
    // SAVE THE STATE ERROR TO BE USED NEXT TIME THIS FUNCTION IS CALLED
    previous_stateErrorInertial = request.ownCrazyflie; // we have already used previous location, update it

    // Adjust (x,y,z) for the stepoint
    previous_stateErrorInertial.x = request.ownCrazyflie.x - setpoint[0];
    previous_stateErrorInertial.y = request.ownCrazyflie.y - setpoint[1];
    previous_stateErrorInertial.z = request.ownCrazyflie.z - setpoint[2];

    // Adjust yaw for the stepoint
    previous_stateErrorInertial.yaw = stateErrorInertial[8];



    // PUBLISH THE CURRENT X,Y,Z, AND YAW (if required)
    if (shouldPublishCurrent_xyz_yaw)
    {
	    // publish setpoint for FollowController of another student group
	    Setpoint temp_current_xyz_yaw;
	    // Fill in the x,y,z, and yaw info directly from the data provided to this
	    // function
	    temp_current_xyz_yaw.x   = request.ownCrazyflie.x;
	    temp_current_xyz_yaw.y   = request.ownCrazyflie.y;
	    temp_current_xyz_yaw.z   = request.ownCrazyflie.z;
	    temp_current_xyz_yaw.yaw = request.ownCrazyflie.yaw;
	    my_current_xyz_yaw_publisher.publish(temp_current_xyz_yaw);
	}


    // DEBUGGING CODE:
    // As part of the D-FaLL-System we have defined a message type names"DebugMsg".
    // By fill this message with data and publishing it you can display the data in
    // real time using rpt plots. Instructions for using rqt plots can be found on
    // the wiki of the D-FaLL-System repository

    // Instantiate a local variable of type "DebugMsg", see the file "DebugMsg.msg"
    // (located in the "msg" folder) to see the full structure of this message.
    DebugMsg debugMsg;

    // Fill the debugging message with the data provided by Vicon
    debugMsg.vicon_x = request.ownCrazyflie.x;
    debugMsg.vicon_y = request.ownCrazyflie.y;
    debugMsg.vicon_z = request.ownCrazyflie.z;
    debugMsg.vicon_roll = request.ownCrazyflie.roll;
    debugMsg.vicon_pitch = request.ownCrazyflie.pitch;
    debugMsg.vicon_yaw = request.ownCrazyflie.yaw;

    // Fill in the debugging message with any other data you would like to display
    // in real time. For example, it might be useful to display the thrust
    // adjustment computed by the z-altitude controller.
    // The "DebugMsg" type has 10 properties from "value_1" to "value_10", all of
    // type "float64" that you can fill in with data you would like to plot in
    // real-time.
    // debugMsg.value_1 = thrustAdjustment;
    // ......................
    // debugMsg.value_10 = your_variable_name;

    // Publish the "debugMsg"
    debugPublisher.publish(debugMsg);


    // An alternate debugging technique is to print out data directly to the
    // command line from which this node was launched.

    // An example of "printing out" the data from the "request" argument to the
    // command line. This might be useful for debugging.
    // ROS_INFO_STREAM("x-coordinates: " << request.ownCrazyflie.x);
    // ROS_INFO_STREAM("y-coordinates: " << request.ownCrazyflie.y);
    // ROS_INFO_STREAM("z-coordinates: " << request.ownCrazyflie.z);
    // ROS_INFO_STREAM("roll: " << request.ownCrazyflie.roll);
    // ROS_INFO_STREAM("pitch: " << request.ownCrazyflie.pitch);
    // ROS_INFO_STREAM("yaw: " << request.ownCrazyflie.yaw);
    // ROS_INFO_STREAM("Delta t: " << request.ownCrazyflie.acquiringTime);

    // An example of "printing out" the control actions computed.
    // ROS_INFO_STREAM("thrustAdjustment = " << thrustAdjustment);
    // ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
    // ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
    // ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);

    // An example of "printing out" the "thrust-to-command" conversion parameters.
    // ROS_INFO_STREAM("motorPoly 0:" << motorPoly[0]);
    // ROS_INFO_STREAM("motorPoly 0:" << motorPoly[1]);
    // ROS_INFO_STREAM("motorPoly 0:" << motorPoly[2]);

    // An example of "printing out" the per motor 16-bit command computed.
    // ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
    // ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
    // ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
    // ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);

    // Return "true" to indicate that the control computation was performed successfully
    return true;
}






//    ------------------------------------------------------------------------------
//    RRRR    OOO   TTTTT    A    TTTTT  EEEEE       III  N   N  TTTTT   OOO
//    R   R  O   O    T     A A     T    E            I   NN  N    T    O   O
//    RRRR   O   O    T    A   A    T    EEE          I   N N N    T    O   O
//    R  R   O   O    T    AAAAA    T    E            I   N  NN    T    O   O
//    R   R   OOO     T    A   A    T    EEEEE       III  N   N    T     OOO
//
//    BBBB    OOO   DDDD   Y   Y       FFFFF  RRRR     A    M   M  EEEEE
//    B   B  O   O  D   D   Y Y        F      R   R   A A   MM MM  E
//    BBBB   O   O  D   D    Y         FFF    RRRR   A   A  M M M  EEE
//    B   B  O   O  D   D    Y         F      R  R   AAAAA  M   M  E
//    BBBB    OOO   DDDD     Y         F      R   R  A   A  M   M  EEEEE
//    ----------------------------------------------------------------------------------

// The arguments for this function are as follows:
// est
// This is an array of length 9 with the estimates the error of of the following values
// relative to the sepcifed setpoint:
//     est[0]    x position of the Crazyflie relative to the inertial frame origin [meters]
//     est[1]    y position of the Crazyflie relative to the inertial frame origin [meters]
//     est[2]    z position of the Crazyflie relative to the inertial frame origin [meters]
//     est[3]    x-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     est[4]    y-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     est[5]    z-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     est[6]    The roll  component of the intrinsic Euler angles [radians]
//     est[7]    The pitch component of the intrinsic Euler angles [radians]
//     est[8]    The yaw   component of the intrinsic Euler angles [radians]
// 
// estBody
// This is an empty array of length 9, this function should fill in all elements of this
// array with the same ordering as for the "est" argument, expect that the (x,y) position
// and (x,y) velocities are rotated into the body frame.
//
// yaw_measured
// This is the yaw component of the intrinsic Euler angles in [radians] as measured by
// the Vicon motion capture system
//
// This function WILL NEED TO BE edited for successful completion of the PPS exercise
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured)
{
	if (shouldPerformConvertIntoBodyFrame)
	{
		float sinYaw = sin(yaw_measured);
    	float cosYaw = cos(yaw_measured);

    	// Fill in the (x,y,z) position estimates to be returned
	    stateBody[0] = stateInertial[0] * cosYaw  +  stateInertial[1] * sinYaw;
	    stateBody[1] = stateInertial[1] * cosYaw  -  stateInertial[0] * sinYaw;
	    stateBody[2] = stateInertial[2];

	    // Fill in the (x,y,z) velocity estimates to be returned
	    stateBody[3] = stateInertial[3] * cosYaw  +  stateInertial[4] * sinYaw;
	    stateBody[4] = stateInertial[4] * cosYaw  -  stateInertial[3] * sinYaw;
	    stateBody[5] = stateInertial[5];

	    // Fill in the (roll,pitch,yaw) estimates to be returned
	    stateBody[6] = stateInertial[6];
	    stateBody[7] = stateInertial[7];
	    stateBody[8] = stateInertial[8];
	}
	else
	{
	    // Fill in the (x,y,z) position estimates to be returned
	    stateBody[0] = stateInertial[0];
	    stateBody[1] = stateInertial[1];
	    stateBody[2] = stateInertial[2];

	    // Fill in the (x,y,z) velocity estimates to be returned
	    stateBody[3] = stateInertial[3];
	    stateBody[4] = stateInertial[4];
	    stateBody[5] = stateInertial[5];

	    // Fill in the (roll,pitch,yaw) estimates to be returned
	    stateBody[6] = stateInertial[6];
	    stateBody[7] = stateInertial[7];
	    stateBody[8] = stateInertial[8];
	}
}





//    ------------------------------------------------------------------------------
//    N   N  EEEEE  W     W   TTTTT   OOO   N   N        CCCC  M   M  DDDD
//    NN  N  E      W     W     T    O   O  NN  N       C      MM MM  D   D
//    N N N  EEE    W     W     T    O   O  N N N  -->  C      M M M  D   D
//    N  NN  E       W W W      T    O   O  N  NN       C      M   M  D   D
//    N   N  EEEEE    W W       T     OOO   N   N        CCCC  M   M  DDDD
//
//     CCCC   OOO   N   N  V   V  EEEEE  RRRR    SSSS  III   OOO   N   N
//    C      O   O  NN  N  V   V  E      R   R  S       I   O   O  NN  N
//    C      O   O  N N N  V   V  EEE    RRRR    SSS    I   O   O  N N N
//    C      O   O  N  NN   V V   E      R  R       S   I   O   O  N  NN
//     CCCC   OOO   N   N    V    EEEEE  R   R  SSSS   III   OOO   N   N
//    ----------------------------------------------------------------------------------

// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
float computeMotorPolyBackward(float thrust)
{
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
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

// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void setpointCallback(const Setpoint& newSetpoint)
{
    setpoint[0] = newSetpoint.x;
    setpoint[1] = newSetpoint.y;
    setpoint[2] = newSetpoint.z;
    setpoint[3] = newSetpoint.yaw;
}



// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// > This function is called anytime a message is published on the topic to which the
//   class instance variable "xyz_yaw_to_follow_subscriber" is subscribed
void xyz_yaw_to_follow_callback(const Setpoint& newSetpoint)
{
        //ROS_INFO("DEBUGGING: Received new setpoint from another agent");
	// The setpoint should only be updated if allow by the respective booelan
	if (shouldFollowAnotherAgent)
	{
	    setpoint[0] = newSetpoint.x;
	    setpoint[1] = newSetpoint.y;
	    setpoint[2] = newSetpoint.z;
	    setpoint[3] = newSetpoint.yaw;
	}
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
			case FETCH_YAML_CUSTOM_CONTROLLER_COORDINATOR:
			// Let the user know that this message was received
			ROS_INFO("The CustomControllerService received the message that YAML parameters were (re-)loaded");
			// Let the user know from where the paramters are being fetched
			ROS_INFO("> Now fetching the parameter values from the this machine");
			// Call the function that fetches the parameters
			fetchYamlParameters(nodeHandle_to_coordinator_parameter_service);
			break;

		case FETCH_YAML_CUSTOM_CONTROLLER_AGENT:
			// Let the user know that this message was received
			ROS_INFO("The CustomControllerService received the message that YAML parameters were (re-)loaded");
			// Let the user know which paramters are being fetch
			ROS_INFO("> Now fetching the parameter values from the this machine");
			// Call the function that fetches the parameters
			fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);
			break;

		default:
			// Let the user know that the command was not relevant
			ROS_INFO("The CustomControllerService received the message that YAML parameters were (re-)loaded");
			ROS_INFO("> However the parameters do not relate to this controller, hence nothing will be fetched.");
			break;
	}
}


// This function CAN BE edited for successful completion of the PPS exercise, and the
// use of parameters fetched from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void fetchYamlParameters(ros::NodeHandle& nodeHandle)
{
    // Here we load the parameters that are specified in the CustomController.yaml file

	// Add the "CustomController" namespace to the "nodeHandle"
	nodeHandle_for_customController = ros::NodeHandle(nodeHandle + "/CustomController");

	// > The mass of the crazyflie
    cf_mass = getParameterFloat(nodeHandle_for_customController , "mass");

	// Display one of the YAML parameters to debug if it is working correctly
	//ROS_INFO_STREAM("DEBUGGING: mass leaded from loacl file = " << cf_mass );

    // > The frequency at which the "computeControlOutput" is being called, as determined
    //   by the frequency at which the Vicon system provides position and attitude data
    control_frequency = getParameterFloat(nodeHandle_for_customController, "control_frequency");

    // > The co-efficients of the quadratic conversation from 16-bit motor command to
    //   thrust force in Newtons
    getParameterFloatVector(nodeHandle_for_customController, "motorPoly", motorPoly, 3);

    // > The boolean for whether to execute the convert into body frame function
    shouldPerformConvertIntoBodyFrame = getParameterBool(nodeHandle_for_customController, "shouldPerformConvertIntoBodyFrame");

    // > The boolean indicating whether the (x,y,z,yaw) of this agent should be published
    //   or not
    shouldPublishCurrent_xyz_yaw = getParameterBool(nodeHandle_for_customController, "shouldPublishCurrent_xyz_yaw");

    // > The boolean indicating whether the (x,y,z,yaw) setpoint of this agent should adapted in
	//   an attempt to follow the "my_current_xyz_yaw_topic" from another agent
    shouldFollowAnotherAgent = getParameterBool(nodeHandle_for_customController, "shouldFollowAnotherAgent");

    // > The ordered vector for ID's that specifies how the agents should follow eachother
    follow_in_a_line_agentIDs.clear();
    int temp_number_of_agents_in_a_line = getParameterIntVectorWithUnknownLength(nodeHandle_for_customController, "follow_in_a_line_agentIDs", follow_in_a_line_agentIDs);
    // > Double check that the sizes agree
    if ( temp_number_of_agents_in_a_line != follow_in_a_line_agentIDs.size() )
    {
    	// Update the user if the sizes don't agree
    	ROS_ERROR_STREAM("parameter 'follow_in_a_line_agentIDs' was loaded with two different lengths, " << temp_number_of_agents_in_a_line << " versus " << follow_in_a_line_agentIDs.size() );
    }

    // Call the function that computes details an values that are needed from these
    // parameters loaded above
    processFetchedParameters();

}


// This function CAN BE edited for successful completion of the PPS exercise, and the
// use of parameters loaded from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void processFetchedParameters()
{
    // Compute the feed-forward force that we need to counteract gravity (i.e., mg)
    // > in units of [Newtons]
    gravity_force = cf_mass * 9.81/(1000*4);

    // Look-up which agent should be followed
    if (shouldFollowAnotherAgent)
    {
    	// Only bother if the "follow_in_a_line_agentIDs" vector has a non-zero length
    	if (follow_in_a_line_agentIDs.size() > 0)
    	{
    		// Instantiate a local boolean variable for checking whether we found
    		// our own agent ID in the list
    		bool foundMyAgentID = false;
    		// Iterate through the vector of "follow_in_a_line_agentIDs"
	    	for ( int i=0 ; i<follow_in_a_line_agentIDs.size() ; i++ )
	    	{
	    		// Check if the current entry matches the ID of this agent
	    		if (follow_in_a_line_agentIDs[i] == my_agentID)
	    		{
	    			// Set the boolean flag that we have found out own agent ID
	    			foundMyAgentID = true;
                    //ROS_INFO_STREAM("DEBUGGING: found my agent ID at index " << i );
	    			// If it is the first entry, then this agent is the leader
	    			if (i==0)
	    			{
	    				// The leader does not follow anyone else
	    				shouldFollowAnotherAgent = false;
    					agentID_to_follow = 0;
	    			}
	    			else
	    			{
	    				// The agent ID to follow is the previous entry
	    				agentID_to_follow = follow_in_a_line_agentIDs[i-1];	
                        shouldFollowAnotherAgent = true;
	    				// Subscribe to the "my_current_xyz_yaw_topic" of the agent ID
	    				// that this agent should be following
	    				ros::NodeHandle nodeHandle("~");
	    				xyz_yaw_to_follow_subscriber = nodeHandle.subscribe("/" + std::to_string(agentID_to_follow) + "/my_current_xyz_yaw_topic", 1, xyz_yaw_to_follow_callback);
	    				//ROS_INFO_STREAM("DEBUGGING: subscribed to agent ID = " << agentID_to_follow );
	    			}
	    			// Break out of the for loop as the assumption is that each agent ID
	    			// appears only once in the "follow_in_a_line_agentIDs" vector of ID's
	    			break;
	    		}
	    	}
	    	// Check whether we found our own agent ID
	    	if (!foundMyAgentID)
	    	{
                ROS_INFO("DEBUGGING: not following because my ID was not found");
	    		// Revert to the default of not following any agent
    			shouldFollowAnotherAgent = false;
    			agentID_to_follow = 0;
	    	}
    	}
    	else
    	{
    		// As the "follow_in_a_line_agentIDs" vector has length zero, revert to the
    		// default of not following any agent
    		shouldFollowAnotherAgent = false;
    		agentID_to_follow = 0;
    		//ROS_INFO("DEBUGGING: not following because line vector has length zero");
    	}
    }
    else
    {
    	// Set to its default value the integer specifying the ID of the agent that will
    	// be followed by this agent
		agentID_to_follow = 0;
		//ROS_INFO("DEBUGGING: not following because I was asked not to follow");
    }

    if (shouldFollowAnotherAgent)
    {
    	ROS_INFO_STREAM("This agent (with ID " << my_agentID << ") will now follow agent ID " << agentID_to_follow );
    }
}


/*
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void customYAMLasMessageCallback(const CustomControllerYAML& newCustomControllerParameters)
{
	// This function puts all the same parameters as the "fetchYamlParameters" function
	// above into the same variables.
	// The difference is that this function allows us to send all parameters from one
	// central coordinator node
	cf_mass = newCustomControllerParameters.mass;
	control_frequency = newCustomControllerParameters.control_frequency;
	for (int i=0;i<3;i++)
	{
		motorPoly[i] = newCustomControllerParameters.motorPoly[i];
	}

	// > The boolean for whether to execute the convert into body frame function
    shouldPerformConvertIntoBodyFrame = newCustomControllerParameters.shouldPerformConvertIntoBodyFrame;

	shouldPublishCurrent_xyz_yaw = newCustomControllerParameters.shouldPublishCurrent_xyz_yaw;
	shouldFollowAnotherAgent = newCustomControllerParameters.shouldFollowAnotherAgent;
	follow_in_a_line_agentIDs.clear();
	for ( int i=0 ; i<newCustomControllerParameters.follow_in_a_line_agentIDs.size() ; i++ )
	{
		follow_in_a_line_agentIDs.push_back( newCustomControllerParameters.follow_in_a_line_agentIDs[i] );
	}

	// Let the user know that the message was received with new YAML parameters
	ROS_INFO("Received message containing a new set of Custom Controller YAML parameters");

	// Display one of the YAML parameters to debug if it is working correctly
	ROS_INFO_STREAM("DEBUGGING: mass received in message = " << newCustomControllerParameters.mass );	

	// Call the function that computes details an values that are needed from these
    // parameters loaded above
    ros::NodeHandle nodeHandle("~");
    processFetchedParameters(nodeHandle);

}
*/


//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------


// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val.size();
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
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
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------

// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
int main(int argc, char* argv[]) {
    
    // Starting the ROS-node
    ros::init(argc, argv, "CustomControllerService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the agent ID as the "ROS_NAMESPACE" this computer.
    // NOTES:
    // > If you look at the "Student.launch" file in the "launch" folder, you will see
    //   the following line of code:
    //   <param name="studentID" value="$(optenv ROS_NAMESPACE)" />
    //   This line of code adds a parameter named "studentID" to the "PPSClient"
    // > Thus, to get access to this "studentID" paremeter, we first need to get a handle
    //   to the "PPSClient" node within which this controller service is nested.
    // Get the namespace of this "CustomControllerService" node
    std::string m_namespace = ros::this_node::getNamespace();
    // Get the handle to the "PPSClient" node
    ros::NodeHandle PPSClient_nodeHandle(m_namespace + "/PPSClient");
    // Get the value of the "studentID" parameter into the instance variable "my_agentID"
    if(!PPSClient_nodeHandle.getParam("studentID", my_agentID))
    {
    	// Throw an error if the student ID parameter could not be obtained
		ROS_ERROR("Failed to get studentID from FollowN_1Service");
	}


	// *********************************************************************************
	// EVERYTHING THAT RELATES TO FETCHING PARAMETERS FROM A YAML FILE

	// Set the class variable "nodeHandle_to_own_agent_parameter_service" to be a node handle
    // for the parameter service that is running on the machone of this agent
    nodeHandle_to_own_agent_parameter_service = ros::NodeHandle(m_namespace + "/ParameterService");

    // Set the class variable "nodeHandle_to_coordinator_parameter_service" to be a node handle
    // for the parameter service that is running on the coordinate machine
    ros::NodeHandle coordinator_nodeHandle = ros::NodeHandle();
    nodeHandle_to_coordinator_parameter_service = ros::NodeHandle(coordinator_nodeHandle + "/ParameterService");

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    ros::Subscriber controllerYamlReadyForFetchSubscriber_to_agent = nodeHandle_to_own_agent_parameter_service.subscribe("controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator_parameter_service.subscribe("controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);

	// Call the class function that loads the parameters for this class.
    fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);

    // *********************************************************************************



    // Instantiate the instance variable "debugPublisher" to be a "ros::Publisher" that
    // advertises under the name "DebugTopic" and is a message with the structure
    // defined in the file "DebugMsg.msg" (located in the "msg" folder).
    debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

    // Instantiate the local variable "setpointSubscriber" to be a "ros::Subscriber"
    // type variable that subscribes to the "Setpoint" topic and calls the class function
    // "setpointCallback" each time a messaged is received on this topic and the message
    // is passed as an input argument to the "setpointCallback" class function.
    ros::Subscriber setpointSubscriber = nodeHandle.subscribe("Setpoint", 1, setpointCallback);

    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "CustomController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("CustomController", calculateControlOutput);

    // Create a "ros::NodeHandle" type local variable "namespace_nodeHandle" that points
    // to the name space of this node, i.e., "d_fall_pps" as specified by the line:
    //     "using namespace d_fall_pps;"
    // in the "DEFINES" section at the top of this file.
    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());

    // Instantiate the instance variable "my_current_xyz_yaw_publisher" to be a "ros::Publisher"
    // that advertises under the name "<my_agentID>/my_current_xyz_yaw_topic" where <my_agentID>
    // is filled in with the student ID number of this computer. The messages published will
    // have the structure defined in the file "Setpoint.msg" (located in the "msg" folder).
    my_current_xyz_yaw_publisher = nodeHandle.advertise<Setpoint>("/" + std::to_string(my_agentID) + "/my_current_xyz_yaw_topic", 1);

    // Print out some information to the user.
    ROS_INFO("CustomControllerService ready");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
