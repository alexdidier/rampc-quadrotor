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
#include "dfall_pkg/ViconData.h"
#include "dfall_pkg/Setpoint.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/Controller.h"
#include "dfall_pkg/DebugMsg.h"

// Include the Parameter Service shared definitions
#include "nodes/Constants.h"

#include <std_msgs/Int32.h>

//#include <Eigen/Dense>



//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.

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

#define CF_COMMAND_TYPE_MOTOR   6
#define CF_COMMAND_TYPE_RATE    7
#define CF_COMMAND_TYPE_ANGLE   8

#define ESTIMATOR_METHOD_FINITE_DIFFERENCE          1
#define ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION   2   // (DEFAULT)
#define ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED      3

int estimator_method = ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool shouldDisplayDebugInfo = false;

float estimator_frequency = 200;

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

// Namespacing the package
using namespace dfall_pkg;


//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
// > For the (x,y,z) position
std::vector<float> PMKF_Ahat_row1_for_positions (2,0.0);
std::vector<float> PMKF_Ahat_row2_for_positions (2,0.0);
std::vector<float> PMKF_Kinf_for_positions      (2,0.0);
// > For the (roll,pitch,yaw) angles
std::vector<float> PMKF_Ahat_row1_for_angles    (2,0.0);
std::vector<float> PMKF_Ahat_row2_for_angles    (2,0.0);
std::vector<float> PMKF_Kinf_for_angles         (2,0.0);

std::vector<float> gainMatrixRollRate_Nested            (3,0.0);
std::vector<float> gainMatrixPitchRate_Nested           (3,0.0);
std::vector<float> gainMatrixYawRate_Nested             (3,0.0);

float current_xzy_rpy_measurement[6];
float previous_xzy_rpy_measurement[6];


// Variables for controller
float cf_mass;                       // Crazyflie mass in grams
std::vector<float> motorPoly(3);     // Coefficients of the 16-bit command to thrust conversion
float control_frequency;             // Frequency at which the controller is running
float gravity_force;                 // The weight of the Crazyflie in Newtons, i.e., mg

float previous_stateErrorInertial[9];     // The location error of the Crazyflie at the "previous" time step

std::vector<float>  setpoint{0.0,0.0,0.4,0.0};     // The setpoints for (x,y,z) position and yaw angle, in that order


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

void angleControlledCrazyflie(float stateInertial[12], float input_angle[4], int Ts_div, Controller::Response &response);
void perform_estimator_update_state_interial(Controller::Request &request, float (&stateInertialEstimate)[12]);
void performEstimatorUpdate_forStateInterial_viaFiniteDifference(float (&stateInertialEstimate)[12]);
void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter(float (&stateInertialEstimate)[12]);




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
// > In the classroom exercise we will only use the RATE_MODE.
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
// This function WILL NEED TO BE edited for successful completion of the classroom exercise

void angleControlledCrazyflie(float stateInertial[12], float input_angle[4], int Ts_div, Controller::Response &response)
{
    float roll_sp = input_angle[0];
    float pitch_sp = input_angle[1];
    float yaw_sp = input_angle[2];
    float ft_sp = input_angle[3];

    // initialize variables that will contain w_x, w_y and w_z
	float w_x_sp  = 0;
	float w_y_sp = 0;
	float w_z_sp   = 0;

	// Create the angle error to use for the inner controller
	float angle_error[3] = {
		stateInertial[6] - roll_sp,
		stateInertial[7] - pitch_sp,
        stateInertial[8] - yaw_sp
	};
	// Perform the "-Kx" LQR computation for the rates and thrust:
	for(int i = 0; i < 3; ++i)
	{
		// BODY FRAME Y CONTROLLER
		w_x_sp  -= gainMatrixRollRate_Nested[i]  * angle_error[i];
		// BODY FRAME X CONTROLLER
		w_y_sp -= gainMatrixPitchRate_Nested[i] * angle_error[i];
		// BODY FRAME Z CONTROLLER
		w_z_sp -= gainMatrixYawRate_Nested[i]   * angle_error[i];
	}


	// UPDATE THE "RETURN" THE VARIABLE NAMED "response"

	// Put the computed rates and thrust into the "response" variable
	// > For roll, pitch, and yaw:
	response.controlOutput.roll  = w_x_sp;
	response.controlOutput.pitch = w_y_sp;
	response.controlOutput.yaw   = w_z_sp;
	// > For the thrust adjustment we must add the feed-forward thrust to counter-act gravity.
	// > NOTE: remember that the thrust is commanded per motor, so you sohuld
	//         consider whether the "thrustAdjustment" computed by your
	//         controller needed to be divided by 4 or not.
	// > NOTE: the "gravity_force_quarter" value was already divided by 4 when
	//         it was loaded/processes from the .yaml file.
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(ft_sp/4.0);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(ft_sp/4.0);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(ft_sp/4.0);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(ft_sp/4.0);

	// Specify that this controller is a rate controller
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTOR;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;

	// Display some details (if requested)
	if (shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("thrust    =" << ft_sp);
		ROS_INFO_STREAM("rollrate  =" << w_x_sp);
		ROS_INFO_STREAM("pitchrate =" << w_y_sp);
		ROS_INFO_STREAM("yawrate   =" << w_z_sp);
	}
}


bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// This is the "start" of the outer loop controller, add all your control
	// computation here, or you may find it convienient to create functions
	// to keep you code cleaner

    // Define a local array to fill in with the state error
    float stateInertialEstimate[12];

	perform_estimator_update_state_interial(request, stateInertialEstimate);

    // The estimate of the state is inside stateInertialEstimate now. Next, compute the error
    // Should we convert into body frame for our MPC controller? Let's skip it for now, YAW is going to be zero in our case

    float x0_full_state[12] = {-setpoint[0], -setpoint[1], -setpoint[2], 0, 0, 0, 0, 0, 0, 0, 0, 0};

    float correction_z = -0.82*(0.4 - stateInertialEstimate[2]) - 0.22 * (0 - stateInertialEstimate[5]) + cf_mass/1000*9.8;

    float input_angle[4] = {-setpoint[0], 0, 0, correction_z};

    // create a function that takes as input angle references, like a crazyflie entity with input angles
    angleControlledCrazyflie(stateInertialEstimate, input_angle, 1, response);



    // Return "true" to indicate that the control computation was performed successfully
    return true;
}


//    ------------------------------------------------------------------------------
//    EEEEE   SSSS  TTTTT  III  M   M    A    TTTTT  III   OOO   N   N
//    E      S        T     I   MM MM   A A     T     I   O   O  NN  N
//    EEE     SSS     T     I   M M M  A   A    T     I   O   O  N N N
//    E          S    T     I   M   M  AAAAA    T     I   O   O  N  NN
//    EEEEE  SSSS     T    III  M   M  A   A    T    III   OOO   N   N
//    ------------------------------------------------------------------------------



void perform_estimator_update_state_interial(Controller::Request &request, float (&stateInertialEstimate)[12])
{

	// PUT THE CURRENT MEASURED DATA INTO THE CLASS VARIABLE
	// > for (x,y,z) position
	current_xzy_rpy_measurement[0] = request.ownCrazyflie.x;
	current_xzy_rpy_measurement[1] = request.ownCrazyflie.y;
	current_xzy_rpy_measurement[2] = request.ownCrazyflie.z;
	// > for (roll,pitch,yaw) angles
	current_xzy_rpy_measurement[3] = request.ownCrazyflie.roll;
	current_xzy_rpy_measurement[4] = request.ownCrazyflie.pitch;
	current_xzy_rpy_measurement[5] = request.ownCrazyflie.yaw;


	// FILLE IN THE STATE INERTIAL ESTIMATE TO BE USED FOR CONTROL
	switch (estimator_method)
	{
		// Estimator based on finte differences
		case ESTIMATOR_METHOD_FINITE_DIFFERENCE:
		{
            // RUN THE FINITE DIFFERENCE ESTIMATOR and fill stateEstimate with result
            performEstimatorUpdate_forStateInterial_viaFiniteDifference(stateInertialEstimate);
			break;
		}
		// Estimator based on Point Mass Kalman Filter
		case ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION:
		{
            // RUN THE POINT MASS KALMAN FILTER ESTIMATOR and fill stateEstimate with result
            performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter(stateInertialEstimate);
			break;
		}
		// Handle the exception
		default:
		{
			// Display that the "estimator_method" was not recognised
			ROS_INFO_STREAM("[DEMO CONTROLLER] ERROR: in the 'calculateControlOutput' function of the 'DemoControllerService': the 'estimator_method' is not recognised.");
			// Transfer the finite difference estimate by default and fill stateEstimate with result
            performEstimatorUpdate_forStateInterial_viaFiniteDifference(stateInertialEstimate);
			break;
		}
	}


	// NOW THAT THE ESTIMATORS HAVE ALL BEEN RUN, PUT THE CURRENT
	// MEASURED DATA INTO THE CLASS VARIABLE FOR THE PREVIOUS
	// > for (x,y,z) position
	previous_xzy_rpy_measurement[0] = current_xzy_rpy_measurement[0];
	previous_xzy_rpy_measurement[1] = current_xzy_rpy_measurement[1];
	previous_xzy_rpy_measurement[2] = current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	previous_xzy_rpy_measurement[3] = current_xzy_rpy_measurement[3];
	previous_xzy_rpy_measurement[4] = current_xzy_rpy_measurement[4];
	previous_xzy_rpy_measurement[5] = current_xzy_rpy_measurement[5];
}

void performEstimatorUpdate_forStateInterial_viaFiniteDifference(float (&stateInertialEstimate)[12])
{
	// PUT IN THE CURRENT MEASUREMENT DIRECTLY
	// > for (x,y,z) position
	stateInertialEstimate[0]  = current_xzy_rpy_measurement[0];
	stateInertialEstimate[1]  = current_xzy_rpy_measurement[1];
	stateInertialEstimate[2]  = current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	stateInertialEstimate[6]  = current_xzy_rpy_measurement[3];
	stateInertialEstimate[7]  = current_xzy_rpy_measurement[4];
	stateInertialEstimate[8]  = current_xzy_rpy_measurement[5];

	// COMPUTE THE VELOCITIES VIA FINITE DIFFERENCE
	// > for (x,y,z) velocities
	stateInertialEstimate[3]  = (current_xzy_rpy_measurement[0] - previous_xzy_rpy_measurement[0]) * estimator_frequency;
	stateInertialEstimate[4]  = (current_xzy_rpy_measurement[1] - previous_xzy_rpy_measurement[1]) * estimator_frequency;
	stateInertialEstimate[5]  = (current_xzy_rpy_measurement[2] - previous_xzy_rpy_measurement[2]) * estimator_frequency;
	// > for (roll,pitch,yaw) velocities
	stateInertialEstimate[9]  = (current_xzy_rpy_measurement[3] - previous_xzy_rpy_measurement[3]) * estimator_frequency;
	stateInertialEstimate[10] = (current_xzy_rpy_measurement[4] - previous_xzy_rpy_measurement[4]) * estimator_frequency;
	stateInertialEstimate[11] = (current_xzy_rpy_measurement[5] - previous_xzy_rpy_measurement[5]) * estimator_frequency;
}



void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter(float (&stateInertialEstimate)[12])
{
	// PERFORM THE KALMAN FILTER UPDATE STEP
	// > First take a copy of the estimator state
	float temp_PMKFstate[12];
	for(int i = 0; i < 12; ++i)
	{
		temp_PMKFstate[i]  = stateInertialEstimate[i];
	}
	// > Now perform update for:
	// > x position and velocity:
	stateInertialEstimate[0] = PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[0] + PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[3] + PMKF_Kinf_for_positions[0]*current_xzy_rpy_measurement[0];
	stateInertialEstimate[3] = PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[0] + PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[3] + PMKF_Kinf_for_positions[1]*current_xzy_rpy_measurement[0];
	// > y position and velocity:
	stateInertialEstimate[1] = PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[1] + PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[4] + PMKF_Kinf_for_positions[0]*current_xzy_rpy_measurement[1];
	stateInertialEstimate[4] = PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[1] + PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[4] + PMKF_Kinf_for_positions[1]*current_xzy_rpy_measurement[1];
	// > z position and velocity:
	stateInertialEstimate[2] = PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[2] + PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[5] + PMKF_Kinf_for_positions[0]*current_xzy_rpy_measurement[2];
	stateInertialEstimate[5] = PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[2] + PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[5] + PMKF_Kinf_for_positions[1]*current_xzy_rpy_measurement[2];

	// > roll  position and velocity:
	stateInertialEstimate[6]  = PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[6] + PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[9]  + PMKF_Kinf_for_angles[0]*current_xzy_rpy_measurement[3];
	stateInertialEstimate[9]  = PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[6] + PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[9]  + PMKF_Kinf_for_angles[1]*current_xzy_rpy_measurement[3];
	// > pitch position and velocity:
	stateInertialEstimate[7]  = PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[7] + PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[10] + PMKF_Kinf_for_angles[0]*current_xzy_rpy_measurement[4];
	stateInertialEstimate[10] = PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[7] + PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[10] + PMKF_Kinf_for_angles[1]*current_xzy_rpy_measurement[4];
	// > yaw   position and velocity:
	stateInertialEstimate[8]  = PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[8] + PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[11] + PMKF_Kinf_for_angles[0]*current_xzy_rpy_measurement[5];
	stateInertialEstimate[11] = PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[8] + PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[11] + PMKF_Kinf_for_angles[1]*current_xzy_rpy_measurement[5];

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

// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
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

// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
void setpointCallback(const Setpoint& newSetpoint)
{
    setpoint[0] = newSetpoint.x;
    setpoint[1] = newSetpoint.y;
    setpoint[2] = newSetpoint.z;
    setpoint[3] = newSetpoint.yaw;
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


// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
void yamlReadyForFetchCallback(const std_msgs::Int32& msg)
{
	// Extract from the "msg" for which controller the and from where to fetch the YAML
	// parameters
	int controller_to_fetch_yaml = msg.data;

	// Switch between fetching for the different controllers and from different locations
	switch(controller_to_fetch_yaml)
	{

		// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
		case FETCH_YAML_MPC_CONTROLLER_FROM_OWN_AGENT:
		{
			// Let the user know that this message was received
			ROS_INFO("[MPC CONTROLLER] Received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from this agent.");
			// Create a node handle to the parameter service running on this agent's machine
			ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);
			// Call the function that fetches the parameters
			fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);
			break;
		}

		// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
		case FETCH_YAML_MPC_CONTROLLER_FROM_COORDINATOR:
		{
			// Let the user know that this message was received
			ROS_INFO("[MPC CONTROLLER] Received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from the coordinator.");
			// Create a node handle to the parameter service running on the coordinator machine
			ros::NodeHandle nodeHandle_to_coordinator_parameter_service(namespace_to_coordinator_parameter_service);
			// Call the function that fetches the parameters
			fetchYamlParameters(nodeHandle_to_coordinator_parameter_service);
			break;
		}

		default:
		{
			// Let the user know that the command was not relevant
			//ROS_INFO("The CustomControllerService received the message that YAML parameters were (re-)loaded");
			//ROS_INFO("> However the parameters do not relate to this controller, hence nothing will be fetched.");
			break;
		}
	}
}


// This function CAN BE edited for successful completion of the classroom exercise, and the
// use of parameters fetched from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void fetchYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the CustomController.yaml file

	// Add the "CustomController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_MpcController(nodeHandle, "MpcController");

	// > The mass of the crazyflie
	cf_mass = getParameterFloat(nodeHandle_for_MpcController , "mass");

	// Display one of the YAML parameters to debug if it is working correctly
	//ROS_INFO_STREAM("DEBUGGING: mass leaded from loacl file = " << cf_mass );

	// > The frequency at which the "computeControlOutput" is being called, as determined
	//   by the frequency at which the Vicon system provides position and attitude data
	control_frequency = getParameterFloat(nodeHandle_for_MpcController, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit motor command to
	//   thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_MpcController, "motorPoly", motorPoly, 3);


	// DEBUGGING: Print out one of the parameters that was loaded
	ROS_INFO_STREAM("[MPC CONTROLLER] DEBUGGING: the fetched CustomController/mass = " << cf_mass);

    getParameterFloatVector(nodeHandle_for_MpcController, "gainMatrixRollRate_Nested",             gainMatrixRollRate_Nested,             3);
	getParameterFloatVector(nodeHandle_for_MpcController, "gainMatrixPitchRate_Nested",            gainMatrixPitchRate_Nested,            3);
	getParameterFloatVector(nodeHandle_for_MpcController, "gainMatrixYawRate_Nested",              gainMatrixYawRate_Nested,              3);


    // THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
	// > For the (x,y,z) position
	getParameterFloatVector(nodeHandle_for_MpcController, "PMKF_Ahat_row1_for_positions",  PMKF_Ahat_row1_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_MpcController, "PMKF_Ahat_row2_for_positions",  PMKF_Ahat_row2_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_MpcController, "PMKF_Kinf_for_positions"     ,  PMKF_Kinf_for_positions     ,  2);
	// > For the (roll,pitch,yaw) angles
	getParameterFloatVector(nodeHandle_for_MpcController, "PMKF_Ahat_row1_for_angles",  PMKF_Ahat_row1_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_MpcController, "PMKF_Ahat_row2_for_angles",  PMKF_Ahat_row2_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_MpcController, "PMKF_Kinf_for_angles"     ,  PMKF_Kinf_for_angles     ,  2);


	// Call the function that computes details an values that are needed from these
	// parameters loaded above
	processFetchedParameters();
}


// This function CAN BE edited for successful completion of the classroom exercise, and the
// use of parameters loaded from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void processFetchedParameters()
{
    // Compute the feed-forward force that we need to counteract gravity (i.e., mg)
    // > in units of [Newtons]
    gravity_force = cf_mass * 9.81/(1000*4);
    // DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[MPC CONTROLLER] DEBUGGING: thus the graity force = " << gravity_force);
}



//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------


// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}
// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}
// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val.size();
}
// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
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

// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
int main(int argc, char* argv[]) {
    
    // Starting the ROS-node
    ros::init(argc, argv, "MpcControllerService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the namespace of this "StudentControllerService" node
    std::string m_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[MPC CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);

    // Get the agent ID as the "ROS_NAMESPACE" this computer.
    // NOTES:
    // > If you look at the "Student.launch" file in the "launch" folder, you will see
    //   the following line of code:
    //   <param name="studentID" value="$(optenv ROS_NAMESPACE)" />
    //   This line of code adds a parameter named "studentID" to the "FlyingAgentClient"
    // > Thus, to get access to this "studentID" paremeter, we first need to get a handle
    //   to the "FlyingAgentClient" node within which this controller service is nested.
    // Get the handle to the "FlyingAgentClient" node
    ros::NodeHandle FlyingAgentClient_nodeHandle(m_namespace + "/FlyingAgentClient");
    // Get the value of the "studentID" parameter into the instance variable "my_agentID"
    if(!FlyingAgentClient_nodeHandle.getParam("agentID", my_agentID))
    {
    	// Throw an error if the student ID parameter could not be obtained
		ROS_ERROR("[MPC CONTROLLER] Failed to get agentID from FlyingAgentClient");
	}


	// *********************************************************************************
	// EVERYTHING THAT RELATES TO FETCHING PARAMETERS FROM A YAML FILE


	// EVERYTHING FOR THE CONNECTION TO THIS AGENT's OWN PARAMETER SERVICE:

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
    ROS_INFO_STREAM("[MPC CONTROLLER] The namespace string for accessing the Paramter Services are:");
    ROS_INFO_STREAM("[MPC CONTROLLER] namespace_to_own_agent_parameter_service    =  " << namespace_to_own_agent_parameter_service);
    ROS_INFO_STREAM("[MPC CONTROLLER] namespace_to_coordinator_parameter_service  =  " << namespace_to_coordinator_parameter_service);


    // FINALLY, FETCH ANY PARAMETERS REQUIRED FROM THESE "PARAMETER SERVICES"

	// Call the class function that loads the parameters for this class.
    //fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);

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
    ros::ServiceServer service = nodeHandle.advertiseService("MpcController", calculateControlOutput);

    // Create a "ros::NodeHandle" type local variable "namespace_nodeHandle" that points
    // to the name space of this node, i.e., "dfall_pkg" as specified by the line:
    //     "using namespace dfall_pkg;"
    // in the "DEFINES" section at the top of this file.
    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());

    // Print out some information to the user.
    ROS_INFO("[MPC CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
