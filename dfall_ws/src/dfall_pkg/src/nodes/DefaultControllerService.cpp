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
//    The fall-back controller
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/DefaultControllerService.h"






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

// This function is the callback that is linked to the "DefaultController" service that
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
// > The allowed values for "onboardControllerType" are in the "Defines"
//   section in the header file, they are:
//   - CF_COMMAND_TYPE_MOTORS
//   - CF_COMMAND_TYPE_RATE
//   - CF_COMMAND_TYPE_ANGLE
//
// > For completeing the class exercises it is only required to use
//   the CF_COMMAND_TYPE_RATE option.
//
// > For the CF_COMMAND_TYPE_RATE optoin:
//   1) the ".roll", ".ptich", and ".yaw" properties of
//      "response.ControlCommand" specify the angular rate in
//      [radians/second] that will be requested from the PID controllers
//      running in the Crazyflie 2.0 firmware.
//   2) the ".motorCmd1" to ".motorCmd4" properties of
//      "response.ControlCommand" are the baseline motor commands
//      requested from the Crazyflie, with the adjustment for body rates
//      being added on top of this in the firmware (i.e., as per the
//      code of the "distribute_power" found in the firmware).
//   3) the axis convention for the roll, pitch, and yaw body rates
//      returned in "response.ControlCommand" should use right-hand
//      coordinate axes with x-forward and z-upwards (i.e., the positive
//      z-axis is aligned with the direction of positive thrust). To
//      assist, the following is an ASCII art of this convention.
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
bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// This is the "start" of the outer loop controller, add all your control
	// computation here, or you may find it convienient to create functions
	// to keep you code cleaner


	// Define a local array to fill in with the state error
	float stateErrorInertial[9];

	// Fill in the (x,y,z) position error
	stateErrorInertial[0] = request.ownCrazyflie.x - m_setpoint[0];
	stateErrorInertial[1] = request.ownCrazyflie.y - m_setpoint[1];
	stateErrorInertial[2] = request.ownCrazyflie.z - m_setpoint[2];

	// Compute an estimate of the velocity
	// > As simply the derivative between the current and previous position
	stateErrorInertial[3] = (stateErrorInertial[0] - m_previous_stateErrorInertial[0]) * yaml_control_frequency;
	stateErrorInertial[4] = (stateErrorInertial[1] - m_previous_stateErrorInertial[1]) * yaml_control_frequency;
	stateErrorInertial[5] = (stateErrorInertial[2] - m_previous_stateErrorInertial[2]) * yaml_control_frequency;

	// Fill in the roll and pitch angle measurements directly
	stateErrorInertial[6] = request.ownCrazyflie.roll;
	stateErrorInertial[7] = request.ownCrazyflie.pitch;

	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > First, get the yaw error into a local variable
	float yawError = request.ownCrazyflie.yaw - m_setpoint[3];
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


	// SAVE THE STATE ERROR TO BE USED NEXT TIME THIS FUNCTION IS CALLED
	// > as we have already used previous error we can now update it update it
	for(int i = 0; i < 9; ++i)
	{
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];
	}



	
	// YAW CONTROLLER

	// Perform the "-Kx" LQR computation for the yaw rate
	// to respond with
	float yawRate_forResponse = 0;
	for(int i = 0; i < 9; ++i)
	{
		yawRate_forResponse -= m_gainMatrixYawRate[i] * stateErrorBody[i];
	}
	// Put the computed yaw rate into the "response" variable
	response.controlOutput.yaw = yawRate_forResponse;




	// ALITUDE CONTROLLER (i.e., z-controller)
	
	// Perform the "-Kx" LQR computation for the thrust adjustment
	// to use for computing the response with
	float thrustAdjustment = 0;
	for(int i = 0; i < 9; ++i)
	{
		thrustAdjustment -= m_gainMatrixThrust[i] * stateErrorBody[i];
	}

	// Add the feed-forward thrust before putting in the response
	float feed_forward_thrust_per_motor = m_cf_weight_in_newtons / 4.0;
	float thrust_per_motor = thrustAdjustment + feed_forward_thrust_per_motor;

	// > NOTE: the function "computeMotorPolyBackward" converts the
	//         input argument from Newtons to the 16-bit command
	//         expected by the Crazyflie.
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_per_motor);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_per_motor);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_per_motor);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_per_motor);

	
	// BODY FRAME X CONTROLLER

	// Perform the "-Kx" LQR computation for the pitch rate
	// to respoond with
	float pitchRate_forResponse = 0;
	for(int i = 0; i < 9; ++i)
	{
		pitchRate_forResponse -= m_gainMatrixPitchRate[i] * stateErrorBody[i];
	}
	// Put the computed pitch rate into the "response" variable
	response.controlOutput.pitch = pitchRate_forResponse;




	// BODY FRAME Y CONTROLLER

	// Instantiate the local variable for the roll rate that will be requested
	// from the Crazyflie's on-baord "inner-loop" controller
	

	// Perform the "-Kx" LQR computation for the roll rate
	// to respoond with
	float rollRate_forResponse = 0;
	for(int i = 0; i < 9; ++i)
	{
		rollRate_forResponse -= m_gainMatrixRollRate[i] * stateErrorBody[i];
	}
	// Put the computed roll rate into the "response" variable
	response.controlOutput.roll = rollRate_forResponse;

	
	
	// PREPARE AND RETURN THE VARIABLE "response"

	/*choosing the Crazyflie onBoard controller type.
	it can either be Motor, Rate or Angle based */
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;





	//  ***********************************************************
	//  DDDD   EEEEE  BBBB   U   U   GGGG       M   M   SSSS   GGGG
	//  D   D  E      B   B  U   U  G           MM MM  S      G
	//  D   D  EEE    BBBB   U   U  G           M M M   SSS   G
	//  D   D  E      B   B  U   U  G   G       M   M      S  G   G
	//  DDDD   EEEEE  BBBB    UUU    GGGG       M   M  SSSS    GGGG

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
	m_debugPublisher.publish(debugMsg);

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
	// ROS_INFO_STREAM("motorPoly 0:" << yaml_motorPoly[0]);
	// ROS_INFO_STREAM("motorPoly 1:" << yaml_motorPoly[1]);
	// ROS_INFO_STREAM("motorPoly 2:" << yaml_motorPoly[2]);

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
// stateInertial
// This is an array of length 9 with the estimates the error of of the following values
// relative to the sepcifed setpoint:
//     stateInertial[0]    x position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[1]    y position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[2]    z position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[3]    x-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[4]    y-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[5]    z-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[6]    The roll  component of the intrinsic Euler angles [radians]
//     stateInertial[7]    The pitch component of the intrinsic Euler angles [radians]
//     stateInertial[8]    The yaw   component of the intrinsic Euler angles [radians]
// 
// stateBody
// This is an empty array of length 9, this function should fill in all elements of this
// array with the same ordering as for the "stateInertial" argument, expect that the (x,y)
// position and (x,y) velocities are rotated into the body frame.
//
// yaw_measured
// This is the yaw component of the intrinsic Euler angles in [radians] as measured by
// the Vicon motion capture system
//
// This function WILL NEED TO BE edited for successful completion of the PPS exercise
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured)
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

// This function DOES NEED TO BE edited for successful completion of
// the exercise
float computeMotorPolyBackward(float thrust)
{
	// Compute the 16-but command that would produce the requested
	// "thrust" based on the quadratic mapping that is described
	// by the coefficients in the "yaml_motorPoly" variable.
	float cmd_16bit = (-yaml_motorPoly[1] + sqrt(yaml_motorPoly[1] * yaml_motorPoly[1] - 4 * yaml_motorPoly[2] * (yaml_motorPoly[0] - thrust))) / (2 * yaml_motorPoly[2]);

	// Saturate the signal to be 0 or in the range [1000,65000]
	if (cmd_16bit < yaml_command_sixteenbit_min)
	{
		cmd_16bit = 0;
	}
	else if (cmd_16bit > yaml_command_sixteenbit_max)
	{
		cmd_16bit = yaml_command_sixteenbit_max;
	}

	// Return the result
	return cmd_16bit;
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Check if the request if for the default setpoint
		if (newSetpoint.buttonID == REQUEST_DEFAULT_SETPOINT_BUTTON_ID)
		{
			setNewSetpoint(
					yaml_default_setpoint[0],
					yaml_default_setpoint[1],
					yaml_default_setpoint[2],
					yaml_default_setpoint[3]
				);
		}
		else
		{
			// Call the function for actually setting the setpoint
			setNewSetpoint(
					newSetpoint.x,
					newSetpoint.y,
					newSetpoint.z,
					newSetpoint.yaw
				);
		}
	}
}



// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw)
{
	// Put the new setpoint into the class variable
	m_setpoint[0] = x;
	m_setpoint[1] = y;
	m_setpoint[2] = z;
	m_setpoint[3] = yaw;

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.x   = x;
	msg.y   = y;
	msg.z   = z;
	msg.yaw = yaw;
	// Publish the message
	m_setpointChangedPublisher.publish(msg);
}


// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response)
{
	// Directly put the current setpoint into the response
	response.setpointWithHeader.x   = m_setpoint[0];
	response.setpointWithHeader.y   = m_setpoint[1];
	response.setpointWithHeader.z   = m_setpoint[2];
	response.setpointWithHeader.yaw = m_setpoint[3];
	// Return
	return true;
}





//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D
//    C      O   O  M M M  M M M  A   A  N N N  D   D
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD
//    ----------------------------------------------------------------------------------

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , commandReceived.shouldCheckForAgentID , commandReceived.agentIDs );

	if (isRevelant)
	{
		// Extract the data from the message
		int custom_button_index   = commandReceived.button_index;
		float custom_command_code = commandReceived.float_data;

		// Switch between the button pressed
		switch(custom_button_index)
		{

			// > FOR CUSTOM BUTTON 1
			case 1:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO("[DEFAULT CONTROLLER] Button 1 received in controller.");
				// Code here to respond to custom button 1
				
				break;
			}

			// > FOR CUSTOM BUTTON 2
			case 2:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO("[DEFAULT CONTROLLER] Button 2 received in controller.");
				// Code here to respond to custom button 2

				break;
			}

			// > FOR CUSTOM BUTTON 3
			case 3:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEFAULT CONTROLLER] Button 3 received in controller, with command code:" << custom_command_code );
				// Code here to respond to custom button 3

				break;
			}

			default:
			{
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[DEMO CONTROLLER] A button clicked command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.command_code = " << custom_command_code );
				break;
			}
		}
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


// LOADING OF YAML PARAMETERS
void isReadyDefaultControllerYamlCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the data
		int parameter_service_to_load_from = msg.data;
		// Initialise a local variable for the namespace
		std::string namespace_to_use;
		// Load from the respective parameter service
		switch(parameter_service_to_load_from)
		{
			// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
			case LOAD_YAML_FROM_AGENT:
			{
				ROS_INFO("[DEFAULT CONTROLLER] Now fetching the DefaultController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[DEFAULT CONTROLLER] Now fetching the DefaultController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[DEFAULT CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchDefaultControllerYamlParameters(nodeHandle_to_use);
	}
}


// LOADING OF YAML PARAMETERS
void fetchDefaultControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// DefaultController.yaml

	// Add the "DefaultController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "DefaultController");



	// GET THE PARAMETERS:

	// The mass of the crazyflie, in [grams]
	yaml_cf_mass_in_grams = getParameterFloat(nodeHandle_for_paramaters , "mass");

	// > The frequency at which the "computeControlOutput" function
	//   is being called, as determined by the frequency at which
	//   the Motion Caption (Vicon) system provides pose data, i.e.,
	//   measurement of (x,y,z) position and (roll,pitch,yaw) attitude.
	yaml_control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit
	//   motor command to thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// The min and max for saturating 16 bit thrust commands
	yaml_command_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_command_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");

	// The default setpoint, the ordering is (x,y,z,yaw),
	// with unit [meters,meters,meters,radians]
	getParameterFloatVector(nodeHandle_for_paramaters, "default_setpoint", yaml_default_setpoint, 4);



	// > DEBUGGING: Print out one of the parameters that was loaded to
	//   debug if the fetching of parameters worked correctly
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] DEBUGGING: the fetched DefaultController/mass = " << yaml_cf_mass_in_grams);



	// PROCESS THE PARAMTERS

	// > Compute the feed-forward force that we need to counteract
	//   gravity (i.e., mg) in units of [Newtons]
	m_cf_weight_in_newtons = yaml_cf_mass_in_grams * 9.81/1000.0;

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << m_cf_weight_in_newtons);
}





//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------


int main(int argc, char* argv[]) {

	// Starting the ROS-node
	ros::init(argc, argv, "DefaultControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "DefaultControllerService" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "PPSClient" node.
	// > Thus, to get access to this "agentID" paremeter, we first
	//   need to get a handle to the "PPSClient" node within which this
	//   controller service is nested.


	// Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/PPSClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[DEFAULT CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[DEFAULT CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}



	// PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

	// NOTES:
	// > The parameters that are specified thorugh the *.yaml files
	//   are managed by a separate node called the "Parameter Service"
	// > A separate node is used for reasons of speed and generality
	// > To allow for a distirbuted architecture, there are two
	//   "ParamterService" nodes that are relevant:
	//   1) the one that is nested under the "m_agentID" namespace
	//   2) the one that is nested under the "m_coordID" namespace
	// > The following lines of code create the namespace (as strings)
	//   to there two relevant "ParameterService" nodes.
	// > The node handles are also created because they are needed
	//   for the ROS Subscriptions that follow.

	// Set the class variable "m_namespace_to_own_agent_parameter_service",
	// i.e., the namespace of parameter service for this agent
	m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[DEFAULT CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "DefaultController", 1, isReadyDefaultControllerYamlCallback);
	ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("DefaultController", 1, isReadyDefaultControllerYamlCallback);



	// GIVE YAML VARIABLES AN INITIAL VALUE

	// This can be done either here or as part of declaring the
	// variables in the header file



	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// The yaml files for the controllers are not added to
	// "Parameter Service" as part of launching.
	// The process for loading the yaml parameters is to send a
	// service call containing the filename of the *.yaml file,
	// and then a message will be received on the above subscribers
	// when the paramters are ready.
	// > NOTE IMPORTANTLY that by using a serice client
	//   we stall the availability of this node until the
	//   paramter service is ready

	// Create the service client as a local variable
	ros::ServiceClient requestLoadYamlFilenameServiceClient = nodeHandle_to_own_agent_parameter_service.serviceClient<LoadYamlFromFilename>("requestLoadYamlFilename", false);
	// Create the service call as a local variable
	LoadYamlFromFilename loadYamlFromFilenameCall;
	// Specify the Yaml filename as a string
	loadYamlFromFilenameCall.request.stringWithHeader.data = "DefaultController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyDefaultControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
	// Inform the user
		ROS_ERROR("[DEFAULT CONTROLLER] The request load yaml file service call failed.");
	}



	// PUBLISHERS AND SUBSCRIBERS

	// Instantiate the class variable "m_debugPublisher" to be a
	// "ros::Publisher". This variable advertises under the name
	// "DebugTopic" and is a message with the structure defined
	//  in the file "DebugMsg.msg" (located in the "msg" folder).
	m_debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

	// Instantiate the local variable "requestSetpointChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestSetpointChange" topic and calls the class function
	// "requestSetpointChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the setpoint is changed
	// by the user.
	ros::Subscriber requestSetpointChangeSubscriber = nodeHandle.subscribe("RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	std::string namespace_to_coordinator;
	constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("DefaultControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Instantiate the class variable "m_setpointChangedPublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "SetpointChanged" and is a message with the structure defined
	// in the file "SetpointWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// field that displays the current setpoint for this controller.
	m_setpointChangedPublisher = nodeHandle.advertise<SetpointWithHeader>("SetpointChanged", 1);

	// Instantiate the local variable "getCurrentSetpointService" to be
	// a "ros::ServiceServer" type variable that advertises the service
	// called "GetCurrentSetpoint". This service has the input-output
	// behaviour defined in the "GetSetpointService.srv" file (located
	// in the "srv" folder). This service, when called, is provided with
	// an integer (that is essentially ignored), and is expected to respond
	// with the current setpoint of the controller. When a request is made
	// of this service the "getCurrentSetpointCallback" function is called.
	ros::ServiceServer getCurrentSetpointService = nodeHandle.advertiseService("GetCurrentSetpoint", getCurrentSetpointCallback);	



	// Instantiate the local variable "service" to be a "ros::ServiceServer" type
	// variable that advertises the service called "DefaultController". This service has
	// the input-output behaviour defined in the "Controller.srv" file (located in the
	// "srv" folder). This service, when called, is provided with the most recent
	// measurement of the Crazyflie and is expected to respond with the control action
	// that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
	// this is where the "outer loop" controller function starts. When a request is made
	// of this service the "calculateControlOutput" function is called.
	ros::ServiceServer service = nodeHandle.advertiseService("DefaultController", calculateControlOutput);

	// Instantiate the local variable "customCommandSubscriber" to be a "ros::Subscriber"
	// type variable that subscribes to the "GUIButton" topic and calls the class
	// function "customCommandReceivedCallback" each time a messaged is received on this topic
	// and the message received is passed as an input argument to the callback function.
	ros::Subscriber customCommandReceivedSubscriber = nodeHandle.subscribe("CustomButtonPressed", 1, customCommandReceivedCallback);

	// Create a "ros::NodeHandle" type local variable "namespace_nodeHandle" that points
	// to the name space of this node, i.e., "dfall_pkg" as specified by the line:
	//     "using namespace dfall_pkg;"
	// in the "DEFINES" section at the top of this file.
	//ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());

	// Print out some information to the user.
	ROS_INFO("[DEFAULT CONTROLLER] Service ready :-)");

	// Enter an endless while loop to keep the node alive.
	ros::spin();

	// Return zero if the "ross::spin" is cancelled.
	return 0;
}
