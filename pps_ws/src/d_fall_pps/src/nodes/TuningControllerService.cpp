//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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





// INCLUDE THE HEADER
#include "nodes/TuningControllerService.h"





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



bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// THIS IS THE START OF THE "OUTER" CONTROL LOOP
	// > i.e., this is the control loop run on this laptop
	// > this function is called at the frequency specified
	// > this function performs all estimation and control


	// PERFORM THE ESTIMATOR UPDATE FOR THE INTERIAL FRAME STATE
	// > After this function is complete the class variable
	//   "current_stateInertialEstimate" is updated and ready
	//   to be used for subsequent controller copmutations
	performEstimatorUpdate_forStateInterial(request);

	
	// CONVERT THE CURRENT INERTIAL FRAME STATE ESTIMATE, INTO
	// THE BODY FRAME ERROR REQUIRED BY THE CONTROLLER
	// > Define a local array to fill in with the body frame error
	float current_bodyFrameError[12];
	// > Call the function to perform the conversion
	convert_stateInertial_to_bodyFrameError(current_stateInertialEstimate,setpoint,current_bodyFrameError);

	
	// CARRY OUT THE CONTROLLER COMPUTATIONS
	calculateControlOutput_viaLQR(current_bodyFrameError,request,response);


	// PUBLISH THE DEBUG MESSAGE (if required)
	if (shouldPublishDebugMessage)
	{
		construct_and_publish_debug_message(request,response);
	}

    // Return "true" to indicate that the control computation was performed successfully
    return true;
}





//    ----------------------------------------------------------------------------------
//    RRRR   EEEEE  M   M   OOO   TTTTT  EEEEE
//    R   R  E      MM MM  O   O    T    E
//    RRRR   EEE    M M M  O   O    T    EEE
//    R  R   E      M   M  O   O    T    E
//    R   R  EEEEE  M   M   OOO     T    EEEEE
//    
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L
//    C      O   O  NN  N    T    R   R  O   O  L
//    C      O   O  N N N    T    RRRR   O   O  L
//    C      O   O  N  NN    T    R  R   O   O  L
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL
//    ----------------------------------------------------------------------------------


// ACTIVATE THE TESTS
void activateTestCallback(const std_msgs::Int32& msg)
{

	// Get the test index from the message
	int test_index = msg.data;

	switch (test_index)
	{
		// Test the HORIZONTAL controller
		case 1:
		{
			switch (test_horizontal_currentpoint)
			{
				// Currently at setpoint 1 => change to setpoint 2
				case 1:
				{
					setpoint[0] = test_horizontal_setpoint2[0];
					setpoint[1] = test_horizontal_setpoint2[1];
					setpoint[2] = test_horizontal_setpoint2[2];
					setpoint[3] = test_horizontal_setpoint2[3];
					test_horizontal_currentpoint = 2;
					break;
				}
				// Currently at setpoint 2 => change to setpoint 1
				case 2:
				{
					setpoint[0] = test_horizontal_setpoint1[0];
					setpoint[1] = test_horizontal_setpoint1[1];
					setpoint[2] = test_horizontal_setpoint1[2];
					setpoint[3] = test_horizontal_setpoint1[3];
					test_horizontal_currentpoint = 1;	
					break;
				}
				// Handle the exception
				default:
				{
					// Display that the "estimator_method" was not recognised
					ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'activateTestCallback' function of the 'TuningControllerService': the 'test_horizontal_currentpoint' is not recognised.");
					break;
				}
			}
			break;
		}


		// Test the VERTICAL controller
		case 2:
		{
			switch (test_vertical_currentpoint)
			{
				// Currently at setpoint 1 => change to setpoint 2
				case 1:
				{
					setpoint[0] = test_vertical_setpoint2[0];
					setpoint[1] = test_vertical_setpoint2[1];
					setpoint[2] = test_vertical_setpoint2[2];
					setpoint[3] = test_vertical_setpoint2[3];
					test_vertical_currentpoint = 2;
					break;
				}
				// Currently at setpoint 2 => change to setpoint 1
				case 2:
				{
					setpoint[0] = test_vertical_setpoint1[0];
					setpoint[1] = test_vertical_setpoint1[1];
					setpoint[2] = test_vertical_setpoint1[2];
					setpoint[3] = test_vertical_setpoint1[3];
					test_vertical_currentpoint = 1;	
					break;
				}
				// Handle the exception
				default:
				{
					// Display that the "estimator_method" was not recognised
					ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'activateTestCallback' function of the 'TuningControllerService': the 'test_vertical_currentpoint' is not recognised.");
					break;
				}
			}
			break;
		}


		// Test the HEADING controller
		case 3:
		{
			switch (test_heading_currentpoint)
			{
				// Currently at setpoint 1 => change to setpoint 2
				case 1:
				{
					setpoint[0] = test_heading_setpoint2[0];
					setpoint[1] = test_heading_setpoint2[1];
					setpoint[2] = test_heading_setpoint2[2];
					setpoint[3] = test_heading_setpoint2[3];
					test_heading_currentpoint = 2;
					break;
				}
				// Currently at setpoint 2 => change to setpoint 1
				case 2:
				{
					setpoint[0] = test_heading_setpoint1[0];
					setpoint[1] = test_heading_setpoint1[1];
					setpoint[2] = test_heading_setpoint1[2];
					setpoint[3] = test_heading_setpoint1[3];
					test_heading_currentpoint = 1;	
					break;
				}
				// Handle the exception
				default:
				{
					// Display that the "estimator_method" was not recognised
					ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'activateTestCallback' function of the 'TuningControllerService': the 'test_heading_currentpoint' is not recognised.");
					break;
				}
			}
			break;
		}


		// Test ALL the controllers
		case 4:
		{
			switch (test_all_currentpoint)
			{
				// Currently at setpoint 1 => change to setpoint 2
				case 1:
				{
					setpoint[0] = test_all_setpoint2[0];
					setpoint[1] = test_all_setpoint2[1];
					setpoint[2] = test_all_setpoint2[2];
					setpoint[3] = test_all_setpoint2[3];
					test_all_currentpoint = 2;
					break;
				}
				// Currently at setpoint 2 => change to setpoint 1
				case 2:
				{
					setpoint[0] = test_all_setpoint1[0];
					setpoint[1] = test_all_setpoint1[1];
					setpoint[2] = test_all_setpoint1[2];
					setpoint[3] = test_all_setpoint1[3];
					test_all_currentpoint = 1;	
					break;
				}
				// Handle the exception
				default:
				{
					// Display that the "estimator_method" was not recognised
					ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'activateTestCallback' function of the 'TuningControllerService': the 'test_all_currentpoint' is not recognised.");
					break;
				}
			}
			break;
		}


		// Handle the exception
		default:
		{
			// Display that the "estimator_method" was not recognised
			ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'activateTestCallback' function of the 'TuningControllerService': the 'test_index' is not recognised.");
			break;
		}
	}
}

// CHANGE THE GAIN FOR THE HORIZONTAL CONTROLLER
void horizontalGainCallback(const std_msgs::Int32& msg)
{

}

// CHANGE THE GAIN FOR THE VERTICAL CONTROLLER
void verticalGainCallback(const std_msgs::Int32& msg)
{

}

// CHANGE THE GAIN FOR THE HEADING CONTROLLER
void headingGainCallback(const std_msgs::Int32& msg)
{

}









//    ------------------------------------------------------------------------------
//    EEEEE   SSSS  TTTTT  III  M   M    A    TTTTT  III   OOO   N   N
//    E      S        T     I   MM MM   A A     T     I   O   O  NN  N
//    EEE     SSS     T     I   M M M  A   A    T     I   O   O  N N N
//    E          S    T     I   M   M  AAAAA    T     I   O   O  N  NN
//    EEEEE  SSSS     T    III  M   M  A   A    T    III   OOO   N   N
//    ------------------------------------------------------------------------------
void performEstimatorUpdate_forStateInterial(Controller::Request &request)
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


	// RUN THE FINITE DIFFERENCE ESTIMATOR
	performEstimatorUpdate_forStateInterial_viaFiniteDifference();


	// RUN THE POINT MASS KALMAN FILTER ESTIMATOR
	performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter();


	// FILLE IN THE STATE INERTIAL ESTIMATE TO BE USED FOR CONTROL
	switch (estimator_method)
	{
		// Estimator based on finte differences
		case ESTIMATOR_METHOD_FINITE_DIFFERENCE:
		{
			// Transfer the estimate
			for(int i = 0; i < 12; ++i)
			{
				current_stateInertialEstimate[i]  = stateInterialEstimate_viaFiniteDifference[i];
			}
			break;
		}
		// Estimator based on Point Mass Kalman Filter
		case ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION:
		{
			// Transfer the estimate
			for(int i = 0; i < 12; ++i)
			{
				current_stateInertialEstimate[i]  = stateInterialEstimate_viaPointMassKalmanFilter[i];
			}
			break;
		}
		// Handle the exception
		default:
		{
			// Display that the "estimator_method" was not recognised
			ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'calculateControlOutput' function of the 'TuningControllerService': the 'estimator_method' is not recognised.");
			// Transfer the finite difference estimate by default
			for(int i = 0; i < 12; ++i)
			{
				current_stateInertialEstimate[i]  = stateInterialEstimate_viaFiniteDifference[i];
			}
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



void performEstimatorUpdate_forStateInterial_viaFiniteDifference()
{
	// PUT IN THE CURRENT MEASUREMENT DIRECTLY
	// > for (x,y,z) position
	stateInterialEstimate_viaFiniteDifference[0]  = current_xzy_rpy_measurement[0];
	stateInterialEstimate_viaFiniteDifference[1]  = current_xzy_rpy_measurement[1];
	stateInterialEstimate_viaFiniteDifference[2]  = current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	stateInterialEstimate_viaFiniteDifference[6]  = current_xzy_rpy_measurement[3];
	stateInterialEstimate_viaFiniteDifference[7]  = current_xzy_rpy_measurement[4];
	stateInterialEstimate_viaFiniteDifference[8]  = current_xzy_rpy_measurement[5];

	// COMPUTE THE VELOCITIES VIA FINITE DIFFERENCE
	// > for (x,y,z) velocities
	stateInterialEstimate_viaFiniteDifference[3]  = (current_xzy_rpy_measurement[0] - previous_xzy_rpy_measurement[0]) * estimator_frequency;
	stateInterialEstimate_viaFiniteDifference[4]  = (current_xzy_rpy_measurement[1] - previous_xzy_rpy_measurement[1]) * estimator_frequency;
	stateInterialEstimate_viaFiniteDifference[5]  = (current_xzy_rpy_measurement[2] - previous_xzy_rpy_measurement[2]) * estimator_frequency;
	// > for (roll,pitch,yaw) velocities
	stateInterialEstimate_viaFiniteDifference[9]  = (current_xzy_rpy_measurement[3] - previous_xzy_rpy_measurement[3]) * estimator_frequency;
	stateInterialEstimate_viaFiniteDifference[10] = (current_xzy_rpy_measurement[4] - previous_xzy_rpy_measurement[4]) * estimator_frequency;
	stateInterialEstimate_viaFiniteDifference[11] = (current_xzy_rpy_measurement[5] - previous_xzy_rpy_measurement[5]) * estimator_frequency;
}



void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter()
{
	// PERFORM THE KALMAN FILTER UPDATE STEP
	// > First take a copy of the estimator state
	float temp_PMKFstate[12];
	for(int i = 0; i < 12; ++i)
	{
		temp_PMKFstate[i]  = stateInterialEstimate_viaPointMassKalmanFilter[i];
	}
	// > Now perform update for:
	// > x position and velocity:
	stateInterialEstimate_viaPointMassKalmanFilter[0] = PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[0] + PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[3] + PMKF_Kinf_for_positions[0]*current_xzy_rpy_measurement[0];
	stateInterialEstimate_viaPointMassKalmanFilter[3] = PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[0] + PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[3] + PMKF_Kinf_for_positions[1]*current_xzy_rpy_measurement[0];
	// > y position and velocity:
	stateInterialEstimate_viaPointMassKalmanFilter[1] = PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[1] + PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[4] + PMKF_Kinf_for_positions[0]*current_xzy_rpy_measurement[1];
	stateInterialEstimate_viaPointMassKalmanFilter[4] = PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[1] + PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[4] + PMKF_Kinf_for_positions[1]*current_xzy_rpy_measurement[1];
	// > z position and velocity:
	stateInterialEstimate_viaPointMassKalmanFilter[2] = PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[2] + PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[5] + PMKF_Kinf_for_positions[0]*current_xzy_rpy_measurement[2];
	stateInterialEstimate_viaPointMassKalmanFilter[5] = PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[2] + PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[5] + PMKF_Kinf_for_positions[1]*current_xzy_rpy_measurement[2];

	// > roll  position and velocity:
	stateInterialEstimate_viaPointMassKalmanFilter[6]  = PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[6] + PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[9]  + PMKF_Kinf_for_angles[0]*current_xzy_rpy_measurement[3];
	stateInterialEstimate_viaPointMassKalmanFilter[9]  = PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[6] + PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[9]  + PMKF_Kinf_for_angles[1]*current_xzy_rpy_measurement[3];
	// > pitch position and velocity:
	stateInterialEstimate_viaPointMassKalmanFilter[7]  = PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[7] + PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[10] + PMKF_Kinf_for_angles[0]*current_xzy_rpy_measurement[4];
	stateInterialEstimate_viaPointMassKalmanFilter[10] = PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[7] + PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[10] + PMKF_Kinf_for_angles[1]*current_xzy_rpy_measurement[4];
	// > yaw   position and velocity:
	stateInterialEstimate_viaPointMassKalmanFilter[8]  = PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[8] + PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[11] + PMKF_Kinf_for_angles[0]*current_xzy_rpy_measurement[5];
	stateInterialEstimate_viaPointMassKalmanFilter[11] = PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[8] + PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[11] + PMKF_Kinf_for_angles[1]*current_xzy_rpy_measurement[5];
}




//    ----------------------------------------------------------------------------------
//    L       QQQ   RRRR
//    L      Q   Q  R   R
//    L      Q   Q  RRRR
//    L      Q  Q   R  R
//    LLLLL   QQ Q  R   R
//    ----------------------------------------------------------------------------------

void calculateControlOutput_viaLQR(float stateErrorBody[12], Controller::Request &request, Controller::Response &response)
{
	// SWITCH BETWEEN THE DIFFERENT LQR CONTROLLER MODES:
	switch (controller_mode)
	{
		// LQR controller based on the state vector:
		// [position,velocity,angles]
		case CONTROLLER_MODE_LQR_RATE:
		{
			// Call the function that performs the control computations for this mode
			calculateControlOutput_viaLQRforRates(stateErrorBody,request,response);
			break;
		}

		// LQR controller based on the state vector:
		// [position,velocity]
		case CONTROLLER_MODE_LQR_ANGLE:
		{
			// Call the function that performs the control computations for this mode
			calculateControlOutput_viaLQRforAngles(stateErrorBody,request,response);
			break;
		}

		// LQR controller based on the state vector:
		// [position,velocity,angles]
		case CONTROLLER_MODE_LQR_ANGLE_RATE_NESTED:
		{
			// Call the function that performs the control computations for this mode
			calculateControlOutput_viaLQRforAnglesRatesNested(stateErrorBody,request,response);
			break;
		}

		default:
		{
			// Display that the "controller_mode" was not recognised
			ROS_INFO_STREAM("[TUNING CONTROLLER] ERROR: in the 'calculateControlOutput' function of the 'TuningControllerService': the 'controller_mode' is not recognised.");
			// Set everything in the response to zero
			response.controlOutput.roll       =  0;
			response.controlOutput.pitch      =  0;
			response.controlOutput.yaw        =  0;
			response.controlOutput.motorCmd1  =  0;
			response.controlOutput.motorCmd2  =  0;
			response.controlOutput.motorCmd3  =  0;
			response.controlOutput.motorCmd4  =  0;
			response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTOR;
			break;
		}
	}
}




void calculateControlOutput_viaLQRforRates(float stateErrorBody[12], Controller::Request &request, Controller::Response &response)
{
	// PERFORM THE "u=Kx" LQR CONTROLLER COMPUTATION

	// Instantiate the local variables for the following:
	// > body frame roll rate,
	// > body frame pitch rate,
	// > body frame yaw rate,
	// > total thrust adjustment.
	// These will be requested from the Crazyflie's on-baord "inner-loop" controller
	float rollRate_forResponse = 0;
	float pitchRate_forResponse = 0;
	float yawRate_forResponse = 0;
	float thrustAdjustment = 0;
	
	// Perform the "-Kx" LQR computation for the rates and thrust:
	for(int i = 0; i < 9; ++i)
	{
		// BODY FRAME Y CONTROLLER
		rollRate_forResponse  -= gainMatrixRollRate[i] * stateErrorBody[i];
		// BODY FRAME X CONTROLLER
		pitchRate_forResponse -= gainMatrixPitchRate[i] * stateErrorBody[i];
		// BODY FRAME YAW CONTROLLER
		yawRate_forResponse   -= gainMatrixYawRate[i] * stateErrorBody[i];
		// > ALITUDE CONTROLLER (i.e., z-controller):
		thrustAdjustment      -= gainMatrixThrust_NineStateVector[i] * stateErrorBody[i];
	}


	// UPDATE THE "RETURN" THE VARIABLE NAMED "response"

	// Put the computed rates and thrust into the "response" variable
	// > For roll, pitch, and yaw:
	response.controlOutput.roll  = rollRate_forResponse;
	response.controlOutput.pitch = pitchRate_forResponse;
	response.controlOutput.yaw   = yawRate_forResponse;
	// > For the thrust adjustment we must add the feed-forward thrust to counter-act gravity.
	// > NOTE: remember that the thrust is commanded per motor, so you sohuld
	//         consider whether the "thrustAdjustment" computed by your
	//         controller needed to be divided by 4 or not.
	thrustAdjustment = thrustAdjustment / 4.0;
	// > NOTE: the "gravity_force_quarter" value was already divided by 4 when
	//   it was loaded/processes from the .yaml file.
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);

	
	// Specify that this controller is a rate controller
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTOR;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;


	// An alternate debugging technique is to print out data directly to the
	// command line from which this node was launched.
	if (shouldDisplayDebugInfo)
	{
		// An example of "printing out" the data from the "request" argument to the
		// command line. This might be useful for debugging.
		ROS_INFO_STREAM("x-coordinate [m]: " << request.ownCrazyflie.x);
		ROS_INFO_STREAM("y-coordinate [m]: " << request.ownCrazyflie.y);
		ROS_INFO_STREAM("z-coordinate [m]: " << request.ownCrazyflie.z);
		ROS_INFO_STREAM("roll       [deg]: " << request.ownCrazyflie.roll);
		ROS_INFO_STREAM("pitch      [deg]: " << request.ownCrazyflie.pitch);
		ROS_INFO_STREAM("yaw        [deg]: " << request.ownCrazyflie.yaw);
		ROS_INFO_STREAM("Delta t      [s]: " << request.ownCrazyflie.acquiringTime);

		// An example of "printing out" the control actions computed.
		ROS_INFO_STREAM("thrustAdjustment = " << thrustAdjustment);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);

		// An example of "printing out" the "thrust-to-command" conversion parameters.
		ROS_INFO_STREAM("motorPoly 0:" << motorPoly[0]);
		ROS_INFO_STREAM("motorPoly 0:" << motorPoly[1]);
		ROS_INFO_STREAM("motorPoly 0:" << motorPoly[2]);

		// An example of "printing out" the per motor 16-bit command computed.
		ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);
	}
}




void calculateControlOutput_viaLQRforAngles(float stateErrorBody[12], Controller::Request &request, Controller::Response &response)
{
	// PERFORM THE "u=Kx" LQR CONTROLLER COMPUTATION

	// Instantiate the local variables for the following:
	// > body frame roll angle,
	// > body frame pitch angle,
	// > total thrust adjustment.
	// These will be requested from the Crazyflie's on-baord "inner-loop" controller
	float rollAngle_forResponse = 0;
	float pitchAngle_forResponse = 0;
	float thrustAdjustment = 0;

	// Perform the "-Kx" LQR computation for the rates and thrust:
	for(int i = 0; i < 6; ++i)
	{
		// BODY FRAME Y CONTROLLER
		rollAngle_forResponse -= gainMatrixRollAngle[i] * stateErrorBody[i];
		// BODY FRAME X CONTROLLER
		pitchAngle_forResponse -= gainMatrixPitchAngle[i] * stateErrorBody[i];
		// > ALITUDE CONTROLLER (i.e., z-controller):
		thrustAdjustment      -= gainMatrixThrust_SixStateVector[i] * stateErrorBody[i];
	}

	// UPDATE THE "RETURN" THE VARIABLE NAMED "response"

	// Put the computed rates and thrust into the "response" variable
	// > For roll, pitch, and yaw:
	response.controlOutput.roll  = rollAngle_forResponse;
	response.controlOutput.pitch = pitchAngle_forResponse;
	response.controlOutput.yaw   = setpoint[3];
	// > For the thrust adjustment we must add the feed-forward thrust to counter-act gravity.
	// > NOTE: remember that the thrust is commanded per motor, so you sohuld
	//         consider whether the "thrustAdjustment" computed by your
	//         controller needed to be divided by 4 or not.
	thrustAdjustment = thrustAdjustment / 4.0;
	// > NOTE: the "gravity_force_quarter" value was already divided by 4 when
	//         it was loaded/processes from the .yaml file.
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);

	
	// Specify that this controller is a rate controller
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTOR;
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;


}



void calculateControlOutput_viaLQRforAnglesRatesNested( float stateErrorBody[12], Controller::Request &request, Controller::Response &response)
{
	// PERFORM THE NESTED "u=Kx" LQR CONTROLLER COMPUTATION

	// Increment the counter variable
	lqr_angleRateNested_counter++;

	if (lqr_angleRateNested_counter > 4)
	{
		//ROS_INFO("Outer called");
			
		// Reset the counter to 1
		lqr_angleRateNested_counter = 1;

		// PERFORM THE OUTER "u=Kx" LQR CONTROLLER COMPUTATION

		// Reset the class variable to zero for the following:
		// > body frame roll angle,
		// > body frame pitch angle,
		// > body frame yaw angle,
		// > total thrust adjustment.
		// These will be requested from the "inner-loop" LQR controller below
		lqr_angleRateNested_prev_rollAngle        = 0;
		lqr_angleRateNested_prev_pitchAngle       = 0;
		lqr_angleRateNested_prev_thrustAdjustment = 0;

		// Perform the "-Kx" LQR computation for the rates and thrust:
		for(int i = 0; i < 6; ++i)
		{
			// BODY FRAME Y CONTROLLER
			lqr_angleRateNested_prev_rollAngle        -= gainMatrixRollAngle_50Hz[i] * stateErrorBody[i];
			// BODY FRAME X CONTROLLER
			lqr_angleRateNested_prev_pitchAngle       -= gainMatrixPitchAngle_50Hz[i] * stateErrorBody[i];
			// > ALITUDE CONTROLLER (i.e., z-controller):
			lqr_angleRateNested_prev_thrustAdjustment -= gainMatrixThrust_SixStateVector_50Hz[i] * stateErrorBody[i];
		}

		// BODY FRAME Z CONTROLLER
		//lqr_angleRateNested_prev_yawAngle = setpoint[3];
		lqr_angleRateNested_prev_yawAngle = stateErrorBody[8];


	}

	//ROS_INFO("Inner called");

	// PERFORM THE INNER "u=Kx" LQR CONTROLLER COMPUTATION
	// Instantiate the local variables for the following:
	// > body frame roll rate,
	// > body frame pitch rate,
	// > body frame yaw rate,
	// These will be requested from the Crazyflie's on-baord "inner-loop" controller
	float rollRate_forResponse  = 0;
	float pitchRate_forResponse = 0;
	float yawRate_forResponse   = 0;

	// Create the angle error to use for the inner controller
	float temp_stateAngleError[3] = {
		stateErrorBody[6] - lqr_angleRateNested_prev_rollAngle,
		stateErrorBody[7] - lqr_angleRateNested_prev_pitchAngle,
		lqr_angleRateNested_prev_yawAngle
	};
	
	// Perform the "-Kx" LQR computation for the rates and thrust:
	for(int i = 0; i < 4; ++i)
	{
		// BODY FRAME Y CONTROLLER
		rollRate_forResponse  -= gainMatrixRollRate_Nested[i]  * temp_stateAngleError[i];
		// BODY FRAME X CONTROLLER
		pitchRate_forResponse -= gainMatrixPitchRate_Nested[i] * temp_stateAngleError[i];
		// BODY FRAME Z CONTROLLER
		yawRate_forResponse   -= gainMatrixYawRate_Nested[i]   * temp_stateAngleError[i];
	}


	// UPDATE THE "RETURN" THE VARIABLE NAMED "response"

	// Put the computed rates and thrust into the "response" variable
	// > For roll, pitch, and yaw:
	response.controlOutput.roll  = rollRate_forResponse;
	response.controlOutput.pitch = pitchRate_forResponse;
	response.controlOutput.yaw   = yawRate_forResponse;
	// > For the thrust adjustment we must add the feed-forward thrust to counter-act gravity.
	// > NOTE: remember that the thrust is commanded per motor, so you sohuld
	//         consider whether the "thrustAdjustment" computed by your
	//         controller needed to be divided by 4 or not.
	float thrustAdjustment = lqr_angleRateNested_prev_thrustAdjustment / 4.0;
	// > NOTE: the "gravity_force_quarter" value was already divided by 4 when
	//         it was loaded/processes from the .yaml file.
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustAdjustment + gravity_force_quarter);

	
	// Specify that this controller is a rate controller
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTOR;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;

	// Display some details (if requested)
	if (shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("thrust    =" << lqr_angleRateNested_prev_thrustAdjustment );
		ROS_INFO_STREAM("rollrate  =" << response.controlOutput.roll );
		ROS_INFO_STREAM("pitchrate =" << response.controlOutput.pitch );
		ROS_INFO_STREAM("yawrate   =" << response.controlOutput.yaw );
	}
}















void construct_and_publish_debug_message(Controller::Request &request, Controller::Response &response)
{
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
	debugMsg.vicon_x      = request.ownCrazyflie.x;
	debugMsg.vicon_y      = request.ownCrazyflie.y;
	debugMsg.vicon_z      = request.ownCrazyflie.z;
	debugMsg.vicon_roll   = request.ownCrazyflie.roll;
	debugMsg.vicon_pitch  = request.ownCrazyflie.pitch;
	debugMsg.vicon_yaw    = request.ownCrazyflie.yaw;

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
//    ------------------------------------------------------------------------------

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
void convertIntoBodyFrame(float stateInertial[12], float (&stateBody)[12], float yaw_measured)
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

	    // Fill in the (roll,pitch,yaw) velocity estimates to be returned
	    stateBody[9]  = stateInertial[9];
	    stateBody[10] = stateInertial[10];
	    stateBody[11] = stateInertial[11];
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

	    // Fill in the (roll,pitch,yaw) velocity estimates to be returned
	    stateBody[9]  = stateInertial[9];
	    stateBody[10] = stateInertial[10];
	    stateBody[11] = stateInertial[11];
	}
}




void convert_stateInertial_to_bodyFrameError(float stateInertial[12], float setpoint[4], float (&bodyFrameError)[12])
{
	// Store the current YAW in a local variable
	float temp_stateInertial_yaw = stateInertial[8];

	// Adjust the INERTIAL (x,y,z) position for the setpoint
	stateInertial[0] = stateInertial[0] - setpoint[0];
	stateInertial[1] = stateInertial[1] - setpoint[1];
	stateInertial[2] = stateInertial[2] - setpoint[2];

	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > First, get the yaw error into a local variable
	float yawError = stateInertial[8] - setpoint[3];
	// > Second, "unwrap" the yaw error to the interval ( -pi , pi )
	while(yawError > PI) {yawError -= 2 * PI;}
	while(yawError < -PI) {yawError += 2 * PI;}
	// > Third, put the "yawError" into the "stateError" variable
	stateInertial[8] = yawError;


	if (yawError>(PI/6))
	{
		yawError = (PI/6);
	}
	else if (yawError<(-PI/6))
	{
		yawError = (-PI/6);
	}

	// CONVERSION INTO BODY FRAME
	// Conver the state erorr from the Inertial frame into the Body frame
	// > Note: the function "convertIntoBodyFrame" is implemented in this file
	//   and by default does not perform any conversion. The equations to convert
	//   the state error into the body frame should be implemented in that function
	//   for successful completion of the PPS exercise
	convertIntoBodyFrame(stateInertial, bodyFrameError, temp_stateInertial_yaw);
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
//    ------------------------------------------------------------------------------

// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
float computeMotorPolyBackward(float thrust)
{
	// Compute the 16-bit command signal that generates the "thrust" force
	float cmd = (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);

	// Saturate the signal to be 0 or in the range [1000,65000]
	if (cmd < cmd_sixteenbit_min)
	{
		cmd = 0;
	}
	else if (cmd > cmd_sixteenbit_max)
	{
		cmd = cmd_sixteenbit_max;
	}

    return cmd;
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
		case FETCH_YAML_TUNING_CONTROLLER_FROM_OWN_AGENT:
		{
			// Let the user know that this message was received
			ROS_INFO("[TUNING CONTROLLER] Received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from this agent.");
			// Create a node handle to the parameter service running on this agent's machine
			ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);
			// Call the function that fetches the parameters
			fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);
			break;
		}

		// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
		case FETCH_YAML_TUNING_CONTROLLER_FROM_COORDINATOR:
		{
			// Let the user know that this message was received
			ROS_INFO("[TUNING CONTROLLER] Received the message that YAML parameters were (re-)loaded. > Now fetching the parameter values from the coordinator.");
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


// This function CAN BE edited for successful completion of the PPS exercise, and the
// use of parameters fetched from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void fetchYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the CustomController.yaml file

	// Add the "CustomController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_TuningController(nodeHandle, "TuningController");


	// ******************************************************************************* //
	// PARAMETERS SPECIFIC TO THE TUNING CONTROL FEATURE

	/// Setpoint for the HORIZONTAL test
	getParameterFloatVector(nodeHandle_for_TuningController, "test_horizontal_setpoint1", test_horizontal_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_TuningController, "test_horizontal_setpoint2", test_horizontal_setpoint2, 4);

	// Setpoint for the VERTICAL test
	getParameterFloatVector(nodeHandle_for_TuningController, "test_vertical_setpoint1", test_vertical_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_TuningController, "test_vertical_setpoint2", test_vertical_setpoint2, 4);

	// Setpoint for the HEADING test
	getParameterFloatVector(nodeHandle_for_TuningController, "test_heading_setpoint1", test_heading_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_TuningController, "test_heading_setpoint2", test_heading_setpoint2, 4);

	// Setpoint for the ALL test
	getParameterFloatVector(nodeHandle_for_TuningController, "test_all_setpoint1", test_all_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_TuningController, "test_all_setpoint2", test_all_setpoint2, 4);

	// // DEBUGGING: Print out one of the parameters that was loaded
	// ROS_INFO_STREAM("[REMOTE CONTROLLER] DEBUGGING: the fetched RemoteController/default_viconObjectName_forRemote = " << default_viconObjectName_forRemote);

	// ******************************************************************************* //



	// > The mass of the crazyflie
	cf_mass = getParameterFloat(nodeHandle_for_TuningController , "mass");

	// Display one of the YAML parameters to debug if it is working correctly
	//ROS_INFO_STREAM("DEBUGGING: mass leaded from loacl file = " << cf_mass );

	// > The frequency at which the "computeControlOutput" is being called, as determined
	//   by the frequency at which the Vicon system provides position and attitude data
	vicon_frequency = getParameterFloat(nodeHandle_for_TuningController, "vicon_frequency");

	// > The frequency at which the "computeControlOutput" is being called, as determined
	//   by the frequency at which the Vicon system provides position and attitude data
	control_frequency = getParameterFloat(nodeHandle_for_TuningController, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit motor command to
	//   thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_TuningController, "motorPoly", motorPoly, 3);

	// > The boolean for whether to execute the convert into body frame function
	shouldPerformConvertIntoBodyFrame = getParameterBool(nodeHandle_for_TuningController, "shouldPerformConvertIntoBodyFrame");

	// Boolean indiciating whether the "Debug Message" of this agent should be published or not
	shouldPublishDebugMessage = getParameterBool(nodeHandle_for_TuningController, "shouldPublishDebugMessage");

	// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
	shouldDisplayDebugInfo = getParameterBool(nodeHandle_for_TuningController, "shouldDisplayDebugInfo");


	// THE CONTROLLER GAINS:

	// A flag for which controller to use:
	controller_mode = getParameterInt( nodeHandle_for_TuningController , "controller_mode" );

	// A flag for which estimator to use:
	estimator_method = getParameterInt( nodeHandle_for_TuningController , "estimator_method" );

	// The LQR Controller parameters for "LQR_MODE_RATE"
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixThrust_NineStateVector", gainMatrixThrust_NineStateVector, 9);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixRollRate",               gainMatrixRollRate,               9);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixPitchRate",              gainMatrixPitchRate,              9);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixYawRate",                gainMatrixYawRate,                9);
	
	// The LQR Controller parameters for "LQR_MODE_ANGLE"
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixThrust_SixStateVector",  gainMatrixThrust_SixStateVector,  6);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixRollAngle",              gainMatrixRollAngle,              6);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixPitchAngle",             gainMatrixPitchAngle,             6);
	
	// The LQR Controller parameters for "LQR_MODE_ANGLE_RATE_NESTED"
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixThrust_SixStateVector_50Hz",  gainMatrixThrust_SixStateVector_50Hz,  6);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixRollAngle_50Hz",              gainMatrixRollAngle_50Hz,              6);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixPitchAngle_50Hz",             gainMatrixPitchAngle_50Hz,             6);

	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixRollRate_Nested",             gainMatrixRollRate_Nested,             3);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixPitchRate_Nested",            gainMatrixPitchRate_Nested,            3);
	getParameterFloatVector(nodeHandle_for_TuningController, "gainMatrixYawRate_Nested",              gainMatrixYawRate_Nested,              3);

	
	// 16-BIT COMMAND LIMITS
	cmd_sixteenbit_min = getParameterFloat(nodeHandle_for_TuningController, "command_sixteenbit_min");
	cmd_sixteenbit_max = getParameterFloat(nodeHandle_for_TuningController, "command_sixteenbit_max");


	// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
	// > For the (x,y,z) position
	getParameterFloatVector(nodeHandle_for_TuningController, "PMKF_Ahat_row1_for_positions",  PMKF_Ahat_row1_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_TuningController, "PMKF_Ahat_row2_for_positions",  PMKF_Ahat_row2_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_TuningController, "PMKF_Kinf_for_positions"     ,  PMKF_Kinf_for_positions     ,  2);
	// > For the (roll,pitch,yaw) angles
	getParameterFloatVector(nodeHandle_for_TuningController, "PMKF_Ahat_row1_for_angles",  PMKF_Ahat_row1_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_TuningController, "PMKF_Ahat_row2_for_angles",  PMKF_Ahat_row2_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_TuningController, "PMKF_Kinf_for_angles"     ,  PMKF_Kinf_for_angles     ,  2);


	// DEBUGGING: Print out one of the parameters that was loaded
	//ROS_INFO_STREAM("[TUNING CONTROLLER] DEBUGGING: the fetched TuningController/mass = " << cf_mass);


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
	gravity_force         = cf_mass * 9.81/(1000);
	gravity_force_quarter = cf_mass * 9.81/(1000*4);

	// Set that the estimator frequency is the same as the control frequency
	estimator_frequency = vicon_frequency;


	// // Roll and pitch limit (in degrees for angles)
	// remoteControlLimit_roll  = remoteControlLimit_roll_degrees * PI / 180.0;
	// remoteControlLimit_pitch = remoteControlLimit_pitch_degrees * PI / 180.0;


	// // Use the Remote name if the variable is currently empty
	// if (viconObjectName_forRemote == "empty")
	// {
	// 	viconObjectName_forRemote = default_viconObjectName_forRemote;
	// }


	// DEBUGGING: Print out one of the computed quantities
	//ROS_INFO_STREAM("[REMOTE CONTROLLER] DEBUGGING: after processing the viconObjectName_forRemote = " << viconObjectName_forRemote);
}



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
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
std::string getParameterString(ros::NodeHandle& nodeHandle, std::string name)
{
	std::string val;
	if(!nodeHandle.getParam(name, val)){
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
    ros::init(argc, argv, "TuningControllerService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the namespace of this "StudentControllerService" node
    // > This should be something like: "/dfall/agentXXX"
    std::string m_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[TUNING CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);

    // Get the agent ID as the "ROS_NAMESPACE" this computer.
    // NOTES:
    // > If you look at the "Student.launch" file in the "launch" folder, you will see
    //   the following line of code:
    //   <param name="studentID" value="$(optenv ROS_NAMESPACE)" />
    //   This line of code adds a parameter named "studentID" to the "PPSClient"
    // > Thus, to get access to this "studentID" paremeter, we first need to get a handle
    //   to the "PPSClient" node within which this controller service is nested.
    // Get the handle to the "PPSClient" node
    ros::NodeHandle PPSClient_nodeHandle(m_namespace + "/PPSClient");
    // Get the value of the "studentID" parameter into the instance variable "my_agentID"
    if(!PPSClient_nodeHandle.getParam("agentID", my_agentID))
    {
    	// Throw an error if the student ID parameter could not be obtained
		ROS_ERROR("[TUNING CONTROLLER] Failed to get agentID from PPSClient");
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
    ROS_INFO_STREAM("[TUNING CONTROLLER] The namespace string for accessing the Paramter Services are:");
    ROS_INFO_STREAM("[TUNING CONTROLLER] namespace_to_own_agent_parameter_service    =  " << namespace_to_own_agent_parameter_service);
    ROS_INFO_STREAM("[TUNING CONTROLLER] namespace_to_coordinator_parameter_service  =  " << namespace_to_coordinator_parameter_service);


    // FINALLY, FETCH ANY PARAMETERS REQUIRED FROM THESE "PARAMETER SERVICES"

	// Call the class function that loads the parameters for this class.
    fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);

    // *********************************************************************************



    // Subscribe to the message that activates the tests
	ros::Subscriber tuningActivateTestSubscriber = nodeHandle.subscribe("ActivateTest", 1, activateTestCallback);

	// Subscribe to the message that changes the gains
	ros::Subscriber tuningHorizontalGainSubscriber = nodeHandle.subscribe("HorizontalGain", 1, horizontalGainCallback);
	ros::Subscriber tuningVerticalGainSubscriber   = nodeHandle.subscribe("VerticalGain",   1, verticalGainCallback);
	ros::Subscriber tuningHeadingGainSubscriber    = nodeHandle.subscribe("HeadingGain",    1, headingGainCallback);



    // Instantiate the instance variable "debugPublisher" to be a "ros::Publisher" that
    // advertises under the name "DebugTopic" and is a message with the structure
    // defined in the file "DebugMsg.msg" (located in the "msg" folder).
    debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

    // Instantiate the local variable "setpointSubscriber" to be a "ros::Subscriber"
    // type variable that subscribes to the "Setpoint" topic and calls the class function
    // "setpointCallback" each time a messaged is received on this topic and the message
    // is passed as an input argument to the "setpointCallback" class function.
    //ros::Subscriber setpointSubscriber = nodeHandle.subscribe("Setpoint", 1, setpointCallback);

    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "CustomController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("TuningController", calculateControlOutput);

    // Create a "ros::NodeHandle" type local variable "namespace_nodeHandle" that points
    // to the name space of this node, i.e., "d_fall_pps" as specified by the line:
    //     "using namespace d_fall_pps;"
    // in the "DEFINES" section at the top of this file.
    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());

    // Print out some information to the user.
    ROS_INFO("[TUNING CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
