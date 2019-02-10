//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
//    Allows for simple tuning of a P(I)D controller
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

	
	// UPDATE THE SETPOINT FOR FLYING IN A CIRCLE - IF ACTIVE
	if (isActive_fly_test_circle)
	{
		update_setpoint_for_test_circle();
	}


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
			// Display what test will be activated
			ROS_INFO("[TUNING CONTROLLER] Received message to perform HORIZONTAL test");

			// Ensure the circle is not active
			if (isActive_fly_test_circle)
			{
				isActive_fly_test_circle = false;
			}

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
			// Display what test will be activated
			ROS_INFO("[TUNING CONTROLLER] Received message to perform VERTICAL test");

			// Ensure the circle is not active
			if (isActive_fly_test_circle)
			{
				isActive_fly_test_circle = false;
			}

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
			// Display what test will be activated
			ROS_INFO("[TUNING CONTROLLER] Received message to perform HEADING test");

			// Ensure the circle is not active
			if (isActive_fly_test_circle)
			{
				isActive_fly_test_circle = false;
			}

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
			// Display what test will be activated
			ROS_INFO("[TUNING CONTROLLER] Received message to perform ALL test");

			// Ensure the circle is not active
			if (isActive_fly_test_circle)
			{
				isActive_fly_test_circle = false;
			}

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

		// Test FLY A CIRCLE
		case 5:
		{
			// Display what test will be activated
			ROS_INFO("[TUNING CONTROLLER] Received message to FLY IN A CIRCLE");

			// Check that not already flying a circle
			if (!isActive_fly_test_circle)
			{
				// Set the boolean to make the circle active
				isActive_fly_test_circle = true;

				// (Re-)set the tick counter to zero
				test_circle_ticks_since_start = 0;
				
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


void update_setpoint_for_test_circle()
{
	// Compute the time since start
	float time_since_start = float(test_circle_ticks_since_start) / vicon_frequency;

	// Allow time to reach the start point
	if (time_since_start < test_circle_time_to_reach_start)
	{
		// Set the initial setpoint
		setpoint[0] = test_circle_radius;
		setpoint[1] = 0.0;
		setpoint[2] = test_circle_height;
		setpoint[3] = 0.0;
	}
	else if (time_since_start < (2.2*test_circle_seconds_per_rev) )
	{
		// Compute the current point in the period
		float omega_t = ((time_since_start-test_circle_time_to_reach_start)/test_circle_seconds_per_rev) * (2*PI);
		// Set the current setpoint on the circle
		setpoint[0] = test_circle_radius * cos(omega_t);
		setpoint[1] = test_circle_radius * sin(omega_t);
		setpoint[2] = test_circle_height;
		setpoint[3] = test_circle_pirouette_per_rev * omega_t;
	}
	else
	{
		// Turn of the flz circle mode
		isActive_fly_test_circle = false;
		// Return to the center
		setpoint[0] = 0.0;
		setpoint[1] = 0.0;
		setpoint[2] = test_circle_height;
		setpoint[3] = 0.0;
	}

	// Increment the tick counter
	test_circle_ticks_since_start++;
}



// CHANGE THE GAIN FOR THE HORIZONTAL CONTROLLER
void horizontalGainCallback(const std_msgs::Int32& msg)
{
	// Get the value from the message
	float value = float(msg.data) / 1000.0;

	// Compute the new multiplier value
	multiplier_horizontal = multiplier_horizontal_min + value * (multiplier_horizontal_max-multiplier_horizontal_min);

	// Display the value received
	ROS_INFO_STREAM("[TUNING CONTROLLER] Received new HORIZONTAL gain with value " << value << ", the multiplier now = " << multiplier_horizontal );
}

// CHANGE THE GAIN FOR THE VERTICAL CONTROLLER
void verticalGainCallback(const std_msgs::Int32& msg)
{
	// Get the value from the message
	float value = float(msg.data) / 1000.0;

	// Compute the new multiplier value
	multiplier_vertical = multiplier_vertical_min + value * (multiplier_vertical_max-multiplier_vertical_min);

	// Display the value received
	ROS_INFO_STREAM("[TUNING CONTROLLER] Received new VERTICAL gain with value " << value << ", the multiplier now = " << multiplier_vertical );
}

// CHANGE THE GAIN FOR THE HEADING CONTROLLER
void headingGainCallback(const std_msgs::Int32& msg)
{
	// Get the value from the message
	float value = float(msg.data) / 1000.0;

	// Compute the new multiplier value
	multiplier_heading = multiplier_heading_min + value * (multiplier_heading_max-multiplier_heading_min);

	// Display the value received
	ROS_INFO_STREAM("[TUNING CONTROLLER] Received new HEADING gain with value " << value << ", the multiplier now = " << multiplier_heading );
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
		rollRate_forResponse  -= gainMatrixRollRate[i] * stateErrorBody[i] * multiplier_horizontal;
		// BODY FRAME X CONTROLLER
		pitchRate_forResponse -= gainMatrixPitchRate[i] * stateErrorBody[i] * multiplier_horizontal;
		// BODY FRAME YAW CONTROLLER
		yawRate_forResponse   -= gainMatrixYawRate[i] * stateErrorBody[i] * multiplier_heading;
		// > ALITUDE CONTROLLER (i.e., z-controller):
		thrustAdjustment      -= gainMatrixThrust_NineStateVector[i] * stateErrorBody[i] * multiplier_vertical;
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
		rollAngle_forResponse -= gainMatrixRollAngle[i] * stateErrorBody[i] * multiplier_horizontal;
		// BODY FRAME X CONTROLLER
		pitchAngle_forResponse -= gainMatrixPitchAngle[i] * stateErrorBody[i] * multiplier_horizontal;
		// > ALITUDE CONTROLLER (i.e., z-controller):
		thrustAdjustment      -= gainMatrixThrust_SixStateVector[i] * stateErrorBody[i] * multiplier_vertical;
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
			//lqr_angleRateNested_prev_rollAngle        -= gainMatrixRollAngle_50Hz[i] * stateErrorBody[i] * multiplier_horizontal;
			// BODY FRAME X CONTROLLER
			//lqr_angleRateNested_prev_pitchAngle       -= gainMatrixPitchAngle_50Hz[i] * stateErrorBody[i] * multiplier_horizontal;
			// > ALITUDE CONTROLLER (i.e., z-controller):
			lqr_angleRateNested_prev_thrustAdjustment -= gainMatrixThrust_SixStateVector_50Hz[i] * stateErrorBody[i] * multiplier_vertical;
		}

		//lqr_angleRateNested_prev_rollAngle  -= ( -1 * m_gain_P * stateErrorBody[1] - m_gain_P * m_gain_P_to_D_ratio * stateErrorBody[4] );
		lqr_angleRateNested_prev_pitchAngle -= (      m_gain_P * stateErrorBody[0] + m_gain_P * m_gain_P_to_D_ratio * stateErrorBody[3] );

		// BODY FRAME Z CONTROLLER
		//lqr_angleRateNested_prev_yawAngle = setpoint[3];
		//lqr_angleRateNested_prev_yawAngle = stateErrorBody[8];


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
		//rollRate_forResponse  -= gainMatrixRollRate_Nested[i]  * temp_stateAngleError[i];
		// BODY FRAME X CONTROLLER
		pitchRate_forResponse -= gainMatrixPitchRate_Nested[i] * temp_stateAngleError[i];
		// BODY FRAME Z CONTROLLER
		//yawRate_forResponse   -= gainMatrixYawRate_Nested[i]   * temp_stateAngleError[i] * multiplier_heading;
	}



	float thrustAdjustment_200Hz = 0.0;
	// Perform the "-Kx" LQR computation for the rates and thrust:
	for(int i = 0; i < 9; ++i)
	{
		// BODY FRAME Y CONTROLLER
		rollRate_forResponse    -= gainMatrixRollRate[i] * stateErrorBody[i];
		// > ALITUDE CONTROLLER (i.e., z-controller):
		thrustAdjustment_200Hz  -= gainMatrixThrust_NineStateVector[i] * stateErrorBody[i];
		// BODY FRAME YAW CONTROLLER
		yawRate_forResponse   -= gainMatrixYawRate[i] * stateErrorBody[i];
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
	float thrustAdjustment = thrustAdjustment_200Hz / 4.0;
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
	m_debugPublisher.publish(debugMsg);
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
	//   for successful completion of the classroom exercise
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

// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
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

// This function DOES NOT NEED TO BE edited for successful completion of the classroom exercise
// void setpointCallback(const Setpoint& newSetpoint)
// {
//     setpoint[0] = newSetpoint.x;
//     setpoint[1] = newSetpoint.y;
//     setpoint[2] = newSetpoint.z;
//     setpoint[3] = newSetpoint.yaw;
// }




// REQUEST SETPOINT CHANGE CALLBACK
// This function does NOT need to be edited 
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
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


// CHANGE SETPOINT FUNCTION
// This function does NOT need to be edited 
void setNewSetpoint(float x, float y, float z, float yaw)
{
	// Put the new setpoint into the class variable
	setpoint[0] = x;
	setpoint[1] = y;
	setpoint[2] = z;
	setpoint[3] = yaw;

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// // Instantiate a local variable of type "SetpointWithHeader"
	// SetpointWithHeader msg;
	// // Fill in the setpoint
	// msg.x   = x;
	// msg.y   = y;
	// msg.z   = z;
	// msg.yaw = yaw;
	// // Publish the message
	// m_setpointChangedPublisher.publish(msg);
}




// REQUEST SETPOINT CHANGE CALLBACK
// This function does NOT need to be edited 
void requestGainChangeCallback(const FloatWithHeader& newGain)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newGain.shouldCheckForAgentID , newGain.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		m_gain_P = newGain.data;

		ROS_INFO_STREAM("[TUNING CONTROLLER] proportional gain changed to " << m_gain_P );
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
// This function does NOT need to be edited 
void isReadyTuningControllerYamlCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

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
				ROS_INFO("[TUNING CONTROLLER] Now fetching the TuningController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[TUNING CONTROLLER] Now fetching the TuningController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[TUNING CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchTuningControllerYamlParameters(nodeHandle_to_use);
	}
}


// This function CAN BE edited for successful completion of the classroom exercise, and the
// use of parameters fetched from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void fetchTuningControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the CustomController.yaml file

	// Add the "TuningController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "TuningController");


	// ******************************************************************************* //
	// PARAMETERS SPECIFIC TO THE TUNING CONTROL FEATURE

	/// Setpoint for the HORIZONTAL test
	getParameterFloatVector(nodeHandle_for_paramaters, "test_horizontal_setpoint1", test_horizontal_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "test_horizontal_setpoint2", test_horizontal_setpoint2, 4);

	// Setpoint for the VERTICAL test
	getParameterFloatVector(nodeHandle_for_paramaters, "test_vertical_setpoint1", test_vertical_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "test_vertical_setpoint2", test_vertical_setpoint2, 4);

	// Setpoint for the HEADING test
	getParameterFloatVector(nodeHandle_for_paramaters, "test_heading_setpoint1", test_heading_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "test_heading_setpoint2", test_heading_setpoint2, 4);

	// Setpoint for the ALL test
	getParameterFloatVector(nodeHandle_for_paramaters, "test_all_setpoint1", test_all_setpoint1, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "test_all_setpoint2", test_all_setpoint2, 4);

	// Parameters for flying in a circle
	test_circle_radius = getParameterFloat(nodeHandle_for_paramaters, "test_circle_radius");
	test_circle_seconds_per_rev = getParameterFloat(nodeHandle_for_paramaters, "test_circle_seconds_per_rev");
	test_circle_height = getParameterFloat(nodeHandle_for_paramaters, "test_circle_height");
	test_circle_time_to_reach_start = getParameterFloat(nodeHandle_for_paramaters, "test_circle_time_to_reach_start");
	test_circle_pirouette_per_rev = getParameterFloat(nodeHandle_for_paramaters, "test_circle_pirouette_per_rev");

	// Multipliers for the HORIZONTAL contorller
	multiplier_horizontal_min = getParameterFloat(nodeHandle_for_paramaters, "multiplier_horizontal_min");
	multiplier_horizontal_max = getParameterFloat(nodeHandle_for_paramaters, "multiplier_horizontal_max");
	// Multipliers for the VERTICAL contorller
	multiplier_vertical_min = getParameterFloat(nodeHandle_for_paramaters, "multiplier_vertical_min");
	multiplier_vertical_max = getParameterFloat(nodeHandle_for_paramaters, "multiplier_vertical_max");
	// Multipliers for the HEADING contorller
	multiplier_heading_min = getParameterFloat(nodeHandle_for_paramaters, "multiplier_heading_min");
	multiplier_heading_max = getParameterFloat(nodeHandle_for_paramaters, "multiplier_heading_max");

	// // DEBUGGING: Print out one of the parameters that was loaded
	ROS_INFO_STREAM("[TUNING CONTROLLER] DEBUGGING: the fetched TuningController/multiplier_horizontal_min = " << multiplier_horizontal_min);

	// ******************************************************************************* //



	// > The mass of the crazyflie
	cf_mass = getParameterFloat(nodeHandle_for_paramaters , "mass");

	// Display one of the YAML parameters to debug if it is working correctly
	//ROS_INFO_STREAM("DEBUGGING: mass leaded from loacl file = " << cf_mass );

	// > The frequency at which the "computeControlOutput" is being called, as determined
	//   by the frequency at which the Vicon system provides position and attitude data
	vicon_frequency = getParameterFloat(nodeHandle_for_paramaters, "vicon_frequency");

	// > The frequency at which the "computeControlOutput" is being called, as determined
	//   by the frequency at which the Vicon system provides position and attitude data
	control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit motor command to
	//   thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_paramaters, "motorPoly", motorPoly, 3);

	// > The boolean for whether to execute the convert into body frame function
	shouldPerformConvertIntoBodyFrame = getParameterBool(nodeHandle_for_paramaters, "shouldPerformConvertIntoBodyFrame");

	// Boolean indiciating whether the "Debug Message" of this agent should be published or not
	shouldPublishDebugMessage = getParameterBool(nodeHandle_for_paramaters, "shouldPublishDebugMessage");

	// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
	shouldDisplayDebugInfo = getParameterBool(nodeHandle_for_paramaters, "shouldDisplayDebugInfo");


	// THE CONTROLLER GAINS:

	// A flag for which controller to use:
	controller_mode = getParameterInt( nodeHandle_for_paramaters , "controller_mode" );

	// A flag for which estimator to use:
	estimator_method = getParameterInt( nodeHandle_for_paramaters , "estimator_method" );

	// The LQR Controller parameters for "LQR_MODE_RATE"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixThrust_NineStateVector", gainMatrixThrust_NineStateVector, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollRate",               gainMatrixRollRate,               9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchRate",              gainMatrixPitchRate,              9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixYawRate",                gainMatrixYawRate,                9);
	
	// The LQR Controller parameters for "LQR_MODE_ANGLE"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixThrust_SixStateVector",  gainMatrixThrust_SixStateVector,  6);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollAngle",              gainMatrixRollAngle,              6);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchAngle",             gainMatrixPitchAngle,             6);
	
	// The LQR Controller parameters for "LQR_MODE_ANGLE_RATE_NESTED"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixThrust_SixStateVector_50Hz",  gainMatrixThrust_SixStateVector_50Hz,  6);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollAngle_50Hz",              gainMatrixRollAngle_50Hz,              6);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchAngle_50Hz",             gainMatrixPitchAngle_50Hz,             6);

	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollRate_Nested",             gainMatrixRollRate_Nested,             3);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchRate_Nested",            gainMatrixPitchRate_Nested,            3);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixYawRate_Nested",              gainMatrixYawRate_Nested,              3);

	
	// 16-BIT COMMAND LIMITS
	cmd_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	cmd_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");


	// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
	// > For the (x,y,z) position
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row1_for_positions",  PMKF_Ahat_row1_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row2_for_positions",  PMKF_Ahat_row2_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Kinf_for_positions"     ,  PMKF_Kinf_for_positions     ,  2);
	// > For the (roll,pitch,yaw) angles
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row1_for_angles",  PMKF_Ahat_row1_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row2_for_angles",  PMKF_Ahat_row2_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Kinf_for_angles"     ,  PMKF_Kinf_for_angles     ,  2);


	// DEBUGGING: Print out one of the parameters that was loaded
	//ROS_INFO_STREAM("[TUNING CONTROLLER] DEBUGGING: the fetched TuningController/mass = " << cf_mass);


	// Call the function that computes details an values that are needed from these
	// parameters loaded above
	//processFetchedParameters();

	// Compute the feed-forward force that we need to counteract gravity (i.e., mg)
	// > in units of [Newtons]
	gravity_force         = cf_mass * 9.81/(1000);
	gravity_force_quarter = cf_mass * 9.81/(1000*4);

	// Set that the estimator frequency is the same as the control frequency
	estimator_frequency = vicon_frequency;

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[TUNING CONTROLLER] DEBUGGING: after processing the gravity_force_quarter = " << gravity_force_quarter);
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
    ros::init(argc, argv, "TuningControllerService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "TuningControllerService" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[TUNING CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);

    // AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "FlyingAgentClient" node.
	// > Thus, to get access to this "agentID" paremeter, we first
	//   need to get a handle to the "FlyingAgentClient" node within which this
	//   controller service is nested.


	// Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[TUNING CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[TUNING CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
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
	ROS_INFO_STREAM("[TUNING CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[TUNING CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "TuningController", 1, isReadyTuningControllerYamlCallback);
	ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("TuningController", 1, isReadyTuningControllerYamlCallback);



	// GIVE YAML VARIABLES AN INITIAL VALUE
	// This can be done either here or as part of declaring the
	// variables in the header file

	// // > the mass of the crazyflie, in [grams]
	// yaml_cf_mass_in_grams = 25.0;
	// // > the coefficients of the 16-bit command to thrust conversion
	// yaml_motorPoly[0] = 5.484560e-4;
	// yaml_motorPoly[1] = 1.032633e-6;
	// yaml_motorPoly[2] = 2.130295e-11;
	// // > the frequency at which the controller is running
	// yaml_control_frequency = 200.0;
	// // > the default setpoint, the ordering is (x,y,z,yaw),
	// //   with units [meters,meters,meters,radians]
	// yaml_default_setpoint[0] = 0.0;
	// yaml_default_setpoint[1] = 0.0;
	// yaml_default_setpoint[2] = 0.4;
	// yaml_default_setpoint[3] = 0.0;


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
	loadYamlFromFilenameCall.request.stringWithHeader.data = "TuningController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForAgentID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyTuningControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
		// Inform the user
		ROS_ERROR("[TUNING CONTROLLER] The request load yaml file service call failed.");
	}








	// // Subscribe to the message that activates the tests
	// ros::Subscriber tuningActivateTestSubscriber = nodeHandle.subscribe("ActivateTest", 1, activateTestCallback);

	// // Subscribe to the message that changes the gains
	// ros::Subscriber tuningHorizontalGainSubscriber = nodeHandle.subscribe("HorizontalGain", 1, horizontalGainCallback);
	// ros::Subscriber tuningVerticalGainSubscriber   = nodeHandle.subscribe("VerticalGain",   1, verticalGainCallback);
	// ros::Subscriber tuningHeadingGainSubscriber    = nodeHandle.subscribe("HeadingGain",    1, headingGainCallback);






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
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("TuningControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Instantiate the local variable "requestGainChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestGainChange" topic and calls the class function
	// "requestGainChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the gain is changed
	// by the user.
	ros::Subscriber requestGainChangeSubscriber = nodeHandle.subscribe("RequestGainChange", 1, requestGainChangeCallback);



    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "TuningController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("TuningController", calculateControlOutput);



    // Create a "ros::NodeHandle" type local variable "namespace_nodeHandle" that points
    // to the name space of this node, i.e., "dfall_pkg" as specified by the line:
    //     "using namespace dfall_pkg;"
    // in the "DEFINES" section at the top of this file.
    //ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());



    // Print out some information to the user.
    ROS_INFO("[TUNING CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
