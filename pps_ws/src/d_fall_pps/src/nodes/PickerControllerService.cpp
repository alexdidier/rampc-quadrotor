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
#include "nodes/PickerControllerService.h"





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




// REMINDER OF THE NAME OF USEFUL CLASS VARIABLE
// // > Mass of the Crazyflie quad-rotor, in [grams]
// float m_mass_CF_grams;
// // > Mass of the letters to be lifted, in [grams]
// float m_mass_E_grams;
// float m_mass_T_grams;
// float m_mass_H_grams;
// // > Total mass of the Crazyflie plus whatever it is carrying, in [grams]
// float m_mass_total_grams;
// // Thickness of the object at pick-up and put-down, in [meters]
// // > This should also account for extra height due to 
// //   the surface where the object is
// float m_thickness_of_object_at_pickup;
// float m_thickness_of_object_at_putdown;
// // (x,y) coordinates of the pickup location
// std::vector<float> m_pickup_coordinates_xy(2);
// // (x,y) coordinates of the drop off location
// std::vector<float> m_dropoff_coordinates_xy_for_E(2);
// std::vector<float> m_dropoff_coordinates_xy_for_T(2);
// std::vector<float> m_dropoff_coordinates_xy_for_H(2);
// // Length of the string from the Crazyflie
// // to the end of the Picker, in [meters]
// float m_picker_string_length;
// // > The setpoints for (x,y,z) position and yaw angle, in that order
// float m_setpoint[4] = {0.0,0.0,0.4,0.0};
// float m_setpoint_for_controller[4] = {0.0,0.0,0.4,0.0};
// // > Small adjustments to the x-y setpoint
// float m_xAdjustment = 0.0f;
// float m_yAdjustment = 0.0f;
// // Boolean for whether to limit rate of change of the setpoint
// bool m_shouldSmoothSetpointChanges = true;
// // Max setpoint change per second
// float yaml_max_setpoint_change_per_second_horizontal;
// float yaml_max_setpoint_change_per_second_vertical;
// float yaml_max_setpoint_change_per_second_yaw_degrees;
// float m_max_setpoint_change_per_second_yaw_radians;
// // Frequency at which the controller is running
// float yaml_control_frequency;


// A FEW EXTRA COMMENTS ABOUT THE MOST IMPORTANT VARIABLES

// Variable name:    m_setpoint
// Description:
// This is a float array of length 4. It specifies a location
// in space where you want the drone to be. The 4 element are:
// >> m_setpoint[0]   The x-poistion in [meters]
// >> m_setpoint[1]   The y-poistion in [meters]
// >> m_setpoint[2]   The z-poistion in [meters]
// >> m_setpoint[3]   The yaw heading angle in [radians]


// Variable name:    m_setpoint_for_controller
// Description:
// Similar to the variable "m_setpoint" this is also float array
// of length 4 that specifies an (x,y,z,yaw) location. The
// difference it that this variable specifies the location where
// the low-level controller is guiding the drone to be.
// HINT: to make changes the "m_setpoint" variable, you can edit
// the function named "perControlCycleOperations" so that the
// "m_setpoint_for_controller" changes by a maximum amount at
// each cycle of the contoller



// THIS FUNCTION IS CALLED AT "m_control_frequency" HERTZ.
// IT CAN BE USED TO ADJUST THINGS IN "REAL TIME".
// For example, the equation:
// >> yaml_max_setpoint_change_per_second_horizontal / m_control_frequency
// will convert the "change per second" to a "change per cycle".

void perControlCycleOperations()
{
	
}







// CALLBACK FUNCTION THAT RUN WHEN THE RESPECTIVE BUTTON IS PRESSED

// These functions are called from:
// >> "requestSetpointChangeCallback"
// And in that function the following variable are already
// updated appropriately:
// >> "m_picker_current_state"
// >> "m_mass_total_grams"
// >> "m_shouldSmoothSetpointChanges"

void buttonPressed_goto_start()
{
	ROS_INFO("[PICKER CONTROLLER] Goto Start button pressed");

	// // The drone should move smoothly to the start point:
	// m_shouldSmoothSetpointChanges = true;
	// // Set the (x,y) coordinates for the start point:
	// m_setpoint[0] = m_pickup_coordinates_xy[0];
	// m_setpoint[1] = m_pickup_coordinates_xy[1];
	// // Set the z coordinate to be a little more than the
	// // length of the "picker string"
	// m_setpoint[2] = m_picker_string_length + 0.10;
	// // Publish the setpoint so that the GUI updates
	// publishCurrentSetpoint();
}

void buttonPressed_attach()
{
	ROS_INFO("[PICKER CONTROLLER] Attach button pressed");

	// m_shouldSmoothSetpointChanges = true;
	// m_setpoint[2] = m_picker_string_length + m_thickness_of_object_at_pickup;
	// publishCurrentSetpoint();
}

void buttonPressed_lift_up()
{
	ROS_INFO("[PICKER CONTROLLER] Pick up button pressed");

	// m_shouldSmoothSetpointChanges = true;
	// m_mass_total_grams = m_mass_CF_grams + m_mass_E_grams;
	// m_setpoint[2] = m_picker_string_length + 0.25;
	// publishCurrentSetpoint();
}

void buttonPressed_goto_end()
{
	ROS_INFO("[PICKER CONTROLLER] Goto End button pressed");

	// m_shouldSmoothSetpointChanges = true;
	// m_setpoint[0] = m_dropoff_coordinates_xy_for_E[0];
	// m_setpoint[1] = m_dropoff_coordinates_xy_for_E[1];
	// publishCurrentSetpoint();
}	

void buttonPressed_put_down()
{
	ROS_INFO("[PICKER CONTROLLER] Put down button pressed");

	// m_shouldSmoothSetpointChanges = true;
	// m_setpoint[2] = m_picker_string_length + m_thickness_of_object_at_putdown;
	// m_mass_total_grams = m_mass_CF_grams;
	// publishCurrentSetpoint();
}

void buttonPressed_squat()
{
	ROS_INFO("[PICKER CONTROLLER] Squat button pressed");

	// m_shouldSmoothSetpointChanges = true;
	// m_setpoint[2] = m_picker_string_length - 0.10;
	// m_mass_total_grams = m_mass_CF_grams;
	// publishCurrentSetpoint();
}

void buttonPressed_jump()
{
	ROS_INFO("[PICKER CONTROLLER] Jump button pressed");

	// m_shouldSmoothSetpointChanges = false;
	// m_setpoint[2] = m_picker_string_length + 0.10;
	// m_mass_total_grams = m_mass_CF_grams;
	// publishCurrentSetpoint();
}

void buttonPressed_standby()
{
	ROS_INFO("[PICKER CONTROLLER] Standby button pressed");

	// m_shouldSmoothSetpointChanges = false;
	// m_setpoint[2] = m_picker_string_length + 0.10;
	// m_mass_total_grams = m_mass_CF_grams;
	// publishCurrentSetpoint();
}



// THESE CALLBACK FUNCTIONS ALLOW YOU TO IMPLEMENT SOME
// CUSTOM ACTION IN RESPONSE TO THE RESPECTIVE BUTTON PRESSES

// void buttonPressed_1()
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 1 pressed");
// }
// void buttonPressed_2()
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 2 pressed");
// }

// void buttonPressed_3()
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 3 pressed");
// }

// void buttonPressed_4()
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 4 pressed");
// }










// void zSetpointCallback(const std_msgs::Float32& msg)
// {
// 	// The "data" in the message is z-height in [meters]
// 	float z_height = msg.data;
// 	// Display the data
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Z Slider changed to " << z_height << " [m]" );
// 	// Update the z-component of the setpoint class variable
// 	m_setpoint[2] = z_height;
// }


// void yawSetpointCallback(const std_msgs::Float32& msg)
// {
// 	// The "data" in the message is yaw-angle in [radians]
// 	float yaw_angle = msg.data;
// 	// Display the data
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Yaw Dial changed to " << (yaw_angle*RAD2DEG) << " [deg]" );
// 	// Update the yaw-component of the setpoint class variable
// 	m_setpoint[3] = yaw_angle;
// }

// void massCallback(const std_msgs::Float32& msg)
// {
// 	// The "data" in the message is mass in [grams]
// 	float mass = msg.data;
// 	// Display the data
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Mass slider changed to " << mass << " [grams]" );
// 	// Update the total mass class variable
// 	m_mass_total_grams = mass;
// }

// void xAdjustmentCallback(const std_msgs::Float32& msg)
// {
// 	// The "data" in the message is adjustment in [meters]
// 	float x_adjustment = msg.data;
// 	// Display the data
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] X adjustment slider changed to " << x_adjustment << " [m]" );
// 	// Update the x-adjustment class variable
// 	m_xAdjustment = x_adjustment;
// }

// void yAdjustmentCallback(const std_msgs::Float32& msg)
// {
// 	// The "data" in the message is adjustment in [meters]
// 	float y_adjustment = msg.data;
// 	// Display the data
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Y adjustment slider changed to " << y_adjustment << " [m]" );
// 	// Update the y-adjustment class variable
// 	m_yAdjustment = y_adjustment;
// }














// CALLBACK FUNCTION THAT RUN WHEN THE RESPECTIVE BUTTON IS PRESSED
// > AND A SETPOINT IS PROVIDED

// void buttonPressedWithSetpoint_gotoStart(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Goto Start button pressed: (x,y,z)=(" << newSetpointV2.x << "," << newSetpointV2.y << "," << newSetpointV2.z << "), smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the (x,y,z) coordinates:
// 	m_setpoint[0] = newSetpointV2.x;
// 	m_setpoint[1] = newSetpointV2.y;
// 	m_setpoint[2] = newSetpointV2.z;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }

// void buttonPressedWithSetpoint_attach(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Attach button pressed: z=" << newSetpointV2.z << ", smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the z coordinate:
// 	m_setpoint[2] = newSetpointV2.z;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }

// void buttonPressedWithSetpoint_pickup(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Pick up button pressed: z=" << newSetpointV2.z << ", smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the z coordinate:
// 	m_setpoint[2] = newSetpointV2.z;
// 	// Update the mass of the Crazyflie
// 	m_mass_total_grams = m_mass_CF_grams + m_mass_E_grams;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }

// void buttonPressedWithSetpoint_gotoEnd(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Goto End button pressed: (x,y)=(" << newSetpointV2.x << "," << newSetpointV2.y << "), smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the (x,y) coordinates:
// 	m_setpoint[0] = newSetpointV2.x;
// 	m_setpoint[1] = newSetpointV2.y;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }

// void buttonPressedWithSetpoint_putdown(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Put down button pressed: z=" << newSetpointV2.z << ", smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the z coordinate:
// 	m_setpoint[2] = newSetpointV2.z;
// 	// Update the mass of the Crazyflie
// 	m_mass_total_grams = m_mass_CF_grams;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }

// void buttonPressedWithSetpoint_squat(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Squat button pressed: z=" << newSetpointV2.z << ", smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the z coordinate:
// 	m_setpoint[2] = newSetpointV2.z;
// 	// Update the mass of the Crazyflie
// 	m_mass_total_grams = m_mass_CF_grams;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }

// void buttonPressedWithSetpoint_jump(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO_STREAM("[PICKER CONTROLLER] Jump button pressed: z=" << newSetpointV2.z << ", smooth=" << int(newSetpointV2.isChecked) );

// 	// Use the boolean for the smoothing flag
// 	m_shouldSmoothSetpointChanges = newSetpointV2.isChecked;
// 	// Set the z coordinate:
// 	m_setpoint[2] = newSetpointV2.z;
// 	// Update the mass of the Crazyflie
// 	m_mass_total_grams = m_mass_CF_grams;
// 	// Publish the setpoint so that the GUI updates
// 	//publishCurrentSetpoint();
// }


// void buttonPressedWithSetpoint_1(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 1 pressed with a setpoint provided");
// }

// void buttonPressedWithSetpoint_2(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 2 pressed with a setpoint provided");
// }

// void buttonPressedWithSetpoint_3(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 3 pressed with a setpoint provided");
// }

// void buttonPressedWithSetpoint_4(const SetpointV2& newSetpointV2)
// {
// 	ROS_INFO("[PICKER CONTROLLER] Button 4 pressed with a setpoint provided with a setpoint provided");
// }







void smoothSetpointChanges()
{
	if (m_shouldSmoothSetpointChanges)
	{
		for(int i = 0; i < 4; ++i)
		{
			float max_for_this_coordinate;
			// FILL IN THE STATE INERTIAL ESTIMATE TO BE USED FOR CONTROL
			switch (i)
			{
				case 0:
					max_for_this_coordinate = yaml_max_setpoint_change_per_second_horizontal / yaml_control_frequency;
					break;
				case 1:
					max_for_this_coordinate = yaml_max_setpoint_change_per_second_horizontal / yaml_control_frequency;
					break;
				case 2:
					max_for_this_coordinate = yaml_max_setpoint_change_per_second_vertical / yaml_control_frequency;
					break;
				case 3:
					max_for_this_coordinate = m_max_setpoint_change_per_second_yaw_radians / yaml_control_frequency;
					break;
				// Handle the exception
				default:
					max_for_this_coordinate = 0.0f;
					break;
			}

			// Compute the difference in setpoint
			float setpoint_difference = m_setpoint[i] - m_setpoint_for_controller[i];

			// Clip the difference to the maximum
			if (setpoint_difference > max_for_this_coordinate)
			{
				setpoint_difference = max_for_this_coordinate;
			}
			else if (setpoint_difference < -max_for_this_coordinate)
			{
				setpoint_difference = -max_for_this_coordinate;
			}

			// Update the setpoint of the controller
			m_setpoint_for_controller[i] += setpoint_difference;
		}
		
	}
	else
	{
		m_setpoint_for_controller[0] = m_setpoint[0];
		m_setpoint_for_controller[1] = m_setpoint[1];
		m_setpoint_for_controller[2] = m_setpoint[2];
		m_setpoint_for_controller[3] = m_setpoint[3];
	}
}













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

// This function is the callback that is linked to the "PickerController" service that
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
//   top of this file, they are:
//   CF_COMMAND_TYPE_MOTOR
//   CF_COMMAND_TYPE_RATE
//   CF_COMMAND_TYPE_ANGLE.
// > With CF_COMMAND_TYPE_RATE the ".roll", ".ptich", and ".yaw" properties of "response.ControlCommand"
//   specify the angular rate in [radians/second] that will be requested from the
//   PID controllers running in the Crazyflie 2.0 firmware.
// > With CF_COMMAND_TYPE_RATE the ".motorCmd1" to ".motorCmd4" properties of "response.ControlCommand"
//   are the baseline motor commands requested from the Crazyflie, with the adjustment
//   for body rates being added on top of this in the firmware (i.e., as per the code
//   of the "distribute_power" function provided in exercise sheet 2).
// > With CF_COMMAND_TYPE_RATE the axis convention for the roll, pitch, and yaw body rates returned
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

	// Keep track of time
	//m_time_ticks++;
	//m_time_seconds = float(m_time_ticks) / yaml_control_frequency;


	// CALL THE FUNCTION FOR PER CYLCE OPERATIONS
	perControlCycleOperations();


	// THIS IS THE START OF THE "OUTER" CONTROL LOOP
	// > i.e., this is the control loop run on this laptop
	// > this function is called at the frequency specified
	// > this function performs all estimation and control


	// PERFORM THE ESTIMATOR UPDATE FOR THE INTERIAL FRAME STATE
	// > After this function is complete the class variable
	//   "m_current_stateInertialEstimate" is updated and ready
	//   to be used for subsequent controller copmutations
	performEstimatorUpdate_forStateInterial(request);


	// SMOOTH ANY CHANGES THAT MAY HAVE OCCURRED IN THE
	// SETPOINT
	smoothSetpointChanges();


	// CONVERT THE CURRENT INERTIAL FRAME STATE ESTIMATE, INTO
	// THE BODY FRAME ERROR REQUIRED BY THE CONTROLLER
	// > Define a local array to fill in with the body frame error
	float current_bodyFrameError[12];
	// > Call the function to perform the conversion
	convert_stateInertial_to_bodyFrameError(m_current_stateInertialEstimate,m_setpoint_for_controller,current_bodyFrameError);


	// CARRY OUT THE CONTROLLER COMPUTATIONS
	// Call the function that performs the control computations for this mode
	calculateControlOutput_viaLQRforRates(current_bodyFrameError,request,response);


	// // PUBLISH THE CURRENT X,Y,Z, AND YAW (if required)
	// if (shouldPublishCurrent_xyz_yaw)
	// {
	// 	publish_current_xyz_yaw(request.ownCrazyflie.x,request.ownCrazyflie.y,request.ownCrazyflie.z,request.ownCrazyflie.yaw);
	// }

	// PUBLISH THE DEBUG MESSAGE (if required)
	if (yaml_shouldPublishDebugMessage)
	{
		construct_and_publish_debug_message(request,response);
	}

	// RETURN "true" TO INDICATE THAT THE COMPUTATIONS WERE SUCCESSFUL
	return true;
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
	m_current_xzy_rpy_measurement[0] = request.ownCrazyflie.x;
	m_current_xzy_rpy_measurement[1] = request.ownCrazyflie.y;
	m_current_xzy_rpy_measurement[2] = request.ownCrazyflie.z;
	// > for (roll,pitch,yaw) angles
	m_current_xzy_rpy_measurement[3] = request.ownCrazyflie.roll;
	m_current_xzy_rpy_measurement[4] = request.ownCrazyflie.pitch;
	m_current_xzy_rpy_measurement[5] = request.ownCrazyflie.yaw;


	// RUN THE FINITE DIFFERENCE ESTIMATOR
	performEstimatorUpdate_forStateInterial_viaFiniteDifference();


	// RUN THE POINT MASS KALMAN FILTER ESTIMATOR
	performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter();


	// FILLE IN THE STATE INERTIAL ESTIMATE TO BE USED FOR CONTROL
	switch (yaml_estimator_method)
	{
		// Estimator based on finte differences
		case ESTIMATOR_METHOD_FINITE_DIFFERENCE:
		{
			// Transfer the estimate
			for(int i = 0; i < 12; ++i)
			{
				m_current_stateInertialEstimate[i]  = m_stateInterialEstimate_viaFiniteDifference[i];
			}
			break;
		}
		// Estimator based on Point Mass Kalman Filter
		case ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION:
		{
			// Transfer the estimate
			for(int i = 0; i < 12; ++i)
			{
				m_current_stateInertialEstimate[i]  = m_stateInterialEstimate_viaPointMassKalmanFilter[i];
			}
			break;
		}
		// Handle the exception
		default:
		{
			// Display that the "yaml_estimator_method" was not recognised
			ROS_INFO_STREAM("[PICKER CONTROLLER] ERROR: in the 'calculateControlOutput' function of the 'PickerControllerService': the 'yaml_estimator_method' is not recognised.");
			// Transfer the finite difference estimate by default
			for(int i = 0; i < 12; ++i)
			{
				m_current_stateInertialEstimate[i]  = m_stateInterialEstimate_viaFiniteDifference[i];
			}
			break;
		}
	}


	// NOW THAT THE ESTIMATORS HAVE ALL BEEN RUN, PUT THE CURRENT
	// MEASURED DATA INTO THE CLASS VARIABLE FOR THE PREVIOUS 
	// > for (x,y,z) position
	m_previous_xzy_rpy_measurement[0] = m_current_xzy_rpy_measurement[0];
	m_previous_xzy_rpy_measurement[1] = m_current_xzy_rpy_measurement[1];
	m_previous_xzy_rpy_measurement[2] = m_current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	m_previous_xzy_rpy_measurement[3] = m_current_xzy_rpy_measurement[3];
	m_previous_xzy_rpy_measurement[4] = m_current_xzy_rpy_measurement[4];
	m_previous_xzy_rpy_measurement[5] = m_current_xzy_rpy_measurement[5];
}



void performEstimatorUpdate_forStateInterial_viaFiniteDifference()
{
	// PUT IN THE CURRENT MEASUREMENT DIRECTLY
	// > for (x,y,z) position
	m_stateInterialEstimate_viaFiniteDifference[0]  = m_current_xzy_rpy_measurement[0];
	m_stateInterialEstimate_viaFiniteDifference[1]  = m_current_xzy_rpy_measurement[1];
	m_stateInterialEstimate_viaFiniteDifference[2]  = m_current_xzy_rpy_measurement[2];
	// > for (roll,pitch,yaw) angles
	m_stateInterialEstimate_viaFiniteDifference[6]  = m_current_xzy_rpy_measurement[3];
	m_stateInterialEstimate_viaFiniteDifference[7]  = m_current_xzy_rpy_measurement[4];
	m_stateInterialEstimate_viaFiniteDifference[8]  = m_current_xzy_rpy_measurement[5];

	// COMPUTE THE VELOCITIES VIA FINITE DIFFERENCE
	// > for (x,y,z) velocities
	m_stateInterialEstimate_viaFiniteDifference[3]  = (m_current_xzy_rpy_measurement[0] - m_previous_xzy_rpy_measurement[0]) * m_estimator_frequency;
	m_stateInterialEstimate_viaFiniteDifference[4]  = (m_current_xzy_rpy_measurement[1] - m_previous_xzy_rpy_measurement[1]) * m_estimator_frequency;
	m_stateInterialEstimate_viaFiniteDifference[5]  = (m_current_xzy_rpy_measurement[2] - m_previous_xzy_rpy_measurement[2]) * m_estimator_frequency;
	// > for (roll,pitch,yaw) velocities
	m_stateInterialEstimate_viaFiniteDifference[9]  = (m_current_xzy_rpy_measurement[3] - m_previous_xzy_rpy_measurement[3]) * m_estimator_frequency;
	m_stateInterialEstimate_viaFiniteDifference[10] = (m_current_xzy_rpy_measurement[4] - m_previous_xzy_rpy_measurement[4]) * m_estimator_frequency;
	m_stateInterialEstimate_viaFiniteDifference[11] = (m_current_xzy_rpy_measurement[5] - m_previous_xzy_rpy_measurement[5]) * m_estimator_frequency;
}



void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter()
{
	// PERFORM THE KALMAN FILTER UPDATE STEP
	// > First take a copy of the estimator state
	float temp_PMKFstate[12];
	for(int i = 0; i < 12; ++i)
	{
		temp_PMKFstate[i]  = m_stateInterialEstimate_viaPointMassKalmanFilter[i];
	}
	// > Now perform update for:
	// > x position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[0] = yaml_PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[0] + yaml_PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[3] + yaml_PMKF_Kinf_for_positions[0]*m_current_xzy_rpy_measurement[0];
	m_stateInterialEstimate_viaPointMassKalmanFilter[3] = yaml_PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[0] + yaml_PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[3] + yaml_PMKF_Kinf_for_positions[1]*m_current_xzy_rpy_measurement[0];
	// > y position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[1] = yaml_PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[1] + yaml_PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[4] + yaml_PMKF_Kinf_for_positions[0]*m_current_xzy_rpy_measurement[1];
	m_stateInterialEstimate_viaPointMassKalmanFilter[4] = yaml_PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[1] + yaml_PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[4] + yaml_PMKF_Kinf_for_positions[1]*m_current_xzy_rpy_measurement[1];
	// > z position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[2] = yaml_PMKF_Ahat_row1_for_positions[0]*temp_PMKFstate[2] + yaml_PMKF_Ahat_row1_for_positions[1]*temp_PMKFstate[5] + yaml_PMKF_Kinf_for_positions[0]*m_current_xzy_rpy_measurement[2];
	m_stateInterialEstimate_viaPointMassKalmanFilter[5] = yaml_PMKF_Ahat_row2_for_positions[0]*temp_PMKFstate[2] + yaml_PMKF_Ahat_row2_for_positions[1]*temp_PMKFstate[5] + yaml_PMKF_Kinf_for_positions[1]*m_current_xzy_rpy_measurement[2];

	// > roll  position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[6]  = yaml_PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[6] + yaml_PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[9]  + yaml_PMKF_Kinf_for_angles[0]*m_current_xzy_rpy_measurement[3];
	m_stateInterialEstimate_viaPointMassKalmanFilter[9]  = yaml_PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[6] + yaml_PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[9]  + yaml_PMKF_Kinf_for_angles[1]*m_current_xzy_rpy_measurement[3];
	// > pitch position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[7]  = yaml_PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[7] + yaml_PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[10] + yaml_PMKF_Kinf_for_angles[0]*m_current_xzy_rpy_measurement[4];
	m_stateInterialEstimate_viaPointMassKalmanFilter[10] = yaml_PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[7] + yaml_PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[10] + yaml_PMKF_Kinf_for_angles[1]*m_current_xzy_rpy_measurement[4];
	// > yaw   position and velocity:
	m_stateInterialEstimate_viaPointMassKalmanFilter[8]  = yaml_PMKF_Ahat_row1_for_angles[0]*temp_PMKFstate[8] + yaml_PMKF_Ahat_row1_for_angles[1]*temp_PMKFstate[11] + yaml_PMKF_Kinf_for_angles[0]*m_current_xzy_rpy_measurement[5];
	m_stateInterialEstimate_viaPointMassKalmanFilter[11] = yaml_PMKF_Ahat_row2_for_angles[0]*temp_PMKFstate[8] + yaml_PMKF_Ahat_row2_for_angles[1]*temp_PMKFstate[11] + yaml_PMKF_Kinf_for_angles[1]*m_current_xzy_rpy_measurement[5];
}





//    ----------------------------------------------------------------------------------
//    L       QQQ   RRRR
//    L      Q   Q  R   R
//    L      Q   Q  RRRR
//    L      Q  Q   R  R
//    LLLLL   QQ Q  R   R
//    ----------------------------------------------------------------------------------

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
		rollRate_forResponse  -= yaml_gainMatrixRollRate[i] * stateErrorBody[i];
		// BODY FRAME X CONTROLLER
		pitchRate_forResponse -= yaml_gainMatrixPitchRate[i] * stateErrorBody[i];
		// BODY FRAME YAW CONTROLLER
		yawRate_forResponse   -= yaml_gainMatrixYawRate[i] * stateErrorBody[i];
		// > ALITUDE CONTROLLER (i.e., z-controller):
		thrustAdjustment      -= yaml_gainMatrixThrust_NineStateVector[i] * stateErrorBody[i];
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
	// > Compute the feed-forward force
	float feed_forward_thrust_per_motor = m_weight_total_in_newtons / 4.0;
	// > Put in the per motor commands
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustAdjustment + feed_forward_thrust_per_motor);

	
	// Specify that this controller is a rate controller
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTOR;
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;
	// response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_ANGLE;


	// An alternate debugging technique is to print out data directly to the
	// command line from which this node was launched.
	if (yaml_shouldDisplayDebugInfo)
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
		ROS_INFO_STREAM("motorPoly 0:" << yaml_motorPoly[0]);
		ROS_INFO_STREAM("motorPoly 1:" << yaml_motorPoly[1]);
		ROS_INFO_STREAM("motorPoly 2:" << yaml_motorPoly[2]);

		// An example of "printing out" the per motor 16-bit command computed.
		ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);
	}
}



//  ***********************************************************
//  DDDD   EEEEE  BBBB   U   U   GGGG       M   M   SSSS   GGGG
//  D   D  E      B   B  U   U  G           MM MM  S      G
//  D   D  EEE    BBBB   U   U  G           M M M   SSS   G
//  D   D  E      B   B  U   U  G   G       M   M      S  G   G
//  DDDD   EEEEE  BBBB    UUU    GGGG       M   M  SSSS    GGGG

void construct_and_publish_debug_message(Controller::Request &request, Controller::Response &response)
{
	
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

void convertIntoBodyFrame(float stateInertial[12], float (&stateBody)[12], float yaw_measured)
{
	if (yaml_shouldPerformConvertIntoBodyFrame)
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

	// Clip the z-coordination to +/-0.40 meters
	if (stateInertial[2] > 0.40f)
	{
		stateInertial[2] = 0.40f;
	}
	else if (stateInertial[2] < -0.40f)
	{
		stateInertial[2] = -0.40f;
	}

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
	float cmd = (-yaml_motorPoly[1] + sqrt(yaml_motorPoly[1] * yaml_motorPoly[1] - 4 * yaml_motorPoly[2] * (yaml_motorPoly[0] - thrust))) / (2 * yaml_motorPoly[2]);

	// Saturate the signal to be 0 or in the range [1000,65000]
	if (cmd < yaml_cmd_sixteenbit_min)
	{
		cmd = 0;
	}
	else if (cmd > yaml_cmd_sixteenbit_max)
	{
		cmd = yaml_cmd_sixteenbit_max;
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


// REQUEST SETPOINT CHANGE CALLBACK
// This function does NOT need to be edited 
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Check that the ".buttonID" property is
		// actually one of the possible states
		int button_index = newSetpoint.buttonID;
		switch(button_index)
		{
			case PICKER_STATE_STANDBY:
			case PICKER_STATE_GOTO_START:
			case PICKER_STATE_ATTACH:
			case PICKER_STATE_LIFT_UP:
			case PICKER_STATE_GOTO_END:
			case PICKER_STATE_PUT_DOWN:
			case PICKER_STATE_SQUAT:
			case PICKER_STATE_JUMP:
			{
				// Call the function for actually setting the setpoint
				setNewSetpoint(
						newSetpoint.buttonID,
						newSetpoint.isChecked,
						newSetpoint.x,
						newSetpoint.y,
						newSetpoint.z,
						newSetpoint.yaw,
						newSetpoint.mass
					);
				break;
			}
			default:
			{
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[PICKER CONTROLLER] A button pressed message was received in the controller but not recognised, message.data = " << button_index );
				break;
			}
		}
		
		// Call the specific function for each button
		switch(button_index)
		{
			case PICKER_STATE_STANDBY:
			{
				buttonPressed_standby();
				break;
			}
			case PICKER_STATE_GOTO_START:
			{
				buttonPressed_goto_start();
				break;
			}
			case PICKER_STATE_ATTACH:
			{
				buttonPressed_attach();
				break;
			}
			case PICKER_STATE_LIFT_UP:
			{
				buttonPressed_lift_up();
				break;
			}
			case PICKER_STATE_GOTO_END:
			{
				buttonPressed_goto_end();
				break;
			}
			case PICKER_STATE_PUT_DOWN:
			{
				buttonPressed_put_down();
				break;
			}
			case PICKER_STATE_SQUAT:
			{
				buttonPressed_squat();
				break;
			}
			case PICKER_STATE_JUMP:
			{
				buttonPressed_jump();
				break;
			}
		}
	}
}


// CHANGE SETPOINT FUNCTION
// This function does NOT need to be edited 
void setNewSetpoint(int state, bool should_smooth, float x, float y, float z, float yaw, float mass)
{
	// Put the state into the class variable
	m_picker_current_state = state;

	// Put the smoothing flag in the class variable
	m_shouldSmoothSetpointChanges = should_smooth;

	// Put the new setpoint into the class variable
	m_setpoint[0] = x;
	m_setpoint[1] = y;
	m_setpoint[2] = z;
	m_setpoint[3] = yaw;

	// Put the mass into the class variable
	m_mass_total_in_grams = mass;
	m_weight_total_in_newtons = m_mass_total_in_grams * (9.81/1000.0);

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.buttonID  = state;
	msg.isChecked = should_smooth;
	msg.x         = x;
	msg.y         = y;
	msg.z         = z;
	msg.yaw       = yaw;
	msg.mass      = mass;
	// Publish the message
	m_setpointChangedPublisher.publish(msg);
}


// GET CURRENT SETPOINT SERVICE CALLBACK
// This function does NOT need to be edited 
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response)
{
	// Directly put the current setpoint into the response
	response.setpointWithHeader.buttonID  = m_picker_current_state;
	response.setpointWithHeader.isChecked = m_shouldSmoothSetpointChanges;
	response.setpointWithHeader.x         = m_setpoint[0];
	response.setpointWithHeader.y         = m_setpoint[1];
	response.setpointWithHeader.z         = m_setpoint[2];
	response.setpointWithHeader.yaw       = m_setpoint[3];
	response.setpointWithHeader.mass      = m_mass_total_in_grams;
	// Return
	return true;
}






// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// void setpointCallback(const Setpoint& newSetpoint)
// {
//     m_setpoint[0] = newSetpoint.x;
//     m_setpoint[1] = newSetpoint.y;
//     m_setpoint[2] = newSetpoint.z;
//     m_setpoint[3] = newSetpoint.yaw;
// }


// void publishCurrentSetpoint()
// {
// 	Setpoint msg_setpoint;
//     msg_setpoint.x   = m_setpoint[0];
//     msg_setpoint.y   = m_setpoint[1];
//     msg_setpoint.z   = m_setpoint[2];
//     msg_setpoint.yaw = m_setpoint[3];

//     pickerSetpointToGUIPublisher.publish(msg_setpoint);
// }





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
// void buttonPressedCallback(const std_msgs::Int32& msg)
// {
// 	// Extract the data from the message
// 	int button_index = msg.data;

// 	// Switch between the button pressed
// 	switch(button_index)
// 	{
// 		case PICKER_BUTTON_GOTOSTART:
// 			buttonPressed_gotoStart();
// 			break;
// 		case PICKER_BUTTON_ATTACH:
// 			buttonPressed_attach();
// 			break;
// 		case PICKER_BUTTON_PICKUP:
// 			buttonPressed_pickup();
// 			break;
// 		case PICKER_BUTTON_GOTOEND:
// 			buttonPressed_gotoEnd();
// 			break;
// 		case PICKER_BUTTON_PUTDOWN:
// 			buttonPressed_putdown();
// 			break;
// 		case PICKER_BUTTON_SQUAT:
// 			buttonPressed_squat();
// 			break;
// 		case PICKER_BUTTON_JUMP:
// 			buttonPressed_jump();
// 			break;
// 		case PICKER_BUTTON_1:
// 			buttonPressed_1();
// 			break;
// 		case PICKER_BUTTON_2:
// 			buttonPressed_2();
// 			break;
// 		case PICKER_BUTTON_3:
// 			buttonPressed_3();
// 			break;
// 		case PICKER_BUTTON_4:
// 			buttonPressed_4();
// 			break;
// 		default:
// 		{
// 			// Let the user know that the command was not recognised
// 			ROS_INFO_STREAM("[PICKER CONTROLLER] A button pressed message was received in the controller but not recognised, message.data = " << button_index );
// 			break;
// 		}
// 	}
// }



// void buttonPressedWithSetpointCallback(const SetpointV2& newSetpointV2)
// {
// 	// Extract the "buttonID" from the message
// 	int button_index = newSetpointV2.buttonID;

// 	// Switch between the button pressed
// 	switch(button_index)
// 	{
// 		case PICKER_BUTTON_GOTOSTART:
// 			buttonPressedWithSetpoint_gotoStart(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_ATTACH:
// 			buttonPressedWithSetpoint_attach(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_PICKUP:
// 			buttonPressedWithSetpoint_pickup(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_GOTOEND:
// 			buttonPressedWithSetpoint_gotoEnd(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_PUTDOWN:
// 			buttonPressedWithSetpoint_putdown(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_SQUAT:
// 			buttonPressedWithSetpoint_squat(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_JUMP:
// 			buttonPressedWithSetpoint_jump(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_1:
// 			buttonPressedWithSetpoint_1(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_2:
// 			buttonPressedWithSetpoint_2(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_3:
// 			buttonPressedWithSetpoint_3(newSetpointV2);
// 			break;
// 		case PICKER_BUTTON_4:
// 			buttonPressedWithSetpoint_4(newSetpointV2);
// 			break;
// 		default:
// 		{
// 			// Let the user know that the command was not recognised
// 			ROS_INFO_STREAM("[PICKER CONTROLLER] A button pressed message was received in the controller but not recognised, message.data = " << button_index );
// 			break;
// 		}
// 	}
// }



//  ************************************************************************************************
//  PPPP   U   U  BBBB   L      III   SSSS  H  H       X   X  Y   Y  ZZZZZ     Y   Y    A    W     W
//  P   P  U   U  B   B  L       I   S      H  H        X X    Y Y      Z       Y Y    A A   W     W
//  PPPP   U   U  BBBB   L       I    SSS   HHHH         X      Y      Z         Y    A   A  W     W
//  P      U   U  B   B  L       I       S  H  H        X X     Y     Z          Y    AAAAA   W W W
//  P       UUU   BBBB   LLLLL  III  SSSS   H  H       X   X    Y    ZZZZZ       Y    A   A    W W

// PUBLISH THE CURRENT X,Y,Z, AND YAW (if required)
void publish_current_xyz_yaw(float x, float y, float z, float yaw)
{
	// // publish setpoint for FollowController of another student group
	// SetpointWithHeader temp_current_xyz_yaw;
	// // Fill in the x,y,z, and yaw info directly from the data provided to this
	// // function
	// temp_current_xyz_yaw.x   = x;
	// temp_current_xyz_yaw.y   = y;
	// temp_current_xyz_yaw.z   = z;
	// temp_current_xyz_yaw.yaw = yaw;
	// m_current_xyz_yaw_publisher.publish(temp_current_xyz_yaw);
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
void isReadyPickerControllerYamlCallback(const IntWithHeader & msg)
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
				ROS_INFO("[PICKER CONTROLLER] Now fetching the PickerController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[PICKER CONTROLLER] Now fetching the PickerController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[PICKER CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchPickerControllerYamlParameters(nodeHandle_to_use);
	}
}


void fetchPickerControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// StudentController.yaml

	// Add the "PickerController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "PickerController");



	// GET THE PARAMETERS:

	// // // > The mass of the letters
	// m_mass_E_grams = getParameterFloat(nodeHandle_for_paramaters , "mass_E");
	// m_mass_T_grams = getParameterFloat(nodeHandle_for_paramaters , "mass_T");
	// m_mass_H_grams = getParameterFloat(nodeHandle_for_paramaters , "mass_H");

	// // Thickness of the object at pick-up and put-down, in [meters]
	// // > This should also account for extra height due to the surface where the object is
	// m_thickness_of_object_at_pickup  = getParameterFloat(nodeHandle_for_paramaters , "thickness_of_object_at_pickup");
	// m_thickness_of_object_at_putdown = getParameterFloat(nodeHandle_for_paramaters , "thickness_of_object_at_putdown");

	// // (x,y) coordinates of the pickup location
	// getParameterFloatVector(nodeHandle_for_paramaters, "pickup_coordinates_xy", m_pickup_coordinates_xy, 2);

	// // (x,y) coordinates of the drop off location
	// getParameterFloatVector(nodeHandle_for_paramaters, "dropoff_coordinates_xy_for_E", m_dropoff_coordinates_xy_for_E, 2);
	// getParameterFloatVector(nodeHandle_for_paramaters, "dropoff_coordinates_xy_for_T", m_dropoff_coordinates_xy_for_T, 2);
	// getParameterFloatVector(nodeHandle_for_paramaters, "dropoff_coordinates_xy_for_H", m_dropoff_coordinates_xy_for_H, 2);

	// // Length of the string from the Crazyflie to the end of the Picker, in [meters]
	// m_picker_string_length = getParameterFloat(nodeHandle_for_paramaters , "picker_string_length");

	// Max setpoint change per second
	yaml_max_setpoint_change_per_second_horizontal = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_change_per_second_horizontal");
	yaml_max_setpoint_change_per_second_vertical = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_change_per_second_vertical");
	yaml_max_setpoint_change_per_second_yaw_degrees = getParameterFloat(nodeHandle_for_paramaters , "max_setpoint_change_per_second_yaw_degrees");

	

	// ------------------------------------------------------
	// PARAMTERS THAT ARE STANDARD FOR A "CONTROLLER SERVICE"

	// > The mass of the crazyflie
	yaml_mass_cf_in_grams = getParameterFloat(nodeHandle_for_paramaters , "mass_cf_in_grams");

	// > The frequency at which the "computeControlOutput" is being called, as determined
	//   by the frequency at which the Vicon system provides position and attitude data
	yaml_control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit motor command to
	//   thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// > The boolean for whether to execute the convert into body frame function
	yaml_shouldPerformConvertIntoBodyFrame = getParameterBool(nodeHandle_for_paramaters, "shouldPerformConvertIntoBodyFrame");

	// // > The boolean indicating whether the (x,y,z,yaw) of this agent should be published
	// //   or not
	// shouldPublishCurrent_xyz_yaw = getParameterBool(nodeHandle_for_paramaters, "shouldPublishCurrent_xyz_yaw");

	// Boolean indiciating whether the "Debug Message" of this agent should be published or not
	yaml_shouldPublishDebugMessage = getParameterBool(nodeHandle_for_paramaters, "shouldPublishDebugMessage");

	// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
	yaml_shouldDisplayDebugInfo = getParameterBool(nodeHandle_for_paramaters, "shouldDisplayDebugInfo");

	// A flag for which estimator to use:
	yaml_estimator_method = getParameterInt( nodeHandle_for_paramaters , "estimator_method" );

	// The LQR Controller parameters for "LQR_MODE_RATE"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixThrust_NineStateVector", yaml_gainMatrixThrust_NineStateVector, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollRate",               yaml_gainMatrixRollRate,               9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchRate",              yaml_gainMatrixPitchRate,              9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixYawRate",                yaml_gainMatrixYawRate,                9);
		
	// 16-BIT COMMAND LIMITS
	yaml_cmd_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_cmd_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");

	// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
	// > For the (x,y,z) position
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row1_for_positions",  yaml_PMKF_Ahat_row1_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row2_for_positions",  yaml_PMKF_Ahat_row2_for_positions,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Kinf_for_positions"     ,  yaml_PMKF_Kinf_for_positions     ,  2);
	// > For the (roll,pitch,yaw) angles
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row1_for_angles",  yaml_PMKF_Ahat_row1_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Ahat_row2_for_angles",  yaml_PMKF_Ahat_row2_for_angles,  2);
	getParameterFloatVector(nodeHandle_for_paramaters, "PMKF_Kinf_for_angles"     ,  yaml_PMKF_Kinf_for_angles     ,  2);


	// DEBUGGING: Print out one of the parameters that was loaded
	ROS_INFO_STREAM("[PICKER CONTROLLER] DEBUGGING: the fetched PickerController/mass_cf_in_grams = " << yaml_mass_cf_in_grams);



	// PROCESS THE PARAMTERS
	// Convert from degrees to radians
	m_max_setpoint_change_per_second_yaw_radians = DEG2RAD * yaml_max_setpoint_change_per_second_yaw_degrees;

    // Set that the estimator frequency is the same as the control frequency
    m_estimator_frequency = yaml_control_frequency;

}



//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------


// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
// {
//     float val;
//     if(!nodeHandle.getParam(name, val))
//     {
//         ROS_ERROR_STREAM("missing parameter '" << name << "'");
//     }
//     return val;
// }
// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
// {
//     if(!nodeHandle.getParam(name, val)){
//         ROS_ERROR_STREAM("missing parameter '" << name << "'");
//     }
//     if(val.size() != length) {
//         ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
//     }
// }
// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
// {
//     int val;
//     if(!nodeHandle.getParam(name, val))
//     {
//         ROS_ERROR_STREAM("missing parameter '" << name << "'");
//     }
//     return val;
// }
// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
// {
//     if(!nodeHandle.getParam(name, val)){
//         ROS_ERROR_STREAM("missing parameter '" << name << "'");
//     }
//     if(val.size() != length) {
//         ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
//     }
// }
// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
// {
//     if(!nodeHandle.getParam(name, val)){
//         ROS_ERROR_STREAM("missing parameter '" << name << "'");
//     }
//     return val.size();
// }
// // This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
// bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name)
// {
//     bool val;
//     if(!nodeHandle.getParam(name, val))
//     {
//         ROS_ERROR_STREAM("missing parameter '" << name << "'");
//     }
//     return val;
// }
    





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
    ros::init(argc, argv, "PickerControllerService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the namespace of this "PickerControllerService" node
    std::string m_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[PICKER CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "PPSClient" node.
	// > Thus, to get access to this "studentID" paremeter, we first
	//   need to get a handle to the "PPSClient" node within which this
	//   controller service is nested.


    // Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/PPSClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[PICKER CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[PICKER CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
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
	ROS_INFO_STREAM("[PICKER CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[PICKER CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber pickerContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "PickerController", 1, isReadyPickerControllerYamlCallback);
	ros::Subscriber pickerContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("PickerController", 1, isReadyPickerControllerYamlCallback);



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
	loadYamlFromFilenameCall.request.stringWithHeader.data = "PickerController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyStudentControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
		// Inform the user
		ROS_ERROR("[STUDENT CONTROLLER] The request load yaml file service call failed.");
	}





    // INITIALISE OTHER VARIABLES AS REQUIRED
    m_mass_total_in_grams = yaml_mass_cf_in_grams;
    m_weight_total_in_newtons = m_mass_total_in_grams * (9.81/1000.0);





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
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("PickerControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

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
    // variable that advertises the service called "PickerController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("PickerController", calculateControlOutput);

    // Create a "ros::NodeHandle" type local variable "namespace_nodeHandle" that points
    // to the name space of this node, i.e., "d_fall_pps" as specified by the line:
    //     "using namespace d_fall_pps;"
    // in the "DEFINES" section at the top of this file.
    //ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());

    // Instantiate the instance variable "my_current_xyz_yaw_publisher" to be a "ros::Publisher"
    // that advertises under the name "<my_agentID>/my_current_xyz_yaw_topic" where <my_agentID>
    // is filled in with the student ID number of this computer. The messages published will
    // have the structure defined in the file "Setpoint.msg" (located in the "msg" folder).
    //my_current_xyz_yaw_publisher = nodeHandle.advertise<Setpoint>("my_current_xyz_yaw_topic", 1);

    // Instantiate the local variable "customCommandSubscriber" to be a "ros::Subscriber"
    // type variable that subscribes to the "StudentCustomButton" topic and calls the class
    // function "customCommandReceivedCallback" each time a messaged is received on this topic
    // and the message received is passed as an input argument to the callback function.
    //ros::Subscriber buttonPressedCallbackSubscriber = nodeHandle.subscribe("GUIButton", 1, customCommandReceivedCallback);



    // // ADDED FOR THE PICKER
    // ros::Subscriber pickerButtonPressedSubscriber  =  nodeHandle.subscribe("ButtonPressed", 1, buttonPressedCallback);
    // ros::Subscriber pickerZSetpointSubscriber      =  nodeHandle.subscribe("ZSetpoint", 1, zSetpointCallback);
    // ros::Subscriber pickerYawSetpointSubscriber    =  nodeHandle.subscribe("YawSetpoint", 1, yawSetpointCallback);
    // ros::Subscriber pickerMassSubscriber           =  nodeHandle.subscribe("Mass", 1, massCallback);
    // ros::Subscriber pickerXAdjustmentSubscriber    =  nodeHandle.subscribe("XAdjustment", 1, xAdjustmentCallback);
    // ros::Subscriber pickerYAdjustmentSubscriber    =  nodeHandle.subscribe("YAdjustment", 1, yAdjustmentCallback);

    // pickerSetpointToGUIPublisher = nodeHandle.advertise<Setpoint>("SetpointToGUI", 1);

    // ros::Subscriber pickerButtonPressedWithSetpointSubscriber  =  nodeHandle.subscribe("ButtonPressedWithSetpoint", 1, buttonPressedWithSetpointCallback);



    // Print out some information to the user.
    ROS_INFO("[Picker CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
