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

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>

// Include the DFALL message types
#include "d_fall_pps/IntWithHeader.h"
//#include "d_fall_pps/StringWithHeader.h"
#include "d_fall_pps/SetpointWithHeader.h"
//#include "d_fall_pps/CustomButtonWithHeader.h"
#include "d_fall_pps/ViconData.h"
//#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/Controller.h"
#include "d_fall_pps/DebugMsg.h"

//the generated structs from the msg-files have to be included
// #include "d_fall_pps/Setpoint.h"
// #include "d_fall_pps/SetpointV2.h"
// #include "d_fall_pps/ControlCommand.h"
// #include "d_fall_pps/CustomButton.h"

// Include the DFALL service types
#include "d_fall_pps/LoadYamlFromFilename.h"
#include "d_fall_pps/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"
#include "nodes/PickerContorllerConstants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Needed for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>





// Namespacing the package
using namespace d_fall_pps;





//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.

// FOR WHICH BUTTON WAS PRESSED IN THE PICKER CONTOLLER
// #define PICKER_BUTTON_GOTOSTART     1
// #define PICKER_BUTTON_ATTACH        2
// #define PICKER_BUTTON_PICKUP        3
// #define PICKER_BUTTON_GOTOEND       4
// #define PICKER_BUTTON_PUTDOWN       5
// #define PICKER_BUTTON_SQUAT         6
// #define PICKER_BUTTON_JUMP          7

// #define PICKER_BUTTON_1             11
// #define PICKER_BUTTON_2             12
// #define PICKER_BUTTON_3             13
// #define PICKER_BUTTON_4             14


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
// #define CF_COMMAND_TYPE_MOTOR   6
// #define CF_COMMAND_TYPE_RATE    7
// #define CF_COMMAND_TYPE_ANGLE   8

// These constants define the method used for estimating the Inertial
// frame state.
// All methods are run at all times, this flag indicates which estimate
// is passed onto the controller.
// The following is a short description about each mode:
//
// ESTIMATOR_METHOD_FINITE_DIFFERENCE
//       Takes the poisition and angles directly as measured,
//       and estimates the velocities as a finite different to the
//       previous measurement
//
// ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION
//       Uses a 2nd order random walk estimator independently for
//       each of (x,y,z,roll,pitch,yaw)
//
// ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED
//       Uses the model of the quad-rotor and the previous inputs
//
// #define ESTIMATOR_METHOD_FINITE_DIFFERENCE          1
// #define ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION   2   // (DEFAULT)
// #define ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED      3





//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------



// ----------------------------------------
// VARIABLES SPECIFIC TO THE PICKER SERVICE

// The current state of the picker, i.e.,
// {goto start, attach, lift up, goto end, put down,
//  squat, jump, standby}
int m_picker_current_state = PICKER_STATE_STANDBY;

// Current time
//int m_time_ticks = 0;
//float m_time_seconds;

// > Total mass of the Crazyflie plus whatever it is carrying, in [grams]
float m_mass_total_in_grams;
float m_weight_total_in_newtons;

// The setpoint to be tracked, the ordering is (x,y,z,yaw),
// with units [meters,meters,meters,radians]
std::vector<float>  m_setpoint{0.0,0.0,0.4,0.0};

// The setpoint that is actually used by the controller, this
// differs from the setpoint when smoothing is turned on
std::vector<float> m_setpoint_for_controller[4] = {0.0,0.0,0.4,0.0};

// Boolean for whether to limit rate of change of the setpoint
bool m_shouldSmoothSetpointChanges = true;

// Max setpoint change per second
float yaml_max_setpoint_change_per_second_horizontal;
float yaml_max_setpoint_change_per_second_vertical;
float yaml_max_setpoint_change_per_second_yaw_degrees;
float m_max_setpoint_change_per_second_yaw_radians;





// ------------------------------------------------------
// VARIABLES THAT ARE STANDARD FOR A "CONTROLLER SERVICE"

// The ID of the agent that this node is monitoring
int m_agentID;

// The ID of the agent that can coordinate this node
int m_coordID;

// NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;



// VARIABLES FOR THE CONTOLLER

// > the mass of the crazyflie, in [grams]
float yaml_mass_cf_in_grams = 25.0;

// > the coefficients of the 16-bit command to thrust conversion
//std::vector<float> yaml_motorPoly(3);
std::vector<float> yaml_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};

// > the frequency at which the controller is running
float yaml_control_frequency = 200.0;

// > the default setpoint, the ordering is (x,y,z,yaw),
//   with units [meters,meters,meters,radians]
std::vector<float> yaml_default_setpoint = {0.0,0.0,0.4,0.0};

// The weight of the Crazyflie in Newtons, i.e., mg
float m_weight_cf_in_newtons = 0.0;

// The location error of the Crazyflie at the "previous" time step
std::vector<float> m_previous_stateErrorInertial = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
std::vector<float> yaml_gainMatrixThrust_NineStateVector (9,0.0);
std::vector<float> yaml_gainMatrixRollRate               (9,0.0);
std::vector<float> yaml_gainMatrixPitchRate              (9,0.0);
std::vector<float> yaml_gainMatrixYawRate                (9,0.0);

// The 16-bit command limits
float yaml_cmd_sixteenbit_min = 1000;
float yaml_cmd_sixteenbit_max = 60000;



// VARIABLES FOR THE ESTIMATOR

// Frequency at which the controller is running
float yaml_estimator_frequency = 200.0;

// > A flag for which estimator to use:
int yaml_estimator_method = ESTIMATOR_METHOD_FINITE_DIFFERENCE;
// > The current state interial estimate,
//   for use by the controller
float m_current_stateInertialEstimate[12];

// > The measurement of the Crazyflie at the "current" time step,
//   to avoid confusion
float m_current_xzy_rpy_measurement[6];

// > The measurement of the Crazyflie at the "previous" time step,
//   used for computing finite difference velocities
float m_previous_xzy_rpy_measurement[6];

// > The full 12 state estimate maintained by the finite
//   difference state estimator
float m_stateInterialEstimate_viaFiniteDifference[12];

// > The full 12 state estimate maintained by the point mass
//   kalman filter state estimator
float m_stateInterialEstimate_viaPointMassKalmanFilter[12];

// THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
// > For the (x,y,z) position
std::vector<float> yaml_PMKF_Ahat_row1_for_positions (2,0.0);
std::vector<float> yaml_PMKF_Ahat_row2_for_positions (2,0.0);
std::vector<float> yaml_PMKF_Kinf_for_positions      (2,0.0);
// > For the (roll,pitch,yaw) angles
std::vector<float> yaml_PMKF_Ahat_row1_for_angles    (2,0.0);
std::vector<float> yaml_PMKF_Ahat_row2_for_angles    (2,0.0);
std::vector<float> yaml_PMKF_Kinf_for_angles         (2,0.0);



// VARIABLES RELATING TO PERFORMING THE CONVERSION INTO BODY FRAME

// Boolean whether to execute the convert into body frame function
bool yaml_shouldPerformConvertIntoBodyFrame = false;


// VARIABLES RELATING TO THE PUBLISHING OF A DEBUG MESSAGE

// Boolean indiciating whether the "Debug Message" of this agent should be published or not
bool yaml_shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool yaml_shouldDisplayDebugInfo = false;

// ROS Publisher for debugging variables
ros::Publisher m_debugPublisher;


// VARIABLES RELATING TO COMMUNICATING THE SETPOINT

// ROS Publisher for inform the network about
// changes to the setpoin
ros::Publisher m_setpointChangedPublisher;






// // Current time
// int m_time_ticks = 0;
// float m_time_seconds;

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
// float m_max_setpoint_change_per_second_horizontal;
// float m_max_setpoint_change_per_second_vertical;
// float m_max_setpoint_change_per_second_yaw_degrees;
// float m_max_setpoint_change_per_second_yaw_radians;


// // Frequency at which the controller is running
// float m_vicon_frequency;





// THE FOLLOWING PARAMETERS ARE USED
// FOR THE LOW-LEVEL CONTROLLER

// // Frequency at which the controller is running
// float control_frequency;

// // > Coefficients of the 16-bit command to thrust conversion
// std::vector<float> motorPoly(3);

// // The LQR Controller parameters for "CONTROLLER_MODE_LQR_RATE"
// std::vector<float> gainMatrixThrust_NineStateVector (9,0.0);
// std::vector<float> gainMatrixRollRate               (9,0.0);
// std::vector<float> gainMatrixPitchRate              (9,0.0);
// std::vector<float> gainMatrixYawRate                (9,0.0);

// // The 16-bit command limits
// float cmd_sixteenbit_min;
// float cmd_sixteenbit_max;


// // VARIABLES FOR THE ESTIMATOR

// // Frequency at which the controller is running
// float estimator_frequency;

// // > A flag for which estimator to use:
// int estimator_method = ESTIMATOR_METHOD_FINITE_DIFFERENCE;
// // > The current state interial estimate,
// //   for use by the controller
// float current_stateInertialEstimate[12];

// // > The measurement of the Crazyflie at the "current" time step,
// //   to avoid confusion
// float current_xzy_rpy_measurement[6];

// // > The measurement of the Crazyflie at the "previous" time step,
// //   used for computing finite difference velocities
// float previous_xzy_rpy_measurement[6];

// // > The full 12 state estimate maintained by the finite
// //   difference state estimator
// float stateInterialEstimate_viaFiniteDifference[12];

// // > The full 12 state estimate maintained by the point mass
// //   kalman filter state estimator
// float stateInterialEstimate_viaPointMassKalmanFilter[12];

// // THE POINT MASS KALMAN FILTER (PMKF) GAINS AND ERROR EVOLUATION
// // > For the (x,y,z) position
// std::vector<float> PMKF_Ahat_row1_for_positions (2,0.0);
// std::vector<float> PMKF_Ahat_row2_for_positions (2,0.0);
// std::vector<float> PMKF_Kinf_for_positions      (2,0.0);
// // > For the (roll,pitch,yaw) angles
// std::vector<float> PMKF_Ahat_row1_for_angles    (2,0.0);
// std::vector<float> PMKF_Ahat_row2_for_angles    (2,0.0);
// std::vector<float> PMKF_Kinf_for_angles         (2,0.0);



// // VARIABLES FOR THE NAMESPACES FOR THE PARAMETER SERVICES
// // > For the paramter service of this agent
// std::string namespace_to_own_agent_parameter_service;
// // > For the parameter service of the coordinator
// std::string namespace_to_coordinator_parameter_service;


// // ROS PUBLISHER FOR SENDING OUT THE DEBUG MESSAGES
// ros::Publisher debugPublisher;


// // VARIABLES RELATING TO PERFORMING THE CONVERSION INTO BODY FRAME

// // Boolean whether to execute the convert into body frame function
// bool shouldPerformConvertIntoBodyFrame = false;


// // VARIABLES RELATING TO THE PUBLISHING OF A DEBUG MESSAGE

// // Boolean indiciating whether the "Debug Message" of this agent should be published or not
// bool shouldPublishDebugMessage = false;

// // Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
// bool shouldDisplayDebugInfo = false;


// // VARIABLES RELATING TO PUBLISHING CURRENT POSITION AND FOLLOWING ANOTHER AGENT'S
// // POSITION

// // The ID of this agent, i.e., the ID of this compute
// int my_agentID = 0;

// // Boolean indicating whether the (x,y,z,yaw) of this agent should be published or not
// // > The default behaviour is: do not publish,
// // > This varaible is changed based on parameters loaded from the YAML file
// bool shouldPublishCurrent_xyz_yaw = false;


// // ROS Publisher for my current (x,y,z,yaw) position
// ros::Publisher my_current_xyz_yaw_publisher;

// // ROS Publisher for the current setpoint
// ros::Publisher pickerSetpointToGUIPublisher;


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


// ADDED FOR THE PICKER
void perControlCycleOperations();

// CALLBACK FROM ROS MESSAGES RECEIVED
//void buttonPressedCallback(const std_msgs::Int32& msg);
// void zSetpointCallback(const std_msgs::Float32& msg);
// void yawSetpointCallback(const std_msgs::Float32& msg);
// void massCallback(const std_msgs::Float32& msg);
// void xAdjustmentCallback(const std_msgs::Float32& msg);
// void yAdjustmentCallback(const std_msgs::Float32& msg);

void buttonPressedWithSetpointCallback(const SetpointV2& newSetpointV2);


// SEPARATE CALLBACK FUNCTIONS FOR EACH BUTTON
void buttonPressed_goto_start();
void buttonPressed_attach();
void buttonPressed_lift_up();
void buttonPressed_goto_end();
void buttonPressed_put_down();
void buttonPressed_squat();
void buttonPressed_jump();
void buttonPressed_standby();

// void buttonPressed_1();
// void buttonPressed_2();
// void buttonPressed_3();
// void buttonPressed_4();


// SEPARATE CALLBACK FUNCTIONS FOR EACH BUTTON
// > WITH A SETPOINT IN THE MESSAGE
// void buttonPressedWithSetpoint_gotoStart(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_attach(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_pickup(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_gotoEnd(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_putdown(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_squat(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_jump(const SetpointV2& newSetpointV2);

// void buttonPressedWithSetpoint_1(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_2(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_3(const SetpointV2& newSetpointV2);
// void buttonPressedWithSetpoint_4(const SetpointV2& newSetpointV2);







// CONTROLLER COMPUTATIONS
// > The function that is called to "start" all estimation and control computations
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);

// > The function that smooth changes in the setpoin
void smoothSetpointChanges();

// > The various functions that implement an LQR controller
void calculateControlOutput_viaLQRforRates(float stateErrorBody[12], Controller::Request &request, Controller::Response &response);

// ESTIMATOR COMPUTATIONS
void performEstimatorUpdate_forStateInterial(Controller::Request &request);
void performEstimatorUpdate_forStateInterial_viaFiniteDifference();
void performEstimatorUpdate_forStateInterial_viaPointMassKalmanFilter();

// PUBLISHING OF THE DEBUG MESSAGE
void construct_and_publish_debug_message(Controller::Request &request, Controller::Response &response);

// TRANSFORMATION FROM INTERIAL ESTIMATE TO
// BODY FRAME ERROR
void convert_stateInertial_to_bodyFrameError(float stateInertial[12], float setpoint[4], float (&bodyFrameError)[12]);

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR
// INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[12], float (&stateBody)[12], float yaw_measured);

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);

// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint);

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw);

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);

// CUSTOM COMMAND RECEIVED CALLBACK
//void customCommandReceivedCallback(const CustomButton& commandReceived);

// PUBLISH THE CURRENT X,Y,Z, AND YAW
void publish_current_xyz_yaw(float x, float y, float z, float yaw);

// FOR LOADING THE YAML PARAMETERS
// float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name);
// void  getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length);
// int   getParameterInt(ros::NodeHandle& nodeHandle, std::string name);
// void  getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length);
// int   getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val);
// bool  getParameterBool(ros::NodeHandle& nodeHandle, std::string name);

void isReadyPickerControllerYamlCallback(const IntWithHeader & msg);
void fetchPickerControllerYamlParameters(ros::NodeHandle& nodeHandle);

//void yamlReadyForFetchCallback(const std_msgs::Int32& msg);
//void fetchYamlParameters(ros::NodeHandle& nodeHandle);
//void processFetchedParameters();

