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
#include "dfall_pkg/IntWithHeader.h"
//#include "dfall_pkg/StringWithHeader.h"
#include "dfall_pkg/SetpointWithHeader.h"
//#include "dfall_pkg/CustomButtonWithHeader.h"
#include "dfall_pkg/ViconData.h"
//#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/Controller.h"
#include "dfall_pkg/DebugMsg.h"

// Include the DFALL service types
#include "dfall_pkg/LoadYamlFromFilename.h"
#include "dfall_pkg/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"
#include "nodes/PickerControllerConstants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Needed for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>





// Namespacing the package
using namespace dfall_pkg;





//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.

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
#define ESTIMATOR_METHOD_FINITE_DIFFERENCE          1
#define ESTIMATOR_METHOD_POINT_MASS_PER_DIMENSION   2   // (DEFAULT)
#define ESTIMATOR_METHOD_QUADROTOR_MODEL_BASED      3





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
float m_setpoint[4] = {0.0,0.0,0.4,0.0};

// The setpoint that is actually used by the controller, this
// differs from the setpoint when smoothing is turned on
float m_setpoint_for_controller[4] = {0.0,0.0,0.4,0.0};

// Boolean for whether to limit rate of change of the setpoint
bool m_shouldSmoothSetpointChanges = true;

// Max setpoint change per second
float yaml_max_setpoint_change_per_second_horizontal = 0.1;
float yaml_max_setpoint_change_per_second_vertical = 0.1;
float yaml_max_setpoint_change_per_second_yaw_degrees = 90.0;
float m_max_setpoint_change_per_second_yaw_radians = 90.0 * DEG2RAD;





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
float yaml_mass_cf_in_grams = 30.0;

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
std::vector<float> yaml_gainMatrixThrust_NineStateVector  =  { 0.00, 0.00, 0.98, 0.00, 0.00, 0.25, 0.00, 0.00, 0.00};
std::vector<float> yaml_gainMatrixRollRate                =  { 0.00,-6.20, 0.00, 0.00,-3.00, 0.00, 5.20, 0.00, 0.00};
std::vector<float> yaml_gainMatrixPitchRate               =  { 6.20, 0.00, 0.00, 3.00, 0.00, 0.00, 0.00, 5.20, 0.00};
std::vector<float> yaml_gainMatrixYawRate                 =  { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.30};


// The 16-bit command limits
float yaml_cmd_sixteenbit_min = 1000;
float yaml_cmd_sixteenbit_max = 60000;



// VARIABLES FOR THE ESTIMATOR

// Frequency at which the controller is running
float m_estimator_frequency = 200.0;

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
std::vector<float> yaml_PMKF_Ahat_row1_for_positions  =  {  0.6723, 0.0034};
std::vector<float> yaml_PMKF_Ahat_row2_for_positions  =  {-12.9648, 0.9352};
std::vector<float> yaml_PMKF_Kinf_for_positions       =  {  0.3277,12.9648};
// > For the (roll,pitch,yaw) angles
std::vector<float> yaml_PMKF_Ahat_row1_for_angles     =  {  0.6954, 0.0035};
std::vector<float> yaml_PMKF_Ahat_row2_for_angles     =  {-11.0342, 0.9448};
std::vector<float> yaml_PMKF_Kinf_for_angles          =  {  0.3046,11.0342};



// VARIABLES RELATING TO PERFORMING THE CONVERSION INTO BODY FRAME

// Boolean whether to execute the convert into body frame function
bool yaml_shouldPerformConvertIntoBodyFrame = true;


// VARIABLES RELATING TO THE PUBLISHING OF A DEBUG MESSAGE

// Boolean indiciating whether the "Debug Message" of this agent should be published or not
bool yaml_shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool yaml_shouldDisplayDebugInfo = false;

// ROS Publisher for debugging variables
ros::Publisher m_debugPublisher;


// VARIABLES RELATING TO COMMUNICATING THE SETPOINT

// ROS Publisher for inform the network about
// changes to the setpoint
ros::Publisher m_setpointChangedPublisher;








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


// SEPARATE CALLBACK FUNCTIONS FOR EACH BUTTON
void buttonPressed_goto_start();
void buttonPressed_attach();
void buttonPressed_lift_up();
void buttonPressed_goto_end();
void buttonPressed_put_down();
void buttonPressed_squat();
void buttonPressed_jump();
void buttonPressed_standby();




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
//void setNewSetpoint(float x, float y, float z, float yaw);
void setNewSetpoint(int state, bool should_smooth, float x, float y, float z, float yaw, float mass);

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);

// CUSTOM COMMAND RECEIVED CALLBACK
//void customCommandReceivedCallback(const CustomButton& commandReceived);

// PUBLISH THE CURRENT X,Y,Z, AND YAW
void publish_current_xyz_yaw(float x, float y, float z, float yaw);

// FOR LOADING THE YAML PARAMETERS
void timerCallback_initial_load_yaml(const ros::TimerEvent&);
void isReadyPickerControllerYamlCallback(const IntWithHeader & msg);
void fetchPickerControllerYamlParameters(ros::NodeHandle& nodeHandle);