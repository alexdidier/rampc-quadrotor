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
//    A Deepc Controller for students build from
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
#include "dfall_pkg/CustomButtonWithHeader.h"
#include "dfall_pkg/ViconData.h"
#include "dfall_pkg/Setpoint.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/Controller.h"
#include "dfall_pkg/DebugMsg.h"

// Include the DFALL service types
#include "dfall_pkg/IntIntService.h"
#include "dfall_pkg/LoadYamlFromFilename.h"
#include "dfall_pkg/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"
#include "nodes/DeepcControllerConstants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>
#include <fstream>

// Include Eigen for matrix operations
#include "Eigen/Dense"

// Include Gurobi optimization platform
#include "gurobi_c++.h"

// Includes required for threading
#include <mutex>
#include <boost/thread/thread.hpp>

// Namespacing the package
using namespace dfall_pkg;
using namespace std;
using namespace Eigen;




//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// These constants are defined to make the code more readable and adaptable.

// NOTE: many constants are already defined in the
//       "Constant.h" header file



// ---------- STRUCTS ----------

// Control output structure
struct control_output
{
	float thrust;
	float rollRate;
	float pitchRate;
	float yawRate;
};


//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// The ID of the agent that this node is monitoring
int m_agentID;

// The ID of the agent that can coordinate this node
int m_coordID;

// NAMESPACES FOR THE PARAMETER SERVICES
// > For the paramter service of this agent
string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
string m_namespace_to_coordinator_parameter_service;

// STATE MACHINE VARIABLES

// The current state of the Deepc Controller
int m_current_state = DEEPC_CONTROLLER_STATE_STANDBY;

// A flag for when the state is changed, this is used
// so that a "one-off" operation can be performed
// the first time after changing that state
bool m_current_state_changed = false;

// The elapased time, incremented by counting the motion
// capture callbacks
// Used in states that require time
float m_time_in_seconds = 0.0;

// VARIABLES FOR PERFORMING THE LANDING MANOEUVRE

// Height change for the landing move-down
float yaml_landing_move_down_end_height_setpoint  = 0.05;
float yaml_landing_move_down_end_height_threshold = 0.10;
// The time for: landing move-down
float yaml_landing_move_down_time_max = 5.0;

// The thrust for landing spin motors
float yaml_landing_spin_motors_thrust = 10000;
// The time for: landing spin motors
float yaml_landing_spin_motors_time = 1.5;


// VARAIBLES FOR VALUES LOADED FROM THE YAML FILE
// > the mass of the crazyflie, in [grams]
float yaml_cf_mass_in_grams = 25.0;

// > the frequency at which the controller is running
float yaml_control_frequency = 200.0;
float m_control_deltaT = (1.0/200.0);

// > the coefficients of the 16-bit command to thrust conversion
//std::vector<float> yaml_motorPoly(3);
vector<float> yaml_motorPoly = {5.484560e-4, 1.032633e-6, 2.130295e-11};


// The min and max for saturating 16 bit thrust commands
float yaml_command_sixteenbit_min = 1000;
float yaml_command_sixteenbit_max = 60000;

// > the default setpoint, the ordering is (x,y,z,yaw),
//   with units [meters,meters,meters,radians]
vector<float> yaml_default_setpoint = {0.0,0.0,0.4,0.0};

// Boolean indiciating whether the "Debug Message" of this agent should be published or not
bool yaml_shouldPublishDebugMessage = false;

// Boolean indiciating whether the debugging ROS_INFO_STREAM should be displayed or not
bool yaml_shouldDisplayDebugInfo = false;

// The LQR Controller parameters for "LQR_RATE_MODE"
vector<float> yaml_gainMatrixThrust_NineStateVector  =  { 0.00, 0.00, 0.98, 0.00, 0.00, 0.25, 0.00, 0.00, 0.00};
vector<float> yaml_gainMatrixRollRate                =  { 0.00,-6.20, 0.00, 0.00,-3.00, 0.00, 5.20, 0.00, 0.00};
vector<float> yaml_gainMatrixPitchRate               =  { 6.20, 0.00, 0.00, 3.00, 0.00, 0.00, 0.00, 5.20, 0.00};
vector<float> yaml_gainMatrixYawRate                 =  { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.30};

// HOME path used for file read/write
const string HOME = getenv("HOME");

// Data folder location, relative to HOME path
string yaml_dataFolder = "/work/D-FaLL-System/Deepc_data/";

// CSV output data folder location, relative to dataFolder
string yaml_outputFolder = "output/";

// Log files folder location, relative to dataFolder
string yaml_logFolder = "log/";

// CSV input data files location, relative to dataFolder
string yaml_thrustExcSignalFile = "thrust_exc_signal.csv";
string yaml_rollRateExcSignalFile = "rollRate_exc_signal.csv";
string yaml_pitchRateExcSignalFile = "pitchRate_exc_signal.csv";
string yaml_yawRateExcSignalFile = "yawRate_exc_signal.csv";

// Thrust excitation parameters
float yaml_thrustExcAmp_in_grams = 0.0;

// Roll rate excitation parameters
float yaml_rollRateExcAmp_in_deg = 0.0;

// Pitch rate excitation parameters
float yaml_pitchRateExcAmp_in_deg = 0.0;

// Yaw rate excitation parameters
float yaml_yawRateExcAmp_in_deg = 0.0;

// Excitation start time, in s. Used to collect steady-state data before excitation
float yaml_exc_start_time = 0.0;

// Gurobi optimization parameters
bool yaml_grb_LogToFile = false;
bool yaml_grb_LogToConsole = false;

// The weight of the Crazyflie in Newtons, i.e., mg
float m_cf_weight_in_newtons = 0.0;

// The location error of the Crazyflie at the "previous" time step
float m_previous_stateErrorInertial[9];

// The setpoint to be tracked, the ordering is (x,y,z,yaw),
// with units [meters,meters,meters,radians]
float  m_setpoint[4] = {0.0,0.0,0.4,0.0};

// The setpoint that is actually used by the controller, this
// differs from the setpoint when landing
float m_setpoint_for_controller[4] = {0.0,0.0,0.4,0.0};

// Absolute CSV output data folder location
string m_outputFolder = HOME + yaml_dataFolder + yaml_outputFolder;

// Absolute log files folder location
string m_logFolder = HOME + yaml_dataFolder + yaml_logFolder;

// Thrust excitation variables
float m_thrustExcAmp_in_newtons = 0.0;
MatrixXf m_thrustExcSignal;
bool m_thrustExcEnable = false;
int m_thrustExcIndex = 0;

// Roll rate excitation variables
float m_rollRateExcAmp_in_rad = 0.0;
MatrixXf m_rollRateExcSignal;
bool m_rollRateExcEnable = false;
int m_rollRateExcIndex = 0;

// Pitch rate excitation variables
float m_pitchRateExcAmp_in_rad = 0.0;
MatrixXf m_pitchRateExcSignal;
bool m_pitchRateExcEnable = false;
int m_pitchRateExcIndex = 0;

// Yaw rate excitation in variables
float m_yawRateExcAmp_in_rad = 0.0;
MatrixXf m_yawRateExcSignal;
bool m_yawRateExcEnable = false;
int m_yawRateExcIndex = 0;

// Data collection matrices
MatrixXf m_u_data;
MatrixXf m_y_data;
int m_dataIndex = 0;
bool m_write_data = false;

// Gurobi optimization variables
GRBEnv m_grb_env;
GRBModel m_grb_model = GRBModel(m_grb_env);
GRBVar m_grb_x;
GRBVar m_grb_y;
GRBVar m_grb_z;
bool m_grb_setup_success = false;

// Variables for thread management
mutex m_Deepc_mutex;
// Flags for communication with Deepc thread
bool m_params_changed = false;
bool m_setupDeepc = false;
bool m_solveDeepc = false;

// ROS Publisher for debugging variables
ros::Publisher m_debugPublisher;

// ROS Publisher for inform the network about
// changes to the setpoin
ros::Publisher m_setpointChangedPublisher;

// ROS Publisher to inform the flying agent client
// when a requested manoeuvre is complete
ros::Publisher m_manoeuvreCompletePublisher;




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

// These function prototypes are not strictly required for this code to
// complile, but adding the function prototypes here means the the functions
// can be written below in any order. If the function prototypes are not
// included then the function need to written below in an order that ensure
// each function is implemented before it is called from another function,
// hence why the "main" function is at the bottom.

// DEEPC FUNCTIONS
void deepc_thread_main();
void change_Deepc_params();
bool setup_Deepc();
bool solve_Deepc();

// DEEPC HELPER FUNCTIONS

// READ/WRITE CSV FILES
MatrixXf read_csv(const string & path);
bool write_csv(const string & path, MatrixXf M);

// CONTROLLER COMPUTATIONS
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);
void computeResponse_for_standby(Controller::Request &request, Controller::Response &response);
void computeResponse_for_LQR(Controller::Request &request, Controller::Response &response);
void computeResponse_for_excitation(Controller::Request &request, Controller::Response &response);
void computeResponse_for_Deepc(Controller::Request &request, Controller::Response &response);
void computeResponse_for_landing_move_down(Controller::Request &request, Controller::Response &response);
void computeResponse_for_landing_spin_motors(Controller::Request &request, Controller::Response &response);
void calculateControlOutput_viaLQR(Controller::Request &request, control_output &output);

// TRANSFORMATION OF THE (x,y) INERTIAL FRAME ERROR
// INTO AN (x,y) BODY FRAME ERROR
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured);

// CONVERSION FROM THRUST IN NEWTONS TO 16-BIT COMMAND
float computeMotorPolyBackward(float thrust);

// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint);

// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw);

// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response);

// PUBLISH THE CURRENT SETPOINT AND STATE
void publishCurrentSetpointAndState();

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived);
void processCustomButton1(float float_data, int int_data, bool* bool_data);
void processCustomButton2(float float_data, int int_data, bool* bool_data);
void processCustomButton3(float float_data, int int_data, bool* bool_data);

// FOR LOADING THE YAML PARAMETERS
void isReadyDeepcControllerYamlCallback(const IntWithHeader & msg);
void fetchDeepcControllerYamlParameters(ros::NodeHandle& nodeHandle);
