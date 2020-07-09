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
#include "nodes/RampcControllerConstants.h"
//#include "nodes/DeepcControllerConstants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>
#include <fstream>

// Include Eigen for matrix operations
#include "Eigen/Dense"
#include "Eigen/Sparse"

// Includes required for threading
#include <mutex>
#include <boost/thread/thread.hpp>

// Include Gurobi optimization platform
#include "gurobi_c++.h"

// Include OSQP optimization platform
#include "osqp.h"

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

// NOTE ABOUT THREAD MANAGEMENT
// Variables starting with 's_' are shared between main and Deepc threads
// Mutex must be used before read/writing them
// Variables starting with 'd_' are used by Deepc thread only
// They are declared global for inter-function communication and/or for speed
// All other variables are used by main thread only

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
int m_current_state = RAMPC_CONTROLLER_STATE_STANDBY;

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
float yaml_cf_mass_in_grams = 28.0;

// > the frequency at which the controller is running
float yaml_control_frequency = 25.0;
float m_control_deltaT = 1.0 / yaml_control_frequency;

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

// Data collection max time, in minutes
float yaml_data_collection_max_time;

// HOME path used for file read/write
const string HOME = getenv("HOME");

// Data folder location, relative to HOME path
string yaml_dataFolder = "/work/D-FaLL-System/Deepc_data/";

// CSV output data folder location, relative to dataFolder
string yaml_outputFolder = "output/";

// CSV input data files location, relative to dataFolder
string yaml_thrustExcSignalFile = "thrust_exc_signal.csv";
string yaml_rollRateExcSignalFile = "rollRate_exc_signal.csv";
string yaml_pitchRateExcSignalFile = "pitchRate_exc_signal.csv";
string yaml_yawRateExcSignalFile = "yawRate_exc_signal.csv";

// Log files folder location, relative to dataFolder
string yaml_logFolder = "log/";

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

// Deepc parameters
// Flag that indicates whether to use roll and pitch angle measurements in Deepc
bool yaml_Deepc_measure_roll_pitch = true;
// Flag that activates yaw control through Deepc
bool yaml_Deepc_yaw_control = true;
// Prediction horizon in discrete time steps
int yaml_N = 25;
int s_yaml_N = yaml_N;
// Tini in discrete time steps
int s_yaml_Tini = 3;
// Output cost matrix diagonal entries (x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw)
vector<float> s_yaml_Q = {40.0, 40.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0};
// Input cost matrix diagonal entries (thurst, rollRate, pitchRate, yawRate)
vector<float> s_yaml_R = {4.0, 4.0, 4.0, 1.0};
// Terminal output cost matrix diagonal entries (x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw)
vector<float> s_yaml_P = {657.21, 657.21, 8.88, 96.92, 96.92, 0.47, 629.60, 629.60, 84.21};
// Regularization parameters
float s_yaml_lambda2_g = 0.0;
float s_yaml_lambda2_s = 1.0e7;
// Output constraints (x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw)
vector<float> s_yaml_output_min = {-4.0, -4.0, -4.0, -100, -100, -100, -PI/6, -PI/6, -PI/6};
vector<float> s_yaml_output_max = {4.0, 4.0, 4.0, 100, 100, 100, PI/6, PI/6, PI/6};
// Input constraints (thurst, rollRate, pitchRate, yawRate)
vector<float> s_yaml_input_min = {0.0, -PI, -PI, -PI};
vector<float> s_yaml_input_max = {0.6388, PI, PI, PI};

// Optimization parameters
string s_yaml_solver = "gurobi";
bool s_yaml_opt_sparse = true;
bool s_yaml_opt_verbose = false;
bool s_yaml_opt_steady_state = false;

// Parameters specific to Gurobi
bool s_yaml_grb_LogToFile = false;
bool s_yaml_grb_presolve_at_setup = false;

// Changing reference parameters
// Figure 8 amplitude, in m
float yaml_figure_8_amplitude = 0.5;
// Figure 8 frequency, in Hz
float yaml_figure_8_frequency = 0.1;
// z sine amplitude, in m
float yaml_z_sine_amplitude = 0.2;
// z sine frequency, in Hz
float yaml_z_sine_frequency = 0.1;

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

// Data collection matrix max size
int m_data_collection_max_size;

// Absolute data folder location
string m_dataFolder = HOME + yaml_dataFolder;

// Absolute CSV output data folder location
string m_outputFolder = m_dataFolder + yaml_outputFolder;

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

// Yaw rate excitation variables
float m_yawRateExcAmp_in_rad = 0.0;
MatrixXf m_yawRateExcSignal;
bool m_yawRateExcEnable = false;
int m_yawRateExcIndex = 0;

// Data collection matrices
// Variables used for excitation
MatrixXf m_u_data;
MatrixXf m_y_data;
int m_dataIndex = 0;
bool m_write_data = false;

// Variables used for general data collection
MatrixXf m_Deepc_active_setpoint = MatrixXf::Zero(4, 1);
bool m_collect_data = false;
MatrixXf m_u_data_lqr = MatrixXf::Zero(0,0);
MatrixXf m_y_data_lqr = MatrixXf::Zero(0,0);
MatrixXf m_r_data_lqr = MatrixXf::Zero(0,0);
int m_dataIndex_lqr = 0;
MatrixXf m_u_data_Deepc = MatrixXf::Zero(0,0);
MatrixXf m_uf_data_Deepc = MatrixXf::Zero(0,0);
MatrixXf m_y_data_Deepc = MatrixXf::Zero(0,0);
MatrixXf m_yf_data_Deepc = MatrixXf::Zero(0,0);
MatrixXf m_r_data_Deepc = MatrixXf::Zero(0,0);
MatrixXf m_solveTime_data_Deepc = MatrixXf::Zero(0,0);
int m_dataIndex_Deepc = 0;
int m_num_hankels = 0;

// Variables used for changing reference
bool m_changing_ref_enable = false;
float m_figure_8_frequency_rad;
float m_z_sine_frequency_rad;


// Variables shared between main and Deepc thread
float s_cf_weight_in_newtons = m_cf_weight_in_newtons;
MatrixXf s_setpoint = MatrixXf::Zero(4, 1);
MatrixXf s_Deepc_active_setpoint = MatrixXf::Zero(4, 1);
string s_dataFolder = m_dataFolder;
string s_logFolder = m_dataFolder + yaml_logFolder;
bool s_Deepc_measure_roll_pitch = true;
bool s_Deepc_yaw_control = true;
int s_num_inputs;
int s_num_outputs;
int s_Nuini;
int s_Nyini;
MatrixXf s_uini;
MatrixXf s_yini;
MatrixXf s_u_f;
MatrixXf s_y_f;
float s_solve_time;
bool s_setupDeepc_success = false;
// Variables for thread management
mutex s_Deepc_mutex;
// Flags for communication with Deepc thread
bool s_params_changed = false;
bool s_setpoint_changed = false;
bool s_setupDeepc = false;
bool s_solveDeepc = false;
// Variables used for changing reference
bool s_changing_ref_enable = m_changing_ref_enable;
float s_figure_8_amplitude = yaml_figure_8_amplitude;
float s_figure_8_frequency_rad = m_figure_8_frequency_rad;
float s_z_sine_amplitude = yaml_z_sine_amplitude;
float s_z_sine_frequency_rad = m_z_sine_frequency_rad;
float s_control_deltaT = m_control_deltaT;

// Global variables used by Deepc thread only
// Declared as global for inter-function communication and/or speed
float d_cf_weight_in_newtons = s_cf_weight_in_newtons;
MatrixXf d_setpoint = s_setpoint;
string d_dataFolder = s_dataFolder;
string d_logFolder = s_logFolder;
bool d_Deepc_measure_roll_pitch = s_Deepc_measure_roll_pitch;
bool d_Deepc_yaw_control = s_Deepc_yaw_control;
int d_Tini = s_yaml_Tini;
int d_N = s_yaml_N;
float d_lambda2_g = s_yaml_lambda2_g;
float d_lambda2_s = s_yaml_lambda2_s;
vector<float> d_Q_vec = s_yaml_Q;
vector<float> d_R_vec = s_yaml_R;
vector<float> d_P_vec = s_yaml_P;
vector<float> d_input_min_vec = s_yaml_input_min;
vector<float> d_input_max_vec = s_yaml_input_max;
vector<float> d_output_min_vec = s_yaml_output_min;
vector<float> d_output_max_vec = s_yaml_output_max;
int d_solver = RAMPC_CONTROLLER_SOLVER_GUROBI;
bool d_opt_sparse = s_yaml_opt_sparse;
bool d_opt_verbose = s_yaml_opt_verbose;
bool d_opt_steady_state = s_yaml_opt_steady_state;
bool d_grb_LogToFile = s_yaml_grb_LogToFile;
bool d_grb_presolve_at_setup = s_yaml_grb_presolve_at_setup;
int d_num_inputs;
int d_num_outputs;
int d_num_block_rows;
int d_Ng;
int d_Ns;
int d_Nuf;
int d_Nyf;
int d_Nuini;
int d_Nyini;
int d_num_opt_vars;
MatrixXf d_U_p;
MatrixXf d_U_f;
MatrixXf d_Y_p;
MatrixXf d_Y_f;
MatrixXf d_Q;
MatrixXf d_P;
MatrixXf d_R;
MatrixXf d_r;
MatrixXf d_r_gs;
MatrixXf d_A_gs;
MatrixXf d_b_gs;
MatrixXf d_gs;
MatrixXf d_lin_cost_vec_gs;
MatrixXf d_lin_cost_vec_us;
MatrixXf d_lin_cost_vec_r;
MatrixXf d_input_min;
MatrixXf d_input_max;
MatrixXf d_output_min;
MatrixXf d_output_max;
MatrixXf d_g;
MatrixXf d_uini;
MatrixXf d_yini;
bool d_setupDeepc_success = s_setupDeepc_success;
int d_DeepcOpt_status = 0;
int d_i;
int d_uf_start_i;
int d_yf_start_i;
int d_gs_start_i;
int d_us_start_i;
int d_uini_start_i;
int d_yini_start_i;
int d_r_gs_start_i;
int d_num_stat_eq_constr;
int d_num_dyn_eq_constr;
MatrixXf d_u_f;
MatrixXf d_y_f;
float d_solve_time;
// Gurobi optimization variables
GRBEnv d_grb_env;
GRBModel d_grb_model = GRBModel(d_grb_env);
GRBModel* d_grb_model_presolved;
GRBVar* d_grb_vars = 0;
GRBQuadExpr d_grb_quad_obj = 0;
GRBLinExpr d_grb_lin_obj_us = 0;
GRBLinExpr d_grb_lin_obj_r = 0;
GRBLinExpr d_grb_lin_obj_gs = 0;
GRBConstr* d_grb_dyn_constrs = 0;
// OSQP optimization variables
OSQPSettings* d_osqp_settings;
OSQPWorkspace* d_osqp_work;
MatrixXf d_osqp_q;
c_float* d_osqp_q_new;
c_float* d_osqp_l_new;
c_float* d_osqp_u_new;
// Repeat variables for Deepc gs matrix inversion thread variables
bool d_get_gs = false;
bool d_gs_inversion_complete = false;
// Variables used for changing reference
bool d_changing_ref_enable = s_changing_ref_enable;
float d_figure_8_amplitude = s_figure_8_amplitude;
float d_figure_8_frequency_rad = s_figure_8_frequency_rad;
float d_z_sine_amplitude = s_z_sine_amplitude;
float d_z_sine_frequency_rad = s_z_sine_frequency_rad;
float d_figure_8_scale;
float d_time_in_seconds;
float d_control_deltaT = s_control_deltaT;

// Variables shared between Deepc thread and Deepc gs matrix inversion thread
MatrixXf ds_A_gs;
MatrixXf ds_b_gs;
MatrixXf ds_gs;
// Variables for thread management
mutex ds_Deepc_gs_inversion_mutex;
// Flags for communication with Deepc thread
bool ds_get_gs = false;
bool ds_gs_inversion_complete = false;

// Deepc related global variables used by main thread only
// Declared as global for speed
int m_num_inputs = s_num_inputs;
MatrixXf m_uini;
MatrixXf m_yini;
bool m_Deepc_solving_first_opt = false;
int m_Deepc_cycles_since_solve = 0;
MatrixXf m_u_f;
MatrixXf m_y_f;
float m_solve_time;

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


// MPC VARIABLES
MatrixXf d_mpc_q;
c_float* d_mpc_q_new;
c_float* d_mpc_l_new;
c_float* d_mpc_u_new;
MatrixXf m_previous_xyz = MatrixXf::Zero(3, 1);
MatrixXf m_current_state_estimate = MatrixXf::Zero(8, 1);
MatrixXf d_current_state_estimate = MatrixXf::Zero(8, 1);
MatrixXf s_current_state_estimate = MatrixXf::Zero(8, 1);
// MPC FUNCTIONS
void change_Deepc_setpoint_mpc();
void change_Deepc_setpoint_mpc_changing_ref();
void setup_Deepc_mpc();
void solve_Deepc_mpc();

// DEEPC FUNCTIONS
void Deepc_thread_main();
void change_Deepc_params();
void change_Deepc_setpoint_gurobi();
void change_Deepc_setpoint_gurobi_changing_ref();
void change_Deepc_setpoint_osqp();
void change_Deepc_setpoint_osqp_changing_ref();
void setup_Deepc_gurobi();
void setup_Deepc_osqp();
void solve_Deepc_gurobi();
void solve_Deepc_osqp();

// DEEPC HELPER FUNCTIONS
// UPDATE UINI YINI
void update_uini_yini(Controller::Request &request, control_output &output);
// GET U_DATA FROM FILE
MatrixXf get_u_data();
// GET Y_DATA FROM FILE
MatrixXf get_y_data();
// GET VARIABLE LENGTHS
void get_variable_lengths(const MatrixXf& u_data, const MatrixXf& y_data);
// GET HANKEL MATICES
void get_hankel_matrices(const MatrixXf& u_data, const MatrixXf& y_data);
// GET COST MATRICES
void get_cost_matrices();
// GET INPUT/OUTPUT CONSTRAINT VECTORS
void get_input_output_constr_vectors();
// GET OPTIMIZATION QUADRATIC COST MATRIX
MatrixXf get_quad_cost_matrix();
// GET OPTIMIZATION LINEAR COST VECTORS
void get_lin_cost_vectors();
// UPDATE OPTIMIZATION LINEAR COST VECTORS
void update_lin_cost_vectors();
// UPDATE OPTIMIZATION LINEAR COST VECTORS WHEN REFERENCE IS CHANGING
void update_lin_cost_vectors_changing_ref();
// GET STATIC EQUALITY CONSTRAINTS MATRIX
MatrixXf get_static_eq_constr_matrix();
// GET STATIC EQUALITY CONSTRAINTS VECTOR
MatrixXf get_static_eq_constr_vector();
// GET DYNAMIC EQUALITY CONSTRAINTS MATRIX
MatrixXf get_dynamic_eq_constr_matrix();
// GET DYNAMIC EQUALITY CONSTRAINTS VECTOR
MatrixXf get_dynamic_eq_constr_vector();
// SOME STEPS TO FINISH DEEPC SETUP
void finish_Deepc_setup();
// CLEAR SETUP DEEPC SUCCESS FLAG
void clear_setupDeepc_success_flag();
// GUROBI CLEANUP
void gurobi_cleanup();
// OSQP CLEANUP FUNCTIONS
void osqp_extended_cleanup();
void osqp_cleanup_data(OSQPData* data);
// DATA TO HANKEL
MatrixXf data2hankel(const MatrixXf& data, int num_block_rows);
// CONVERT EIGEN SPARSE MATRIX TO CSC FORMAT USED IN OSQP
csc* eigen2csc(const MatrixXf& eigen_dense_mat);
// READ/WRITE CSV FILES
MatrixXf read_csv(const string& path);
bool write_csv(const string& path, const MatrixXf& M);

// DEEPC GS MATRIX INVERSION THREAD MAIN
void Deepc_gs_inversion_thread_main();

// CONTROLLER COMPUTATIONS
bool calculateControlOutput(Controller::Request &request, Controller::Response &response);
void computeResponse_for_standby(Controller::Request &request, Controller::Response &response);
void computeResponse_for_LQR(Controller::Request &request, Controller::Response &response);
void computeResponse_for_excitation_LQR(Controller::Request &request, Controller::Response &response);
void computeResponse_for_Deepc(Controller::Request &request, Controller::Response &response);
void computeResponse_for_excitation_Deepc(Controller::Request &request, Controller::Response &response);
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
void processCustomButton4(float float_data, int int_data, bool* bool_data);
void processCustomButton5(float float_data, int int_data, bool* bool_data);

// FOR LOADING THE YAML PARAMETERS
void isReadyDeepcControllerYamlCallback(const IntWithHeader& msg);
void fetchDeepcControllerYamlParameters(ros::NodeHandle& nodeHandle);
