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
//    ROS node that manages the student's setup.
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

#include "ros/ros.h"
#include <stdlib.h>
#include <std_msgs/String.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "d_fall_pps/IntWithHeader.h"
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/Setpoint.h"

// Include the DFALL service types
#include "d_fall_pps/Controller.h"
#include "d_fall_pps/CMQuery.h"

// Include the Parameter Service shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>

#include "d_fall_pps/ControlCommand.h"





//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------



// Namespacing the package
using namespace d_fall_pps;





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
std::string m_namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string m_namespace_to_coordinator_parameter_service;


// The safe controller specified in the ClientConfig.yaml
ros::ServiceClient safeController;
// The Demo controller specified in the ClientConfig.yaml
ros::ServiceClient demoController;
// The Student controller specified in the ClientConfig.yaml
ros::ServiceClient studentController;
// The MPC controller specified in the ClientConfig.yaml
ros::ServiceClient mpcController;
// The Remote controller specified in the ClientConfig.yaml
ros::ServiceClient remoteController;
// The Tuning controller specified in the ClientConfig.yaml
ros::ServiceClient tuningController;
// The Picker controller specified in the ClientConfig.yaml
ros::ServiceClient pickerController;


//values for safteyCheck
bool yaml_strictSafety;
float yaml_angleMargin;




// battery threshold
//float m_battery_threshold_while_flying;
//float m_battery_threshold_while_motors_off;


// battery values

//int m_battery_state;
//float m_battery_voltage;





Setpoint controller_setpoint;

std::vector<float> yaml_default_setpoint = {0.0f, 0.0f, 0.0f, 0.0f};

// variables for linear trayectory
Setpoint current_safe_setpoint;
double distance;
double unit_vector[3];
bool was_in_threshold = false;
double yaml_distance_threshold;      //to be loaded from yaml


ros::ServiceClient centralManager;
ros::Publisher controlCommandPublisher;

// communicate with safeControllerService, setpoint, etc...
ros::Publisher safeControllerServiceSetpointPublisher;

// publisher for flying state
ros::Publisher flyingStatePublisher;

// publisher for battery state
ros::Publisher batteryStatePublisher;

// publisher to send commands to itself.
ros::Publisher commandPublisher;

// communication with crazyRadio node. Connect and disconnect
ros::Publisher crazyRadioCommandPublisher;


// Variable for the namespaces for the paramter services
// > For the paramter service of this agent
std::string namespace_to_own_agent_parameter_service;
// > For the parameter service of the coordinator
std::string namespace_to_coordinator_parameter_service;


// variables for the states:
int flying_state;
bool changed_state_flag;

// variable for crazyradio status
int crazyradio_status;

//describes the area of the crazyflie and other parameters
CrazyflieContext context;

//wheter to use safe of demo controller
int instant_controller;         //variable for the instant controller, e.g., we use safe controller for taking off and landing even if demo controller is enabled. This variable WILL change automatically

// controller used:
int controller_used;

ros::Publisher controllerUsedPublisher;



float yaml_take_off_distance;
float yaml_landing_distance;
float yaml_duration_take_off;
float yaml_duration_landing;

bool finished_take_off = false;
bool finished_land = false;

ros::Timer timer_takeoff;
ros::Timer timer_land;

// A ROS "bag" to store data for post-analysis
//rosbag::Bag bag;




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


// > For the LOAD PARAMETERS
void yamlReadyForFetchCallback(const std_msgs::Int32& msg);
void fetchYamlParametersForSafeController(ros::NodeHandle& nodeHandle);
void fetchClientConfigParameters(ros::NodeHandle& nodeHandle);







void viconCallback(const ViconData& viconData);





// > For the {dis/re}-connect command received from the coordinator
//void crazyRadioCommandAllAgentsCallback(const std_msgs::Int32& msg);
void crazyRadioCommandAllAgentsCallback(const std_msgs::Int32& msg);




void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg);




void commandCallback(const IntWithHeader & commandMsg);




void loadSafeController();
void loadDemoController();
void loadStudentController();
void loadMpcController();
void loadRemoteController();
void loadTuningController();
void loadPickerController();

void sendMessageUsingController(int controller);
void setInstantController(int controller);
int getInstantController();
void setControllerUsed(int controller);
int getControllerUsed();


// > For the BATTERY
//int getBatteryState();
//void setBatteryStateTo(int new_battery_state);
//float movingAverageBatteryFilter(float new_input);
//void CFBatteryCallback(const std_msgs::Float32& msg);
void batteryMonitorStateChangedCallback(std_msgs::Int32 msg);


// > For the LOADING of YAML PARAMETERS
void isReadySafeControllerYamlCallback(const IntWithHeader & msg);
void fetchSafeControllerYamlParameters(ros::NodeHandle& nodeHandle);

void isReadyClientConfigYamlCallback(const IntWithHeader & msg);
void fetchClientConfigYamlParameters(ros::NodeHandle& nodeHandle);





