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
#include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/ViconData.h"
#include "dfall_pkg/CrazyflieData.h"
#include "dfall_pkg/ControlCommand.h"
#include "dfall_pkg/CrazyflieContext.h"
#include "dfall_pkg/Setpoint.h"

// Include the DFALL service types
#include "dfall_pkg/Controller.h"
#include "dfall_pkg/CMQuery.h"
#include "dfall_pkg/IntIntService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"

// Need for having a ROS "bag" to store data for post-analysis
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


// variables for the states:
int m_flying_state;
bool m_changed_flying_state_flag;

// variable for crazyradio status
int crazyradio_status;

//describes the area of the crazyflie and other parameters
CrazyflieContext m_context;

// The index for which element in the "motion captate data"
// array is expected to match the name in "m_context"
// > Negative numbers indicate unknown
int m_poseDataIndex = -1;

// wheter to use safe of demo controller
// variable for the instant controller, e.g., we use
// safe controller for taking off and landing even
// if demo controller is enabled. This variable WILL change automatically
int m_instant_controller;
ros::ServiceClient* m_instant_controller_service_client;
bool m_controllers_avialble = false;
ros::Timer timer_for_loadAllControllers;

// controller used:
int m_controller_nominally_selected;


// The safe controller specified in the ClientConfig.yaml
ros::ServiceClient safeController;
// The default controller specified in the ClientConfig.yaml
ros::ServiceClient defaultController;
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
// The Template controller specified in the ClientConfig.yaml
ros::ServiceClient templateController;


// The values for the safety check on tilt angle
bool yaml_isEnabled_strictSafety = true;
float yaml_angleMargin;
float yaml_maxTiltAngle_for_strictSafety_degrees = 70;
float m_maxTiltAngle_for_strictSafety_redians = 70 * DEG2RAD;




//Setpoint controller_setpoint;

//std::vector<float> yaml_default_setpoint = {0.0f, 0.0f, 0.0f, 0.0f};

// variables for linear trayectory
//Setpoint current_safe_setpoint;
//double distance;
//double unit_vector[3];
//bool was_in_threshold = false;
//double yaml_distance_threshold;      //to be loaded from yaml


// Service Client from which the "CrazyflieContext" is loaded
ros::ServiceClient centralManager;

// Publisher for the control actions to be sent on
// to the Crazyflie (the CrazyRadio node listen to this
// publisher and actually send the commands)
// {onboardControllerType,roll,pitch,yaw,motorCmd1,motorCmd2,motorCmd3,motorCmd4}
ros::Publisher commandForSendingToCrazyfliePublisher;

// communicate with safeControllerService, setpoint, etc...
//ros::Publisher safeControllerServiceSetpointPublisher;

// Publisher for the current flying state of this Flying Agent Client
ros::Publisher flyingStatePublisher;

// Publisher for the commands of:
// {take-off,land,motors-off, and which controller to use}
//ros::Publisher commandPublisher;

// Publisher Communication with crazyRadio node. Connect and disconnect
ros::Publisher crazyRadioCommandPublisher;


// Publisher for which controller is currently being used
ros::Publisher controllerUsedPublisher;



// float yaml_take_off_distance;
// float yaml_landing_distance;
// float yaml_duration_take_off;
// float yaml_duration_landing;

// bool finished_take_off = false;
// bool finished_land = false;

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









void viconCallback(const ViconData& viconData);
int getPoseDataForObjectNameWithExpectedIndex(const ViconData& viconData, std::string name , int expected_index , CrazyflieData& pose);
void coordinatesToLocal(CrazyflieData& cf);



// > For the SAFETY CHECK on area and the angle
bool safetyCheck(CrazyflieData data, ControlCommand controlCommand);


void changeFlyingStateTo(int new_state);


void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg);




void flyingStateRequestCallback(const IntWithHeader & commandMsg);




// void loadSafeController();
// void loadDemoController();
// void loadStudentController();
// void loadMpcController();
// void loadRemoteController();
// void loadTuningController();
// void loadPickerController();
// void loadTemplateController();

void loadController( std::string paramter_name , ros::ServiceClient& serviceClient );

void sendMessageUsingController(int controller);
void setInstantController(int controller);
int getInstantController();
void setControllerNominallySelected(int controller);
int getControllerNominallySelected();



// THE CALLBACK THAT THE CRAZY RADIO STATUS CHANGED
void crazyRadioStatusCallback(const std_msgs::Int32& msg);

// THE CALLBACK THAT AN EMERGENCY STOP MESSAGE WAS RECEIVED
void emergencyStopReceivedCallback(const IntWithHeader & msg);

// THE SERVICE CALLBACK REQUESTING THE CURRENT FLYING STATE
bool getCurrentFlyingStateServiceCallback(IntIntService::Request &request, IntIntService::Response &response);

// FOR THE BATTERY STATE CALLBACK
void batteryMonitorStateChangedCallback(std_msgs::Int32 msg);

// FOR THE SAFETY CHECKS ON POSITION AND TILT ANGLE
bool safetyCheck(CrazyflieData data);

// THE CALLBACK THAT THE CONTEXT DATABASE CHANGED
void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg);

// FOR LOADING THE CONTEXT OF THIS AGENT
void loadCrazyflieContext();

// FOR LOADING THE YAML PARAMETERS
void isReadyClientConfigYamlCallback(const IntWithHeader & msg);
void fetchClientConfigYamlParameters(ros::NodeHandle& nodeHandle);