//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
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

#include "d_fall_pps/Controller.h"
#include "d_fall_pps/CMQuery.h"

#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/Setpoint.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include "d_fall_pps/ControlCommand.h"

// Need for having a ROS "bag" to store data for post-analysis
//#include <rosbag/bag.h>




//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// Types PPS firmware
#define TYPE_PPS_MOTORS 6
#define TYPE_PPS_RATE   7
#define TYPE_PPS_ANGLE  8

// Types of controllers being used:
#define SAFE_CONTROLLER   0
#define CUSTOM_CONTROLLER 1

// The constants that "command" changes in the
// operation state of this agent
#define CMD_USE_SAFE_CONTROLLER   1
#define CMD_USE_CUSTOM_CONTROLLER 2
#define CMD_CRAZYFLY_TAKE_OFF     3
#define CMD_CRAZYFLY_LAND         4
#define CMD_CRAZYFLY_MOTORS_OFF   5

// Flying states
#define STATE_MOTORS_OFF 1
#define STATE_TAKE_OFF   2
#define STATE_FLYING     3
#define STATE_LAND       4

// Battery states
#define BATTERY_STATE_NORMAL 0
#define BATTERY_STATE_LOW    1

// Commands for CrazyRadio
#define CMD_RECONNECT  0
#define CMD_DISCONNECT 1


// CrazyRadio states:
#define CONNECTED        0
#define CONNECTING       1
#define DISCONNECTED     2

// For which controller parameters to load
#define FETCH_YAML_SAFE_CONTROLLER_AGENT          1
#define FETCH_YAML_CUSTOM_CONTROLLER_AGENT        2
#define FETCH_YAML_SAFE_CONTROLLER_COORDINATOR    3
#define FETCH_YAML_CUSTOM_CONTROLLER_COORDINATOR  4


// Parameters for take off and landing. Eventually will go in YAML file
//#define TAKE_OFF_OFFSET  1    //in meters
//#define LANDING_DISTANCE 0.15    //in meters
//#define DURATION_TAKE_OFF  3   // seconds
//#define DURATION_LANDING   3   // seconds

// Universal constants
#define PI 3.141592653589

// Namespacing the package
using namespace d_fall_pps;





//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

//studentID, gives namespace and identifier in CentralManagerService
int studentID;

//the safe controller specified in the ClientConfig.yaml, is considered stable
ros::ServiceClient safeController;
//the custom controller specified in the ClientConfig.yaml, is considered potentially unstable
ros::ServiceClient customController;

//values for safteyCheck
bool strictSafety;
float angleMargin;

// battery threshold
float m_battery_threshold_while_flying;
float m_battery_threshold_while_motors_off;


// battery values

int m_battery_state;
float m_battery_voltage;

Setpoint controller_setpoint;

// variables for linear trayectory
Setpoint current_safe_setpoint;
double distance;
double unit_vector[3];
bool was_in_threshold = false;
double distance_threshold;      //to be loaded from yaml


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

//wheter to use safe of custom controller
int instant_controller;         //variable for the instant controller, e.g., we use safe controller for taking off and landing even if custom controller is enabled. This variable WILL change automatically

// controller used:
int controller_used;

ros::Publisher controllerUsedPublisher;

std::string ros_namespace;

float take_off_distance;
float landing_distance;
float duration_take_off;
float duration_landing;

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



void crazyRadioCommandAllAgentsCallback(const std_msgs::Int32& msg);



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

void loadSafeController() {
	ros::NodeHandle nodeHandle("~");

	std::string safeControllerName;
	if(!nodeHandle.getParam("safeController", safeControllerName)) {
		ROS_ERROR("Failed to get safe controller name");
		return;
	}

	ros::service::waitForService(safeControllerName);
	safeController = ros::service::createClient<Controller>(safeControllerName, true);
    ROS_INFO_STREAM("loaded safe controller: " << safeController.getService());
}

void loadCustomController()
{
	ros::NodeHandle nodeHandle("~");

	std::string customControllerName;
	if(!nodeHandle.getParam("customController", customControllerName))
    {
		ROS_ERROR("Failed to get custom controller name");
		return;
	}

    customController = ros::service::createClient<Controller>(customControllerName, true);
    ROS_INFO_STREAM("loaded custom controller " << customControllerName);
}


void sendMessageUsingController(int controller)
{
    // send a message in topic for the studentGUI to read it
    std_msgs::Int32 controller_used_msg;
    controller_used_msg.data = controller;
    controllerUsedPublisher.publish(controller_used_msg);
}

void setInstantController(int controller) //for right now, temporal use
{
    instant_controller = controller;
    sendMessageUsingController(controller);
    switch(controller)
    {
        case SAFE_CONTROLLER:
            loadSafeController();
            break;
        case CUSTOM_CONTROLLER:
            loadCustomController();
            break;
        default:
            break;
    }
}

int getInstantController()
{
    return instant_controller;
}

void setControllerUsed(int controller) //for permanent configs
{
    controller_used = controller;

    if(flying_state == STATE_MOTORS_OFF || flying_state == STATE_FLYING)
    {
        setInstantController(controller); //if motors OFF or STATE FLYING, transparent, change is instant
    }
}

int getControllerUsed()
{
    return controller_used;
}



//checks if crazyflie is within allowed area and if custom controller returns no data
bool safetyCheck(CrazyflieData data, ControlCommand controlCommand) {
	//position check
	if((data.x < context.localArea.xmin) or (data.x > context.localArea.xmax)) {
		ROS_INFO_STREAM("x safety failed");
		return false;
	}
	if((data.y < context.localArea.ymin) or (data.y > context.localArea.ymax)) {
		ROS_INFO_STREAM("y safety failed");
		return false;
	}
	if((data.z < context.localArea.zmin) or (data.z > context.localArea.zmax)) {
		ROS_INFO_STREAM("z safety failed");
		return false;
	}

	//attitude check
	//if strictSafety is set to true in ClientConfig.yaml the SafeController takes also over if the roll and pitch angles get to large
	//the angleMargin is a value in the range (0,1). The closer to 1, the closer to 90 deg are the roll and pitch angles allowed to become before the safeController takes over
	if(strictSafety){
		if((data.roll > PI/2*angleMargin) or (data.roll < -PI/2*angleMargin)) {
			ROS_INFO_STREAM("roll too big.");
			return false;
		}
		if((data.pitch > PI/2*angleMargin) or (data.pitch < -PI/2*angleMargin)) {
			ROS_INFO_STREAM("pitch too big.");
			return false;
		}
	}

	return true;
}

void coordinatesToLocal(CrazyflieData& cf) {
	AreaBounds area = context.localArea;
	float originX = (area.xmin + area.xmax) / 2.0;
	float originY = (area.ymin + area.ymax) / 2.0;
    // change Z origin to zero, i.e., to the table height, zero of global coordinates, instead of middle of the box
    float originZ = 0.0;
	// float originZ = (area.zmin + area.zmax) / 2.0;

	cf.x -= originX;
	cf.y -= originY;
	cf.z -= originZ;
}


void setCurrentSafeSetpoint(Setpoint setpoint)
{
    current_safe_setpoint = setpoint;
}

void calculateDistanceToCurrentSafeSetpoint(CrazyflieData& local)
{
    double dx = current_safe_setpoint.x - local.x;
    double dy = current_safe_setpoint.y - local.y;
    double dz = current_safe_setpoint.z - local.z;

    distance = sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2));

    unit_vector[0] = dx/distance;
    unit_vector[1] = dy/distance;
    unit_vector[2] = dz/distance;
}


void takeOffCF(CrazyflieData& current_local_coordinates) //local because the setpoint is in local coordinates
{
    // set the setpoint and call safe controller
    Setpoint setpoint_msg;
    setpoint_msg.x = current_local_coordinates.x;           // previous one
    setpoint_msg.y = current_local_coordinates.y;           //previous one
    setpoint_msg.z = current_local_coordinates.z + take_off_distance;           //previous one plus some offset
    // setpoint_msg.yaw = current_local_coordinates.yaw;          //previous one
    setpoint_msg.yaw = 0.0;
    safeControllerServiceSetpointPublisher.publish(setpoint_msg);
    ROS_INFO("Take OFF:");
    ROS_INFO("------Current coordinates:");
    ROS_INFO("X: %f, Y: %f, Z: %f", current_local_coordinates.x, current_local_coordinates.y, current_local_coordinates.z);
    ROS_INFO("------New coordinates:");
    ROS_INFO("X: %f, Y: %f, Z: %f", setpoint_msg.x, setpoint_msg.y, setpoint_msg.z);

    // now, use safe controller to go to that setpoint
    loadSafeController();
    setInstantController(SAFE_CONTROLLER);
    // when do we finish? after some time with delay?

    // update variable that keeps track of current setpoint
    setCurrentSafeSetpoint(setpoint_msg);
}

void landCF(CrazyflieData& current_local_coordinates)
{
    // set the setpoint and call safe controller
    Setpoint setpoint_msg;
    setpoint_msg.x = current_local_coordinates.x;           // previous one
    setpoint_msg.y = current_local_coordinates.y;           //previous one
    setpoint_msg.z = landing_distance;           //previous one plus some offset
    setpoint_msg.yaw = current_local_coordinates.yaw;          //previous one
    safeControllerServiceSetpointPublisher.publish(setpoint_msg);

    // now, use safe controller to go to that setpoint
    loadSafeController();
    setInstantController(SAFE_CONTROLLER);
    setCurrentSafeSetpoint(setpoint_msg);
}

void changeFlyingStateTo(int new_state)
{
    if(crazyradio_status == CONNECTED)
    {
        ROS_INFO("Change state to: %d", new_state);
        flying_state = new_state;
    }
    else
    {
        ROS_INFO("Disconnected and trying to change state. State goes to MOTORS OFF");
        flying_state = STATE_MOTORS_OFF;
    }

    changed_state_flag = true;
    std_msgs::Int32 flying_state_msg;
    flying_state_msg.data = flying_state;
    flyingStatePublisher.publish(flying_state_msg);
}


void takeOffTimerCallback(const ros::TimerEvent&)
{
    finished_take_off = true;
}

void landTimerCallback(const ros::TimerEvent&)
{
    finished_land = true;
}

void goToControllerSetpoint()
{
    safeControllerServiceSetpointPublisher.publish(controller_setpoint);
    setCurrentSafeSetpoint(controller_setpoint);
}


//is called when new data from Vicon arrives
void viconCallback(const ViconData& viconData) {
	for(std::vector<CrazyflieData>::const_iterator it = viconData.crazyflies.begin(); it != viconData.crazyflies.end(); ++it) {
		CrazyflieData global = *it;

        if(global.crazyflieName == context.crazyflieName)
        {
            Controller controllerCall;
            CrazyflieData local = global;
            coordinatesToLocal(local);
            controllerCall.request.ownCrazyflie = local;

            ros::NodeHandle nodeHandle("~");

            switch(flying_state) //things here repeat every X ms, non-blocking stuff
            {
                case STATE_MOTORS_OFF: // here we will put the code of current disabled button
                    if(changed_state_flag) // stuff that will be run only once when changing state
                    {
                        changed_state_flag = false;
                        ROS_INFO("STATE_MOTORS_OFF");
                    }
                    break;
                case STATE_TAKE_OFF: //we need to move up from where we are now.
                    if(changed_state_flag) // stuff that will be run only once when changing state
                    {
                        changed_state_flag = false;
                        takeOffCF(local);
                        finished_take_off = false;
                        ROS_INFO("STATE_TAKE_OFF");
                        timer_takeoff = nodeHandle.createTimer(ros::Duration(duration_take_off), takeOffTimerCallback, true);
                    }
                    if(finished_take_off)
                    {
                        finished_take_off = false;
                        setInstantController(getControllerUsed());
                        changeFlyingStateTo(STATE_FLYING);
                    }
                    break;
                case STATE_FLYING:
                    if(changed_state_flag) // stuff that will be run only once when changing state
                    {
                        changed_state_flag = false;
                        // need to change setpoint to the controller one
                        goToControllerSetpoint();
                        ROS_INFO("STATE_FLYING");
                    }
                    break;
                case STATE_LAND:
                    if(changed_state_flag) // stuff that will be run only once when changing state
                    {
                        changed_state_flag = false;
                        landCF(local);
                        finished_land = false;
                        ROS_INFO("STATE_LAND");
                        timer_takeoff = nodeHandle.createTimer(ros::Duration(duration_landing), landTimerCallback, true);
                    }
                    if(finished_land)
                    {
                        finished_land = false;
                        setInstantController(getControllerUsed());
                        changeFlyingStateTo(STATE_MOTORS_OFF);
                    }
                    break;
                default:
                    break;
            }

            // control part that should always be running, calls to controller, computation of control output
            if(flying_state != STATE_MOTORS_OFF)
            {
                if(!global.occluded)    //if it is not occluded, then proceed to compute the controller output.
                {
                    if(getInstantController() == CUSTOM_CONTROLLER && flying_state == STATE_FLYING) // only enter in custom controller if we are not using safe controller AND the flying state is FLYING
                    {
                        bool success = customController.call(controllerCall);
                        if(!success)
                        {
                            ROS_ERROR("Failed to call custom controller, switching to safe controller");
                            ROS_ERROR_STREAM("custom controller status: valid: " << customController.isValid() << ", exists: " << customController.exists());
                            ROS_ERROR_STREAM("custom controller name: " << customController.getService());
                            setInstantController(SAFE_CONTROLLER);
                        }
                        else if(!safetyCheck(global, controllerCall.response.controlOutput))
                        {
                            setInstantController(SAFE_CONTROLLER);
                            ROS_INFO_STREAM("safety check failed, switching to safe controller");
                        }
                    }
                    else        //SAFE_CONTROLLER and state is different from flying
                    {
                        calculateDistanceToCurrentSafeSetpoint(local); // update distance, it also updates the unit vector
                        // ROS_INFO_STREAM("distance: " << distance);
                        // here, detect if euclidean distance between setpoint and current position is higher than a threshold
                        if(distance > distance_threshold)
                        {
                            ROS_INFO("inside threshold");
                            Setpoint setpoint_msg;
                            // here, where we are now, or where we were in the beginning?
                            setpoint_msg.x = local.x + distance_threshold * unit_vector[0];
                            setpoint_msg.y = local.y + distance_threshold * unit_vector[1];
                            setpoint_msg.z = local.z + distance_threshold * unit_vector[2];

                            // yaw is better divided by the number of steps?
                            setpoint_msg.yaw = current_safe_setpoint.yaw;
                            safeControllerServiceSetpointPublisher.publish(setpoint_msg);
                            was_in_threshold = true;
                        }
                        else
                        {
                            if(was_in_threshold)
                            {
                                was_in_threshold = false;
                                safeControllerServiceSetpointPublisher.publish(current_safe_setpoint);
                                // goToControllerSetpoint(); //maybe this is a bit repetitive?
                            }
                        }

                        bool success = safeController.call(controllerCall);
                        if(!success) {
                            ROS_ERROR_STREAM("Failed to call safe controller, valid: " << safeController.isValid() << ", exists: " << safeController.exists());
                        }
                    }


                    controlCommandPublisher.publish(controllerCall.response.controlOutput);

                    // Putting data into the ROS "bag" for post-analysis
                    //bag.write("ViconData", ros::Time::now(), local);
                    //bag.write("ControlOutput", ros::Time::now(), controllerCall.response.controlOutput);
                }
            }
            else
            {
                ControlCommand zeroOutput = ControlCommand(); //everything set to zero
                zeroOutput.onboardControllerType = TYPE_PPS_MOTORS; //set to motor_mode
                controlCommandPublisher.publish(zeroOutput);

                // Putting data into the ROS "bag" for post-analysis
                //bag.write("ViconData", ros::Time::now(), local);
                //bag.write("ControlOutput", ros::Time::now(), zeroOutput);
            }
        }
	}
}




void loadCrazyflieContext() {
	CMQuery contextCall;
	contextCall.request.studentID = studentID;
	ROS_INFO_STREAM("StudentID:" << studentID);

    CrazyflieContext new_context;

	centralManager.waitForExistence(ros::Duration(-1));

	if(centralManager.call(contextCall)) {
		new_context = contextCall.response.crazyflieContext;
		ROS_INFO_STREAM("CrazyflieContext:\n" << new_context);

        if((context.crazyflieName != "") && (new_context.crazyflieName != context.crazyflieName)) //linked crazyflie name changed and it was not empty before
        {

            // Motors off is done in python script now everytime we disconnect

            // send motors OFF and disconnect before setting context = new_context
            // std_msgs::Int32 msg;
            // msg.data = CMD_CRAZYFLY_MOTORS_OFF;
            // commandPublisher.publish(msg);

            ROS_INFO("CF is now different for this student. Disconnect and turn it off");

            std_msgs::Int32 msg;
            msg.data = CMD_DISCONNECT;
            crazyRadioCommandPublisher.publish(msg);
        }

        context = new_context;

        ros::NodeHandle nh("CrazyRadio");
        nh.setParam("crazyFlieAddress", context.crazyflieAddress);
	}
    else
    {
		ROS_ERROR("Failed to load context. Waiting for next Save in DB by teacher");
	}
}



void commandCallback(const std_msgs::Int32& commandMsg) {
	int cmd = commandMsg.data;

	switch(cmd) {
    	case CMD_USE_SAFE_CONTROLLER:
            ROS_INFO("USE_SAFE_CONTROLLER Command received");
            setControllerUsed(SAFE_CONTROLLER);
    		break;

    	case CMD_USE_CUSTOM_CONTROLLER:
            ROS_INFO("USE_CUSTOM_CONTROLLER Command received");
            setControllerUsed(CUSTOM_CONTROLLER);
    		break;

    	case CMD_CRAZYFLY_TAKE_OFF:
            if(flying_state == STATE_MOTORS_OFF)
            {
                changeFlyingStateTo(STATE_TAKE_OFF);
            }
    		break;

    	case CMD_CRAZYFLY_LAND:
            if(flying_state != STATE_MOTORS_OFF)
            {
                changeFlyingStateTo(STATE_LAND);
            }
    		break;
        case CMD_CRAZYFLY_MOTORS_OFF:
            changeFlyingStateTo(STATE_MOTORS_OFF);
            break;

    	default:
    		ROS_ERROR_STREAM("unexpected command number: " << cmd);
    		break;
	}
}

void DBChangedCallback(const std_msgs::Int32& msg)
{
    loadCrazyflieContext();
}

void emergencyStopCallback(const std_msgs::Int32& msg)
{
    commandCallback(msg);
}

void commandAllAgentsCallback(const std_msgs::Int32& msg)
{
    commandCallback(msg);
}

void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    crazyradio_status = msg.data;
}

void controllerSetPointCallback(const Setpoint& newSetpoint)
{
    // load in variable the setpoint
    controller_setpoint = newSetpoint;

    // if we are in flying, set it up NOW
    if(flying_state == STATE_FLYING)
    {
        goToControllerSetpoint();
    }
}

void safeSetPointCallback(const Setpoint& newSetpoint)
{
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


void yamlReadyForFetchCallback(const std_msgs::Int32& msg)
{
    // Extract from the "msg" for which controller the and from where to fetch the YAML
    // parameters
    int controller_to_fetch_yaml = msg.data;

    // Switch between fetching for the different controllers and from different locations
    switch(controller_to_fetch_yaml)
    {
        case FETCH_YAML_SAFE_CONTROLLER_COORDINATOR:
        {
            // Let the user know that this message was received
            // > and also from where the paramters are being fetched
            ROS_INFO("The PPSClient received the message that YAML parameters were (re-)loaded for the Safe Controller. > Now fetching the parameter values from the coordinator.");
            // Create a node handle to the parameter service running on the coordinator machine
            ros::NodeHandle nodeHandle_to_coordinator_parameter_service = ros::NodeHandle(namespace_to_coordinator_parameter_service);
            // Call the function that fetches the parameters
            fetchYamlParametersForSafeController(nodeHandle_to_coordinator_parameter_service);
            break;
        }

        case FETCH_YAML_SAFE_CONTROLLER_AGENT:
        {
            // Let the user know that this message was received
            ROS_INFO("The PPSClient received the message that YAML parameters were (re-)loaded for the Safe Controller. > Now fetching the parameter values from this machine.");
            // Create a node handle to the parameter service running on this agent's machine
            ros::NodeHandle nodeHandle_to_own_agent_parameter_service(namespace_to_own_agent_parameter_service);
            // Call the function that fetches the parameters
            fetchYamlParametersForSafeController(nodeHandle_to_own_agent_parameter_service);
            break;
        }

        default:
        {
            // Let the user know that the command was not relevant
            //ROS_INFO("The PPSClient received the message that YAML parameters were (re-)loaded.\r> However the parameters do not relate to this service, hence nothing will be fetched.");
            break;
        }
    }
}



void fetchYamlParametersForSafeController(ros::NodeHandle& nodeHandle)
{
    // Here we load the parameters that are specified in the SafeController.yaml file

    // Add the "CustomController" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_safeController(nodeHandle, "SafeController");

    if(!nodeHandle_for_safeController.getParam("takeOffDistance", take_off_distance))
    {
        ROS_ERROR("Failed to get takeOffDistance");
    }

    if(!nodeHandle_for_safeController.getParam("landingDistance", landing_distance))
    {
        ROS_ERROR("Failed to get landing_distance");
    }

    if(!nodeHandle_for_safeController.getParam("durationTakeOff", duration_take_off))
    {
        ROS_ERROR("Failed to get duration_take_off");
    }

    if(!nodeHandle_for_safeController.getParam("durationLanding", duration_landing))
    {
        ROS_ERROR("Failed to get duration_landing");
    }

    if(!nodeHandle_for_safeController.getParam("distanceThreshold", distance_threshold))
    {
        ROS_ERROR("Failed to get distance_threshold");
    }
}


// > Load the paramters from the Client Config YAML file
void fetchClientConfigParameters(ros::NodeHandle& nodeHandle)
{
    if(!nodeHandle.getParam("studentID", studentID)) {
        ROS_ERROR("Failed to get studentID");
    }
    if(!nodeHandle.getParam("strictSafety", strictSafety)) {
        ROS_ERROR("Failed to get strictSafety param");
        return;
    }
    if(!nodeHandle.getParam("angleMargin", angleMargin)) {
        ROS_ERROR("Failed to get angleMargin param");
        return;
    }
    if(!nodeHandle.getParam("battery_threshold_while_flying", m_battery_threshold_while_flying)) {
        ROS_ERROR("Failed to get battery_threshold_while_flying param");
        return;
    }
    if(!nodeHandle.getParam("battery_threshold_while_motors_off", m_battery_threshold_while_motors_off)) {
        ROS_ERROR("Failed to get battery_threshold_while_motors_off param");
        return;
    }
}








void crazyRadioCommandAllAgentsCallback(const std_msgs::Int32& msg)
{
    // The "msg" received can be directly published on the "crazyRadioCommandPublisher"
    // class variable because it is same format message
    // > NOTE: this may be inefficient to "just" pass on the message,
    //   the intention is that it is more transparent that the "coordinator"
    //   node requests all agents to (re/dis)-connect from, and the
    //   individual agents pass this along to their respective radio node.
    crazyRadioCommandPublisher.publish(msg);
}



int getBatteryState()
{
    return m_battery_state;
}


void setBatteryStateTo(int new_battery_state)
{
    switch(new_battery_state)
    {
        case BATTERY_STATE_NORMAL:
            m_battery_state = BATTERY_STATE_NORMAL;
            ROS_INFO("changed battery state to normal");
            break;
        case BATTERY_STATE_LOW:
            m_battery_state = BATTERY_STATE_LOW;
            ROS_INFO("changed battery state to low");
            if(flying_state != STATE_MOTORS_OFF)
                changeFlyingStateTo(STATE_LAND);
            break;
        default:
            ROS_INFO("Unknown battery state command, set to normal");
            m_battery_state = BATTERY_STATE_NORMAL;
            break;
    }

    std_msgs::Int32 battery_state_msg;
    battery_state_msg.data = getBatteryState();
    batteryStatePublisher.publish(battery_state_msg);
}

float movingAverageBatteryFilter(float new_input)
{
    const int N = 7;
    static float previous_output = 0;
    static float inputs[N];


    // imagine an array of an even number of samples, we will output the one in the middle averaged with information from all of them.
    // for that, we only need to store some past of the signal
    float output = previous_output + new_input/N - inputs[N-1]/N;

    // update array of inputs
    for(int i = N - 1; i > 0; i--)
    {
        inputs[i] = inputs[i-1];
    }
    inputs[0] = new_input;


    // update previous output
    previous_output = output;
    return output;
}


void CFBatteryCallback(const std_msgs::Float32& msg)
{
    m_battery_voltage = msg.data;
    // filter and check if inside limits, and if, change status
    // need to do the filtering first
    float filtered_battery_voltage = movingAverageBatteryFilter(m_battery_voltage); //need to perform filtering here

    // ROS_INFO_STREAM("filtered data: " << filtered_battery_voltage);
    if((flying_state != STATE_MOTORS_OFF && (filtered_battery_voltage < m_battery_threshold_while_flying)) ||
       (flying_state == STATE_MOTORS_OFF && (filtered_battery_voltage < m_battery_threshold_while_motors_off)))
    {
        if(getBatteryState() != BATTERY_STATE_LOW)
            setBatteryStateTo(BATTERY_STATE_LOW);
        ROS_INFO("low level battery triggered");
    }
    else                        //maybe add hysteresis somewhere here?
    {
        if(getBatteryState() != BATTERY_STATE_NORMAL)
            setBatteryStateTo(BATTERY_STATE_NORMAL);
    }
}


//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "PPSClient");
	ros::NodeHandle nodeHandle("~");
    ros_namespace = ros::this_node::getNamespace();

    // *********************************************************************************
    // EVERYTHING THAT RELATES TO FETCHING PARAMETERS FROM A YAML FILE

    // > Load the paramters from the Client Config YAML file
    fetchClientConfigParameters(nodeHandle);

    // Get the namespace of this "SafeControllerService" node
    std::string m_namespace = ros::this_node::getNamespace();


    // EVERYTHING FOR THE CONNECTION TO THIS AGENT's OWN PARAMETER SERVICE:

    // Set the class variable "namespace_to_own_agent_parameter_service" to be a the
    // namespace string for the parameter service that is running on the machine of this
    // agent
    namespace_to_own_agent_parameter_service = "ParameterService";
    
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
    //std::string m_ros_namespace = ros::getNamespace();
    namespace_to_coordinator_parameter_service = "ParameterService";

    // Create a node handle to the parameter service running on the coordinator machine
    ros::NodeHandle nodeHandle_to_coordinator = ros::NodeHandle();
    //ros::NodeHandle nodeHandle_to_coordinator_parameter_service = ros::NodeHandle(namespace_to_own_agent_parameter_service);

    // Instantiate the local variable "controllerYamlReadyForFetchSubscriber" to be a
    // "ros::Subscriber" type variable that subscribes to the "controllerYamlReadyForFetch" topic
    // and calls the class function "yamlReadyForFetchCallback" each time a message is
    // received on this topic and the message is passed as an input argument to the
    // "yamlReadyForFetchCallback" class function.
    ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator.subscribe("/ParameterService/controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);
    //ros::Subscriber controllerYamlReadyForFetchSubscriber_to_coordinator = nodeHandle_to_coordinator_parameter_service.subscribe("/ParameterService/controllerYamlReadyForFetch", 1, yamlReadyForFetchCallback);


    // FINALLY, FETCH ANY PARAMETERS REQUIRED FROM THESE "PARAMETER SERVICES"

    // Call the class function that loads the parameters for this class.
    fetchYamlParametersForSafeController(nodeHandle_to_own_agent_parameter_service);

    // *********************************************************************************


    // Load default setpoint from the "SafeController" namespace of the "ParameterService"
    std::vector<float> default_setpoint(4);
    ros::NodeHandle nodeHandle_for_safeController(nodeHandle_to_own_agent_parameter_service, "SafeController");

    if(!nodeHandle_for_safeController.getParam("defaultSetpoint", default_setpoint))
    {
        ROS_ERROR_STREAM("The PPSClient could not find parameter 'defaultSetpoint', as called from main(...)");
    }

    // Copy the default setpoint into the class variable
    controller_setpoint.x = default_setpoint[0];
    controller_setpoint.y = default_setpoint[1];
    controller_setpoint.z = default_setpoint[2];
    controller_setpoint.yaw = default_setpoint[3];
    

	//ros::service::waitForService("/CentralManagerService/CentralManager");
	centralManager = nodeHandle.serviceClient<CMQuery>("/CentralManagerService/Query", false);
	loadCrazyflieContext();

	//keeps 100 messages because otherwise ViconDataPublisher would override the data immediately
	ros::Subscriber viconSubscriber = nodeHandle.subscribe("/ViconDataPublisher/ViconData", 100, viconCallback);
	ROS_INFO_STREAM("successfully subscribed to ViconData");

	//ros::Publishers to advertise the control output
	controlCommandPublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);

	//this topic lets the PPSClient listen to the terminal commands
    commandPublisher = nodeHandle.advertise<std_msgs::Int32>("Command", 1);
    ros::Subscriber commandSubscriber = nodeHandle.subscribe("Command", 1, commandCallback);

    //this topic lets us use the terminal to communicate with crazyRadio node.
    crazyRadioCommandPublisher = nodeHandle.advertise<std_msgs::Int32>("crazyRadioCommand", 1);

    // this topic will publish flying state whenever it changes.
    flyingStatePublisher = nodeHandle.advertise<std_msgs::Int32>("flyingState", 1);

    // will publish battery state when it changes
    batteryStatePublisher = nodeHandle.advertise<std_msgs::Int32>("batteryState", 1);

    controllerUsedPublisher = nodeHandle.advertise<std_msgs::Int32>("controllerUsed", 1);

    // crazy radio status
    crazyradio_status = DISCONNECTED;

    // publish first flying state data
    std_msgs::Int32 flying_state_msg;
    flying_state_msg.data = flying_state;
    flyingStatePublisher.publish(flying_state_msg);

    // SafeControllerServicePublisher:
    ros::NodeHandle namespaceNodeHandle = ros::NodeHandle();
    safeControllerServiceSetpointPublisher = namespaceNodeHandle.advertise<d_fall_pps::Setpoint>("SafeControllerService/Setpoint", 1);
    ros::Subscriber controllerSetpointSubscriber = namespaceNodeHandle.subscribe("student_GUI/ControllerSetpoint", 1, controllerSetPointCallback);
    ros::Subscriber safeSetpointSubscriber = namespaceNodeHandle.subscribe("SafeControllerService/Setpoint", 1, safeSetPointCallback);

    // subscriber for DBChanged
    ros::Subscriber DBChangedSubscriber = namespaceNodeHandle.subscribe("/my_GUI/DBChanged", 1, DBChangedCallback);

    // subscriber for emergencyStop
    ros::Subscriber emergencyStopSubscriber = namespaceNodeHandle.subscribe("/my_GUI/emergencyStop", 1, emergencyStopCallback);

    // Subscriber for "commandAllAgents" commands that are sent from the coordinator node
    ros::Subscriber commandAllAgentsSubscriber = namespaceNodeHandle.subscribe("/my_GUI/commandAllAgents", 1, commandAllAgentsCallback);

    // crazyradio status. Connected, connecting or disconnected
    ros::Subscriber crazyRadioStatusSubscriber = namespaceNodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, crazyRadioStatusCallback);

    // Subscriber for "crazyRadioCommandAllAgents" commands that are sent from the coordinator node
    ros::Subscriber crazyRadioCommandAllAgentsSubscriber = namespaceNodeHandle.subscribe("/my_GUI/crazyRadioCommandAllAgents", 1, crazyRadioCommandAllAgentsCallback);

    // know the battery level of the CF
    ros::Subscriber CFBatterySubscriber = namespaceNodeHandle.subscribe("CrazyRadio/CFBattery", 1, CFBatteryCallback);

	//start with safe controller
    flying_state = STATE_MOTORS_OFF;
    setControllerUsed(SAFE_CONTROLLER);
    setInstantController(SAFE_CONTROLLER); //initialize this also, so we notify GUI

    setBatteryStateTo(BATTERY_STATE_NORMAL); //initialize battery state

    // Open a ROS "bag" to store data for post-analysis
	// std::string package_path;
	// package_path = ros::package::getPath("d_fall_pps") + "/";
	// ROS_INFO_STREAM(package_path);
	// std::string record_file = package_path + "LoggingPPSClient.bag";
	// bag.open(record_file, rosbag::bagmode::Write);

    ros::spin();

    // Close the ROS "bag" that was opened to store data for post-analysis
	//bag.close();

    return 0;
}
