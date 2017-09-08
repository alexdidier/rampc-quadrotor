//    ROS node that manages the student's setup.
//    Copyright (C) 2017  Cyrill Burgener, Marco Mueller, Philipp Friedli
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "ros/ros.h"
#include <stdlib.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <ros/package.h>

#include "d_fall_pps/Controller.h"
#include "d_fall_pps/CMQuery.h"

#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/Setpoint.h"
#include "std_msgs/Int32.h"


#include "d_fall_pps/ControlCommand.h"

// tipes of controllers being used:
#define SAFE_CONTROLLER   0
#define CUSTOM_CONTROLLER 1

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

// commands for CrazyRadio
#define CMD_RECONNECT  0
#define CMD_DISCONNECT 1


// CrazyRadio states:
#define CONNECTED        0
#define CONNECTING       1
#define DISCONNECTED     2

// parameters for take off and landing. Eventually will go in YAML file
#define TAKE_OFF_OFFSET  1    //in meters
#define LANDING_DISTANCE 0.15    //in meters
#define DURATION_TAKE_OFF  3   // seconds
#define DURATION_LANDING   3   // seconds


#define PI 3.141592653589

using namespace d_fall_pps;

//studentID, gives namespace and identifier in CentralManagerService
int studentID;

//the safe controller specified in the ClientConfig.yaml, is considered stable
ros::ServiceClient safeController;
//the custom controller specified in the ClientConfig.yaml, is considered potentially unstable
ros::ServiceClient customController;

//values for safteyCheck
bool strictSafety;
float angleMargin;

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

// publisher to send commands to itself.
ros::Publisher commandPublisher;

// communication with crazyRadio node. Connect and disconnect
ros::Publisher crazyRadioCommandPublisher;

rosbag::Bag bag;

// variables for the states:
int flying_state;
bool changed_state_flag;

// variable for crazyradio status
int crazyradio_status;

//describes the area of the crazyflie and other parameters
CrazyflieContext context;

//wheter to use safe of custom controller
bool using_safe_controller;

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


void useSafeController()
{
    using_safe_controller = true;
    // send a message in topic for the studentGUI to read it
    std_msgs::Int32 controller_used_msg;
    controller_used_msg.data = SAFE_CONTROLLER;
    controllerUsedPublisher.publish(controller_used_msg);
}

void useCustomController()
{
    using_safe_controller = false;
    // send a message in topic for the studentGUI to read it
    std_msgs::Int32 controller_used_msg;
    controller_used_msg.data = CUSTOM_CONTROLLER;
    controllerUsedPublisher.publish(controller_used_msg);
}

bool getUsingSafeController()
{
    return using_safe_controller;
}

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
    useSafeController();
    loadSafeController();
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
    useSafeController();
    loadSafeController();
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
        ROS_INFO("Disconnected and trying to change state. Stays goes to MOTORS OFF");
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
                    if(!getUsingSafeController() && flying_state == STATE_FLYING) // only enter in custom controller if we are not using safe controller AND the flying state is FLYING
                    {
                        bool success = customController.call(controllerCall);
                        if(!success)
                        {
                            ROS_ERROR("Failed to call custom controller, switching to safe controller");
                            ROS_ERROR_STREAM("custom controller status: valid: " << customController.isValid() << ", exists: " << customController.exists());
                            ROS_ERROR_STREAM("custom controller name: " << customController.getService());
                            useSafeController();
                        }
                        else if(!safetyCheck(global, controllerCall.response.controlOutput))
                        {
                            useSafeController();
                            ROS_INFO_STREAM("safety check failed, switching to safe controller");
                        }
                    }
                    else
                    {
                        calculateDistanceToCurrentSafeSetpoint(local); // update distance, it also updates the unit vector
                        ROS_INFO_STREAM("distance: " << distance);
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

                    bag.write("ViconData", ros::Time::now(), local);
                    bag.write("ControlOutput", ros::Time::now(), controllerCall.response.controlOutput);
                }
            }
            else
            {
                ControlCommand zeroOutput = ControlCommand(); //everything set to zero
                zeroOutput.onboardControllerType = 2; //set to motor_mode
                controlCommandPublisher.publish(zeroOutput);
                bag.write("ViconData", ros::Time::now(), local);
                bag.write("ControlOutput", ros::Time::now(), zeroOutput);
            }
        }
	}
}

void loadParameters(ros::NodeHandle& nodeHandle) {
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
}

void loadSafeControllerParameters()
{
    ros::NodeHandle nh_safeControllerService(ros_namespace + "/SafeControllerService");
    if(!nh_safeControllerService.getParam("takeOffDistance", take_off_distance))
    {
		ROS_ERROR("Failed to get takeOffDistance");
	}

    if(!nh_safeControllerService.getParam("landingDistance", landing_distance))
    {
		ROS_ERROR("Failed to get landing_distance");
	}

    if(!nh_safeControllerService.getParam("durationTakeOff", duration_take_off))
    {
		ROS_ERROR("Failed to get duration_take_off");
	}

    if(!nh_safeControllerService.getParam("durationLanding", duration_landing))
    {
		ROS_ERROR("Failed to get duration_landing");
	}

    if(!nh_safeControllerService.getParam("distanceThreshold", distance_threshold))
    {
		ROS_ERROR("Failed to get distance_threshold");
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
    		loadSafeController();
            useSafeController();
    		break;

    	case CMD_USE_CUSTOM_CONTROLLER:
            ROS_INFO("USE_CUSTOM_CONTROLLER Command received");
    		loadCustomController();
            useSafeController();
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


void safeYAMLloadedCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("received msg safe loaded YAML");
    loadSafeControllerParameters();
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "PPSClient");
	ros::NodeHandle nodeHandle("~");
    ros_namespace = ros::this_node::getNamespace();

    // load default setpoint
    std::vector<float> default_setpoint(4);
    ros::NodeHandle nh_safeControllerService(ros_namespace + "/SafeControllerService");

    ROS_INFO_STREAM(ros_namespace << "/SafeControllerService");

    if(!nh_safeControllerService.getParam("defaultSetpoint", default_setpoint))
    {
        ROS_ERROR_STREAM("couldn't find parameter 'defaultSetpoint'");
    }

    controller_setpoint.x = default_setpoint[0];
    controller_setpoint.y = default_setpoint[1];
    controller_setpoint.z = default_setpoint[2];
    controller_setpoint.yaw = default_setpoint[3];

    // load context parameters
	loadParameters(nodeHandle);
    loadSafeControllerParameters();

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

    // crazyradio status. Connected, connecting or disconnected
    ros::Subscriber crazyRadioStatusSubscriber = namespaceNodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, crazyRadioStatusCallback);

    ros::Subscriber safeYAMloadedSubscriber = namespaceNodeHandle.subscribe("student_GUI/safeYAMLloaded", 1, safeYAMLloadedCallback);

	//start with safe controller
    flying_state = STATE_MOTORS_OFF;
    useSafeController();
	loadSafeController();

	std::string package_path;
	package_path = ros::package::getPath("d_fall_pps") + "/";
	ROS_INFO_STREAM(package_path);
	std::string record_file = package_path + "LoggingPPSClient.bag";
	bag.open(record_file, rosbag::bagmode::Write);

    ros::spin();
	bag.close();
    return 0;
}
