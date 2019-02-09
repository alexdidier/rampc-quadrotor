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





// INCLUDE THE HEADER
#include "nodes/FlyingAgentClient.h"





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








//checks if crazyflie is within allowed area and if demo controller returns no data
bool safetyCheck(CrazyflieData data, ControlCommand controlCommand) {
	//position check
	if((data.x < context.localArea.xmin) or (data.x > context.localArea.xmax)) {
		ROS_INFO_STREAM("[FLYING AGENT CLIENT] x safety failed");
		return false;
	}
	if((data.y < context.localArea.ymin) or (data.y > context.localArea.ymax)) {
		ROS_INFO_STREAM("[FLYING AGENT CLIENT] y safety failed");
		return false;
	}
	if((data.z < context.localArea.zmin) or (data.z > context.localArea.zmax)) {
		ROS_INFO_STREAM("[FLYING AGENT CLIENT] z safety failed");
		return false;
	}

	//attitude check
	//if strictSafety is set to true in ClientConfig.yaml the SafeController takes also over if the roll and pitch angles get to large
	//the angleMargin is a value in the range (0,1). The closer to 1, the closer to 90 deg are the roll and pitch angles allowed to become before the safeController takes over
	if(yaml_strictSafety){
		if((data.roll > PI/2*yaml_angleMargin) or (data.roll < -PI/2*yaml_angleMargin)) {
			ROS_INFO_STREAM("[FLYING AGENT CLIENT] roll too big.");
			return false;
		}
		if((data.pitch > PI/2*yaml_angleMargin) or (data.pitch < -PI/2*yaml_angleMargin)) {
			ROS_INFO_STREAM("[FLYING AGENT CLIENT] pitch too big.");
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
    setpoint_msg.z = current_local_coordinates.z + yaml_take_off_distance;           //previous one plus some offset
    // setpoint_msg.yaw = current_local_coordinates.yaw;          //previous one
    setpoint_msg.yaw = 0.0;
    safeControllerServiceSetpointPublisher.publish(setpoint_msg);
    ROS_INFO("[FLYING AGENT CLIENT] Take OFF:");
    ROS_INFO("[FLYING AGENT CLIENT] ------Current coordinates:");
    ROS_INFO("[FLYING AGENT CLIENT] X: %f, Y: %f, Z: %f", current_local_coordinates.x, current_local_coordinates.y, current_local_coordinates.z);
    ROS_INFO("[FLYING AGENT CLIENT] ------New coordinates:");
    ROS_INFO("[FLYING AGENT CLIENT] X: %f, Y: %f, Z: %f", setpoint_msg.x, setpoint_msg.y, setpoint_msg.z);

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
    setpoint_msg.z = yaml_landing_distance;           //previous one plus some offset
    setpoint_msg.yaw = current_local_coordinates.yaw;          //previous one
    safeControllerServiceSetpointPublisher.publish(setpoint_msg);

    // now, use safe controller to go to that setpoint
    loadSafeController();
    setInstantController(SAFE_CONTROLLER);
    setCurrentSafeSetpoint(setpoint_msg);
}

void changeFlyingStateTo(int new_state)
{
    if(crazyradio_status == CRAZY_RADIO_STATE_CONNECTED)
    {
        ROS_INFO("[FLYING AGENT CLIENT] Change state to: %d", new_state);
        flying_state = new_state;
    }
    else
    {
        ROS_INFO("[FLYING AGENT CLIENT] Disconnected and trying to change state. State goes to MOTORS OFF");
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
void viconCallback(const ViconData& viconData)
{
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
                        ROS_INFO("[FLYING AGENT CLIENT] STATE_MOTORS_OFF");
                    }
                    break;
                case STATE_TAKE_OFF: //we need to move up from where we are now.
                    if(changed_state_flag) // stuff that will be run only once when changing state
                    {
                        changed_state_flag = false;
                        takeOffCF(local);
                        finished_take_off = false;
                        ROS_INFO("[FLYING AGENT CLIENT] STATE_TAKE_OFF");
                        timer_takeoff = nodeHandle.createTimer(ros::Duration(yaml_duration_take_off), takeOffTimerCallback, true);
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
                        ROS_INFO("[FLYING AGENT CLIENT] STATE_FLYING");
                    }
                    break;
                case STATE_LAND:
                    if(changed_state_flag) // stuff that will be run only once when changing state
                    {
                        changed_state_flag = false;
                        landCF(local);
                        finished_land = false;
                        ROS_INFO("[FLYING AGENT CLIENT] STATE_LAND");
                        timer_takeoff = nodeHandle.createTimer(ros::Duration(yaml_duration_landing), landTimerCallback, true);
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
                    // Only call an "non-safe" controller if:
                    // 1) we are not currently using safe controller, AND
                    // 2) the flying state is FLYING
                    if( (getInstantController() != SAFE_CONTROLLER) && (flying_state == STATE_FLYING) )
                    {
                        // Initialise a local boolean success variable
                        bool success = false;

                        switch(getInstantController())
                        {
                            case DEMO_CONTROLLER:
                                success = demoController.call(controllerCall);
                                break;
                            case STUDENT_CONTROLLER:
                                success = studentController.call(controllerCall);
                                break;
                            case MPC_CONTROLLER:
                                success = mpcController.call(controllerCall);
                                break;
                            case REMOTE_CONTROLLER:
                                success = remoteController.call(controllerCall);
                                break;
                            case TUNING_CONTROLLER:
                                success = tuningController.call(controllerCall);
                                break;
                            case PICKER_CONTROLLER:
                                success = pickerController.call(controllerCall);
                                break;
                            default:
                                ROS_ERROR("[FLYING AGENT CLIENT] the current controller was not recognised");
                                break;
                        }

                        // Ensure success and enforce safety
                        if(!success)
                        {
                            ROS_ERROR("[FLYING AGENT CLIENT] Failed to call a 'non-safe' controller, switching to safe controller");
                            //ROS_ERROR_STREAM("[FLYING AGENT CLIENT] 'non-safe' controller status: valid: " << demoController.isValid() << ", exists: " << demoController.exists());
                            //ROS_ERROR_STREAM("[FLYING AGENT CLIENT] 'non-safe' controller name: " << demoController.getService());
                            setInstantController(SAFE_CONTROLLER);
                        }
                        else if(!safetyCheck(global, controllerCall.response.controlOutput))
                        {
                            setInstantController(SAFE_CONTROLLER);
                            ROS_INFO_STREAM("[FLYING AGENT CLIENT] safety check failed, switching to safe controller");
                        }

                        
                    }
                    // SAFE_CONTROLLER and state is different from flying
                    else
                    {
                        calculateDistanceToCurrentSafeSetpoint(local); // update distance, it also updates the unit vector
                        // ROS_INFO_STREAM("distance: " << distance);
                        // here, detect if euclidean distance between setpoint and current position is higher than a threshold
                        if(distance > yaml_distance_threshold)
                        {
                            // DEBUGGING: display a message that the crazyflie is inside the thresholds
                            //ROS_INFO("inside threshold");
                            // Declare a local variable for the "setpoint message" to be published
                            Setpoint setpoint_msg;
                            // here, where we are now, or where we were in the beginning?
                            setpoint_msg.x = local.x + yaml_distance_threshold * unit_vector[0];
                            setpoint_msg.y = local.y + yaml_distance_threshold * unit_vector[1];
                            setpoint_msg.z = local.z + yaml_distance_threshold * unit_vector[2];

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
                            ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Failed to call safe controller, valid: " << safeController.isValid() << ", exists: " << safeController.exists());
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
                zeroOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS; //set to motor_mode
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
	contextCall.request.studentID = m_agentID;
	ROS_INFO_STREAM("[FLYING AGENT CLIENT] AgentID:" << m_agentID);

    CrazyflieContext new_context;

	centralManager.waitForExistence(ros::Duration(-1));

	if(centralManager.call(contextCall)) {
		new_context = contextCall.response.crazyflieContext;
		ROS_INFO_STREAM("[FLYING AGENT CLIENT] CrazyflieContext:\n" << new_context);

        if((context.crazyflieName != "") && (new_context.crazyflieName != context.crazyflieName)) //linked crazyflie name changed and it was not empty before
        {

            // Motors off is done in python script now everytime we disconnect

            // send motors OFF and disconnect before setting context = new_context
            // std_msgs::Int32 msg;
            // msg.data = CMD_CRAZYFLY_MOTORS_OFF;
            // commandPublisher.publish(msg);

            ROS_INFO("[FLYING AGENT CLIENT] CF is now different for this student. Disconnect and turn it off");

            IntWithHeader msg;
            msg.shouldCheckForID = false;
            msg.data = CMD_DISCONNECT;
            crazyRadioCommandPublisher.publish(msg);
        }

        context = new_context;

        ros::NodeHandle nh("CrazyRadio");
        nh.setParam("crazyFlieAddress", context.crazyflieAddress);
	}
    else
    {
		ROS_ERROR("[FLYING AGENT CLIENT] Failed to load context. Waiting for next Save in DB by teacher");
	}
}



void commandCallback(const IntWithHeader & msg) {

    // Check whether the message is relevant
    bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForID , msg.agentIDs );

    // Continue if the message is relevant
    if (isRevelant)
    {
        // Extract the data
    	int cmd = msg.data;

    	switch(cmd) {
        	case CMD_USE_SAFE_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_SAFE_CONTROLLER Command received");
                setControllerUsed(SAFE_CONTROLLER);
        		break;

        	case CMD_USE_DEMO_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_DEMO_CONTROLLER Command received");
                setControllerUsed(DEMO_CONTROLLER);
        		break;

            case CMD_USE_STUDENT_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_STUDENT_CONTROLLER Command received");
                setControllerUsed(STUDENT_CONTROLLER);
                break;

            case CMD_USE_MPC_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_MPC_CONTROLLER Command received");
                setControllerUsed(MPC_CONTROLLER);
                break;

            case CMD_USE_REMOTE_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_REMOTE_CONTROLLER Command received");
                setControllerUsed(REMOTE_CONTROLLER);
                break;

            case CMD_USE_TUNING_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_TUNING_CONTROLLER Command received");
                setControllerUsed(TUNING_CONTROLLER);
                break;

            case CMD_USE_PICKER_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_PICKER_CONTROLLER Command received");
                setControllerUsed(PICKER_CONTROLLER);
                break;

        	case CMD_CRAZYFLY_TAKE_OFF:
                ROS_INFO("[FLYING AGENT CLIENT] TAKE_OFF Command received");
                if(flying_state == STATE_MOTORS_OFF)
                {
                    changeFlyingStateTo(STATE_TAKE_OFF);
                }
        		break;

        	case CMD_CRAZYFLY_LAND:
                ROS_INFO("[FLYING AGENT CLIENT] LAND Command received");
                if(flying_state != STATE_MOTORS_OFF)
                {
                    changeFlyingStateTo(STATE_LAND);
                }
        		break;
            case CMD_CRAZYFLY_MOTORS_OFF:
                ROS_INFO("[FLYING AGENT CLIENT] MOTORS_OFF Command received");
                changeFlyingStateTo(STATE_MOTORS_OFF);
                break;

        	default:
        		ROS_ERROR_STREAM("[FLYING AGENT CLIENT] unexpected command number: " << cmd);
        		break;
    	}
    }
}

void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("[FLYING AGENT CLIENT] Received message that the Context Database Changed");
    loadCrazyflieContext();
}

void emergencyStopCallback(const IntWithHeader & msg)
{
    ROS_INFO("[FLYING AGENT CLIENT] Received message to EMERGENCY STOP");
    commandCallback(msg);
}

//void commandAllAgentsCallback(const std_msgs::Int32& msg)
//{
//    commandCallback(msg);
//}

void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Received message with Crazy Radio Status = " << msg.data );
    crazyradio_status = msg.data;
    // RESET THE BATTERY STATE IF DISCONNECTED
    //if (crazyradio_status == CRAZY_RADIO_STATE_DISCONNECTED)
    //{
    //    setBatteryStateTo(BATTERY_STATE_NORMAL);
    //}
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
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//    ----------------------------------------------------------------------------------





void loadSafeController() {
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string safeControllerName;
    if(!nodeHandle.getParam("safeController", safeControllerName)) {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get safe controller name");
        return;
    }

    ros::service::waitForService(safeControllerName);
    safeController = ros::service::createClient<Controller>(safeControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] loaded safe controller: " << safeController.getService());
}

void loadDemoController()
{
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string demoControllerName;
    if(!nodeHandle.getParam("demoController", demoControllerName))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get demo controller name");
        return;
    }

    demoController = ros::service::createClient<Controller>(demoControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded demo controller " << demoController.getService());
}

void loadStudentController()
{
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string studentControllerName;
    if(!nodeHandle.getParam("studentController", studentControllerName))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get student controller name");
        return;
    }

    studentController = ros::service::createClient<Controller>(studentControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded student controller " << studentController.getService());
}

void loadMpcController()
{
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string mpcControllerName;
    if(!nodeHandle.getParam("mpcController", mpcControllerName))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get mpc controller name");
        return;
    }

    mpcController = ros::service::createClient<Controller>(mpcControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded mpc controller " << mpcController.getService());
}

void loadRemoteController()
{
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string remoteControllerName;
    if(!nodeHandle.getParam("remoteController", remoteControllerName))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get remote controller name");
        return;
    }

    remoteController = ros::service::createClient<Controller>(remoteControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded remote controller " << remoteController.getService());
}

void loadTuningController()
{
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string tuningControllerName;
    if(!nodeHandle.getParam("tuningController", tuningControllerName))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get tuning controller name");
        return;
    }

    tuningController = ros::service::createClient<Controller>(tuningControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded tuning controller " << tuningController.getService());
}

void loadPickerController()
{
    //ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string pickerControllerName;
    if(!nodeHandle.getParam("pickerController", pickerControllerName))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get picker controller name");
        return;
    }

    pickerController = ros::service::createClient<Controller>(pickerControllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded picker controller " << pickerController.getService());
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
        case DEMO_CONTROLLER:
            loadDemoController();
            break;
        case STUDENT_CONTROLLER:
            loadStudentController();
            break;
        case MPC_CONTROLLER:
            loadMpcController();
            break;
        case REMOTE_CONTROLLER:
            loadRemoteController();
            break;
        case TUNING_CONTROLLER:
            loadTuningController();
            break;
        case PICKER_CONTROLLER:
            loadPickerController();
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






//    ----------------------------------------------------------------------------------
//    BBBB     A    TTTTT  TTTTT  EEEEE  RRRR   Y   Y
//    B   B   A A     T      T    E      R   R   Y Y
//    BBBB   A   A    T      T    EEE    RRRR     Y
//    B   B  AAAAA    T      T    E      R   R    Y
//    BBBB   A   A    T      T    EEEEE  R   R    Y
//    ----------------------------------------------------------------------------------



void batteryMonitorStateChangedCallback(std_msgs::Int32 msg)
{
    // Extract the data
    int new_battery_state = msg.data;

    // Take action if changed to low battery state
    if (new_battery_state == BATTERY_STATE_LOW)
    {
        if (flying_state != STATE_MOTORS_OFF)
        {
            ROS_INFO("[FLYING AGENT CLIENT] low level battery triggered, now landing.");
            changeFlyingStateTo(STATE_LAND);
        }
        else
        {
            ROS_INFO("[FLYING AGENT CLIENT] low level battery triggered, please turn off the Crazyflie.");
        }
    }
    else if (new_battery_state == BATTERY_STATE_NORMAL)
    {
        ROS_INFO("[FLYING AGENT CLIENT] received message that battery state changed to normal");
    }
    else
    {
        ROS_INFO("[FLYING AGENT CLIENT] received message that battery state changed to something unknown");
    }
    
}



/*
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
            ROS_INFO("[FLYING AGENT CLIENT] changed battery state to normal");
            break;
        case BATTERY_STATE_LOW:
            m_battery_state = BATTERY_STATE_LOW;
            ROS_INFO("[FLYING AGENT CLIENT] changed battery state to low");
            if(flying_state != STATE_MOTORS_OFF)
                changeFlyingStateTo(STATE_LAND);
            break;
        default:
            ROS_INFO("[FLYING AGENT CLIENT] Unknown battery state command, set to normal");
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
    static float previous_output = 4.2f;
    static float inputs[N] = {4.2f,4.2f,4.2f,4.2f,4.2f,4.2f,4.2f};


    // imagine an array of an even number of samples, we will output the one
    // in the middle averaged with information from all of them.
    // For that, we only need to store some past of the signal
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
    if(
        (flying_state != STATE_MOTORS_OFF && (filtered_battery_voltage < m_battery_threshold_while_flying))
        ||
        (flying_state == STATE_MOTORS_OFF && (filtered_battery_voltage < m_battery_threshold_while_motors_off))
    )
    {
        if(getBatteryState() != BATTERY_STATE_LOW)
        {
            setBatteryStateTo(BATTERY_STATE_LOW);
            ROS_INFO("[FLYING AGENT CLIENT] low level battery triggered");
        }
        
    }
    else
    {
        // TO AVOID BEING ABLE TO IMMEDIATELY TAKE-OFF AFTER A
        // "BATTERY LOW" EVENT IS TRIGGERED, WE DO NOT SET THE
        // BATTERY STATE BACK TO NORMAL
        // if(getBatteryState() != BATTERY_STATE_NORMAL)
        // {
        //     setBatteryStateTo(BATTERY_STATE_NORMAL);
        // }
    }
}
*/









//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT      SSSS  TTTTT    A    TTTTT  U   U   SSSS
//    G      E        T       S        T     A A     T    U   U  S
//    G      EEE      T        SSS     T    A   A    T    U   U   SSS
//    G   G  E        T           S    T    AAAAA    T    U   U      S
//     GGGG  EEEEE    T       SSSS     T    A   A    T     UUU   SSSS
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K   SSSS
//    C       A A   L      L      B   B   A A   C      K  K   S
//    C      A   A  L      L      BBBB   A   A  C      KKK     SSS
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K       S
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K  SSSS
//    ----------------------------------------------------------------------------------

bool getCurrentFlyingStateServiceCallback(IntIntService::Request &request, IntIntService::Response &response)
{
    // Put the flying state into the response variable
    response.data = flying_state;
    // Return
    return true;
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



void isReadySafeControllerYamlCallback(const IntWithHeader & msg)
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
                ROS_INFO("[FLYING AGENT CLIENT] Now fetching the SafeController YAML parameter values from this agent.");
                namespace_to_use = m_namespace_to_own_agent_parameter_service;
                break;
            }
            // > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
            case LOAD_YAML_FROM_COORDINATOR:
            {
                ROS_INFO("[FLYING AGENT CLIENT] Now fetching the SafeController YAML parameter values from this agent's coordinator.");
                namespace_to_use = m_namespace_to_coordinator_parameter_service;
                break;
            }

            default:
            {
                ROS_ERROR("[FLYING AGENT CLIENT] Paramter service to load from was NOT recognised.");
                namespace_to_use = m_namespace_to_own_agent_parameter_service;
                break;
            }
        }
        // Create a node handle to the selected parameter service
        ros::NodeHandle nodeHandle_to_use(namespace_to_use);
        // Call the function that fetches the parameters
        fetchSafeControllerYamlParameters(nodeHandle_to_use);
    }
}



void fetchSafeControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
    // Here we load the parameters that are specified in the file:
    // SafeController.yaml

    // Add the "SafeController" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "SafeController");

    // take off and landing parameters (in meters and seconds)
    yaml_take_off_distance = getParameterFloat(nodeHandle_for_paramaters,"takeOffDistance");
    yaml_landing_distance = getParameterFloat(nodeHandle_for_paramaters,"landingDistance");
    yaml_duration_take_off = getParameterFloat(nodeHandle_for_paramaters,"durationTakeOff");
    yaml_duration_landing = getParameterFloat(nodeHandle_for_paramaters,"durationLanding");
    yaml_distance_threshold = getParameterFloat(nodeHandle_for_paramaters,"distanceThreshold");

    // setpoint in meters (x, y, z, yaw)
    getParameterFloatVector(nodeHandle_for_paramaters,"defaultSetpoint",yaml_default_setpoint,4);

    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] DEBUGGING: the fetched SafeController/durationTakeOff = " << yaml_duration_take_off);

    /*
    // Here we load the parameters that are specified in the SafeController.yaml file

    // Add the "SafeController" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_safeController(nodeHandle, "SafeController");

    if(!nodeHandle_for_safeController.getParam("takeOffDistance", take_off_distance))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get takeOffDistance");
    }

    if(!nodeHandle_for_safeController.getParam("landingDistance", landing_distance))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get landing_distance");
    }

    if(!nodeHandle_for_safeController.getParam("durationTakeOff", duration_take_off))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get duration_take_off");
    }

    if(!nodeHandle_for_safeController.getParam("durationLanding", duration_landing))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get duration_landing");
    }

    if(!nodeHandle_for_safeController.getParam("distanceThreshold", distance_threshold))
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get distance_threshold");
    }
    */
}




void isReadyClientConfigYamlCallback(const IntWithHeader & msg)
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
                ROS_INFO("[FLYING AGENT CLIENT] Now fetching the ClientConfig YAML parameter values from this agent.");
                namespace_to_use = m_namespace_to_own_agent_parameter_service;
                break;
            }
            // > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
            case LOAD_YAML_FROM_COORDINATOR:
            {
                ROS_INFO("[FLYING AGENT CLIENT] Now fetching the ClientConfig YAML parameter values from this agent's coordinator.");
                namespace_to_use = m_namespace_to_coordinator_parameter_service;
                break;
            }

            default:
            {
                ROS_ERROR("[FLYING AGENT CLIENT] Paramter service to load from was NOT recognised.");
                namespace_to_use = m_namespace_to_own_agent_parameter_service;
                break;
            }
        }
        // Create a node handle to the selected parameter service
        ros::NodeHandle nodeHandle_to_use(namespace_to_use);
        // Call the function that fetches the parameters
        fetchClientConfigYamlParameters(nodeHandle_to_use);
    }
}



// > Load the paramters from the Client Config YAML file
void fetchClientConfigYamlParameters(ros::NodeHandle& nodeHandle)
{
    // Here we load the parameters that are specified in the file:
    // ClientConfig.yaml

    // Add the "ClientConfig" namespace to the "nodeHandle"
    ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "ClientConfig");

    // Flag for whether to use angle for switching to the Safe Controller
    yaml_strictSafety = getParameterBool(nodeHandle_for_paramaters,"strictSafety");
    yaml_angleMargin = getParameterFloat(nodeHandle_for_paramaters,"angleMargin");
    


    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] DEBUGGING: the fetched ClientConfig/strictSafety = " << yaml_strictSafety);


    /*
    if(!nodeHandle.getParam("strictSafety", strictSafety)) {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get strictSafety param");
        return;
    }
    if(!nodeHandle.getParam("angleMargin", angleMargin)) {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get angleMargin param");
        return;
    }


    if(!nodeHandle.getParam("battery_threshold_while_flying", m_battery_threshold_while_flying)) {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get battery_threshold_while_flying param");
        return;
    }
    if(!nodeHandle.getParam("battery_threshold_while_motors_off", m_battery_threshold_while_motors_off)) {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to get battery_threshold_while_motors_off param");
        return;
    }
    */
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
    // Starting the ROS-node
	ros::init(argc, argv, "FlyingAgentClient");

    // Create a "ros::NodeHandle" type local variable named "nodeHandle",
    // the "~" indcates that "self" is the node handle assigned.
	ros::NodeHandle nodeHandle("~");

    // Get the namespace of this node
    std::string m_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] ros::this_node::getNamespace() =  " << m_namespace);



    // AGENT ID AND COORDINATOR ID

    // Get the ID of the agent and its coordinator
    bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

    // Stall the node IDs are not valid
    if ( !isValid_IDs )
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Node NOT FUNCTIONING :-)");
        ros::spin();
    }
    else
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
    }



    // PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

    // Set the class variable "m_namespace_to_own_agent_parameter_service",
    // i.e., the namespace of parameter service for this agent
    m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

    // Set the class variable "m_namespace_to_coordinator_parameter_service",
    // i.e., the namespace of parameter service for this agent's coordinator
    constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

    // Inform the user of what namespaces are being used
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

    // Create, as local variables, node handles to the parameters services
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



    // SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

    // The parameter service publishes messages with names of the form:
    // /dfall/.../ParameterService/<filename with .yaml extension>
    ros::Subscriber clientConfig_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "ClientConfig", 1, isReadyClientConfigYamlCallback);
    ros::Subscriber clientConfig_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("ClientConfig", 1, isReadyClientConfigYamlCallback);

    ros::Subscriber safeController_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "SafeController", 1, isReadySafeControllerYamlCallback);
    ros::Subscriber safeController_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("SafeController", 1, isReadySafeControllerYamlCallback);



    // GIVE YAML VARIABLES AN INITIAL VALUE

	// This can be done either here or as part of declaring the
	// variables in the header file



	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

    // Call the class function that loads the parameters for this class.
    fetchClientConfigYamlParameters(nodeHandle_to_own_agent_parameter_service);
    fetchSafeControllerYamlParameters(nodeHandle_to_own_agent_parameter_service);


    // THIS IS THE "NEW" WAY TO DO IT
    // BUT NEED TO CHECK ALL VARIABLES HAVE INITIAL VALUE
 //    // FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// // The yaml files for the controllers are not added to
	// // "Parameter Service" as part of launching.
	// // The process for loading the yaml parameters is to send a
	// // service call containing the filename of the *.yaml file,
	// // and then a message will be received on the above subscribers
	// // when the paramters are ready.
	// // > NOTE IMPORTANTLY that by using a serice client
	// //   we stall the availability of this node until the
	// //   paramter service is ready

	// // Create the service client as a local variable
	// ros::ServiceClient requestLoadYamlFilenameServiceClient = nodeHandle_to_own_agent_parameter_service.serviceClient<LoadYamlFromFilename>("requestLoadYamlFilename", false);
	// // Create the service call as a local variable
	// LoadYamlFromFilename loadYamlFromFilenameCall;
	// // Specify the Yaml filename as a string
	// loadYamlFromFilenameCall.request.stringWithHeader.data = "DefaultController";
	// // Set for whom this applies to
	// loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForID = false;
	// // Wait until the serivce exists
	// requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// // Make the service call
	// if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	// {
	// 	// Nothing to do in this case.
	// 	// The "isReadyDefaultControllerYamlCallback" function
	// 	// will be called once the YAML file is loaded
	// }
	// else
	// {
	// // Inform the user
	// 	ROS_ERROR("[DEFAULT CONTROLLER] The request load yaml file service call failed.");
	// }



    // Copy the default setpoint into the class variable
    controller_setpoint.x   = yaml_default_setpoint[0];
    controller_setpoint.y   = yaml_default_setpoint[1];
    controller_setpoint.z   = yaml_default_setpoint[2];
    controller_setpoint.yaw = yaml_default_setpoint[3];




    // PUBLISHERS, SUBSCRIBERS, AND SERVICE CLIENTS

	

    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle nodeHandle_dfall_root("/dfall");

    // CREATE A NODE HANDLE TO THE COORDINATOR
    std::string namespace_to_coordinator;
    constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
    ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);




    // SERVICE CLIENT FOR LOADING THE ALLOCATED FLYING ZONE
    // AND OTHER CONTEXT DETAILS

    //ros::service::waitForService("/CentralManagerService/CentralManager");
	centralManager = nodeHandle_dfall_root.serviceClient<CMQuery>("CentralManagerService/Query", false);
	loadCrazyflieContext();

    // Subscriber for when the Flying Zone Database changed
    ros::Subscriber databaseChangedSubscriber = nodeHandle_dfall_root.subscribe("CentralManagerService/DBChanged", 1, crazyflieContextDatabaseChangedCallback);

    // EMERGENCY STOP OF THE WHOLE "D-FaLL-System"
    ros::Subscriber emergencyStopSubscriber = nodeHandle_dfall_root.subscribe("emergencyStop", 1, emergencyStopCallback);

    // LOCALISATION DATA FROM MOTION CAPTURE SYSTEM
	//keeps 100 messages because otherwise ViconDataPublisher would override the data immediately
	ros::Subscriber viconSubscriber = nodeHandle_dfall_root.subscribe("ViconDataPublisher/ViconData", 100, viconCallback);



    // PUBLISHER FOR COMMANDING THE CRAZYFLIE
    // i.e., motorCmd{1,2,3,4}, roll, pitch, yaw, and onboardControllerType
	//ros::Publishers to advertise the control output
	controlCommandPublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);

	//this topic lets the FlyingAgentClient listen to the terminal commands
    //commandPublisher = nodeHandle.advertise<std_msgs::Int32>("Command", 1);



    // SUBSCRIBER FOR THE CHANGE STATE COMMANDS
    // i.e., {TAKE-OFF,LAND,MOTORS-OFF,CONTROLLER SELECTION}
    // > for the agent GUI
    ros::Subscriber commandSubscriber_to_agent = nodeHandle.subscribe("Command", 1, commandCallback);
    // > for the coord GUI
    ros::Subscriber commandSubscriber_to_coord = nodeHandle_to_coordinator.subscribe("FlyingAgentClient/Command", 1, commandCallback);




    // PUBLISHER FOR THE CRAZYRADIO COMMANDS
    // i.e., {CONNECT,DISCONNECT}
    // This topic lets us use the terminal to communicate with
    // the crazyRadio node even when the GUI is not launched
    crazyRadioCommandPublisher = nodeHandle.advertise<IntWithHeader>("crazyRadioCommand", 1);



    // PUBLISHER FOR THE FLYING STATE
    // Possible states: {MOTORS-OFF,TAKE-OFF,FLYING,LAND}
    // This topic will publish flying state whenever it changes.
    flyingStatePublisher = nodeHandle.advertise<std_msgs::Int32>("flyingState", 1);

    

    // PUBLISHER FOR THE 
    controllerUsedPublisher = nodeHandle.advertise<std_msgs::Int32>("controllerUsed", 1);




    // crazy radio status
    crazyradio_status = CRAZY_RADIO_STATE_DISCONNECTED;

    // publish first flying state data
    std_msgs::Int32 flying_state_msg;
    flying_state_msg.data = flying_state;
    flyingStatePublisher.publish(flying_state_msg);

    // SafeControllerServicePublisher:
    ros::NodeHandle namespaceNodeHandle = ros::NodeHandle();
    safeControllerServiceSetpointPublisher = namespaceNodeHandle.advertise<dfall_pkg::Setpoint>("SafeControllerService/Setpoint", 1);
    ros::Subscriber controllerSetpointSubscriber = namespaceNodeHandle.subscribe("student_GUI/ControllerSetpoint", 1, controllerSetPointCallback);
    ros::Subscriber safeSetpointSubscriber = namespaceNodeHandle.subscribe("SafeControllerService/Setpoint", 1, safeSetPointCallback);

    

    
    // Subscriber for "commandAllAgents" commands that are sent from the coordinator node
    //ros::Subscriber commandAllAgentsSubscriber = namespaceNodeHandle.subscribe("/my_GUI/commandAllAgents", 1, commandAllAgentsCallback);

    // crazyradio status. Connected, connecting or disconnected
    ros::Subscriber crazyRadioStatusSubscriber = namespaceNodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, crazyRadioStatusCallback);





    // will publish battery state when it changes
    //batteryStatePublisher = nodeHandle.advertise<std_msgs::Int32>("batteryState", 1);

    // INITIALISE THE BATTERY VOLTAGE TO SOMETHING CLOSE TO FULL
    // > This is used to prevent the "Low Battery" being trigged when the 
    //   first battery voltage data is received
    //m_battery_voltage = 4.2f;
    // know the battery level of the CF
    //ros::Subscriber CFBatterySubscriber = namespaceNodeHandle.subscribe("CrazyRadio/CFBattery", 1, CFBatteryCallback);

    //setBatteryStateTo(BATTERY_STATE_NORMAL); //initialize battery state

    // Subscribe to the battery state change message from the Battery Monitor node
    std::string namespace_to_battery_monitor = m_namespace + "/BatteryMonitor";
    ros::NodeHandle nodeHandle_to_battery_monitor(namespace_to_battery_monitor);
    ros::Subscriber CFBatterySubscriber = nodeHandle_to_battery_monitor.subscribe("ChangedStateTo", 1, batteryMonitorStateChangedCallback);


	//start with safe controller
    flying_state = STATE_MOTORS_OFF;
    setControllerUsed(SAFE_CONTROLLER);
    setInstantController(SAFE_CONTROLLER); //initialize this also, so we notify GUI


    // Advertise the service for the current flying state
    ros::ServiceServer getCurrentFlyingStateService = nodeHandle.advertiseService("getCurrentFlyingState", getCurrentFlyingStateServiceCallback);

    

    // Open a ROS "bag" to store data for post-analysis
	// std::string package_path;
	// package_path = ros::package::getPath("dfall_pkg") + "/";
	// ROS_INFO_STREAM(package_path);
	// std::string record_file = package_path + "LoggingFlyingAgentClient.bag";
	// bag.open(record_file, rosbag::bagmode::Write);

    ros::spin();

    // Close the ROS "bag" that was opened to store data for post-analysis
	//bag.close();

    return 0;
}
