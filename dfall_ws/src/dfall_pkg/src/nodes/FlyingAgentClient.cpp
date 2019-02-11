//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat, Angel Romero, Cyrill Burgener, Marco Mueller, Philipp Friedli
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















// void takeOffCF(CrazyflieData& current_local_coordinates) //local because the setpoint is in local coordinates
// {
//     // set the setpoint and call safe controller
//     Setpoint setpoint_msg;
//     setpoint_msg.x = current_local_coordinates.x;           // previous one
//     setpoint_msg.y = current_local_coordinates.y;           //previous one
//     setpoint_msg.z = current_local_coordinates.z + yaml_take_off_distance;           //previous one plus some offset
//     // setpoint_msg.yaw = current_local_coordinates.yaw;          //previous one
//     setpoint_msg.yaw = 0.0;
//     safeControllerServiceSetpointPublisher.publish(setpoint_msg);
//     ROS_INFO("[FLYING AGENT CLIENT] Take OFF:");
//     ROS_INFO("[FLYING AGENT CLIENT] ------Current coordinates:");
//     ROS_INFO("[FLYING AGENT CLIENT] X: %f, Y: %f, Z: %f", current_local_coordinates.x, current_local_coordinates.y, current_local_coordinates.z);
//     ROS_INFO("[FLYING AGENT CLIENT] ------New coordinates:");
//     ROS_INFO("[FLYING AGENT CLIENT] X: %f, Y: %f, Z: %f", setpoint_msg.x, setpoint_msg.y, setpoint_msg.z);

//     // now, use safe controller to go to that setpoint
//     loadSafeController();
//     setInstantController(SAFE_CONTROLLER);
//     // when do we finish? after some time with delay?

//     // update variable that keeps track of current setpoint
//     setCurrentSafeSetpoint(setpoint_msg);
// }

// void landCF(CrazyflieData& current_local_coordinates)
// {
//     // set the setpoint and call safe controller
//     Setpoint setpoint_msg;
//     setpoint_msg.x = current_local_coordinates.x;           // previous one
//     setpoint_msg.y = current_local_coordinates.y;           //previous one
//     setpoint_msg.z = yaml_landing_distance;           //previous one plus some offset
//     setpoint_msg.yaw = current_local_coordinates.yaw;          //previous one
//     safeControllerServiceSetpointPublisher.publish(setpoint_msg);

//     // now, use safe controller to go to that setpoint
//     loadSafeController();
//     setInstantController(SAFE_CONTROLLER);
//     setCurrentSafeSetpoint(setpoint_msg);
// }




// void goToControllerSetpoint()
// {
//     safeControllerServiceSetpointPublisher.publish(controller_setpoint);
//     setCurrentSafeSetpoint(controller_setpoint);
// }


//is called when new data from Vicon arrives
void viconCallback(const ViconData& viconData)
{
    // NOTE: THIS FUNCTION IS CALL AT THE FREQUENCY OF THE MOTION
    //       CAPTURE SYSTEM. HENCE ANY OPTERATIONS PERFORMED IN
    //       THIS FUNCTION MUST BE NON-BLOCKING.

    // Initilise a variable for the pose data of this agent
    CrazyflieData poseDataForThisAgent;

    // Extract the pose data from the full motion capture array
    // NOTE: that if the return index is a negative then this
    //       indicates that the pose data was not found.
    m_poseDataIndex = getPoseDataForObjectNameWithExpectedIndex( viconData, m_context.crazyflieName , m_poseDataIndex , poseDataForThisAgent );



    // switch(m_flying_state) 
    // {
    //     case STATE_MOTORS_OFF: // here we will put the code of current disabled button
    //         if(m_changed_flying_state_flag) // stuff that will be run only once when changing state
    //         {
    //             m_changed_flying_state_flag = false;
    //             ROS_INFO("[FLYING AGENT CLIENT] STATE_MOTORS_OFF");
    //         }
    //         break;
    //     case STATE_TAKE_OFF: //we need to move up from where we are now.
    //         if(m_changed_flying_state_flag) // stuff that will be run only once when changing state
    //         {
    //             m_changed_flying_state_flag = false;
    //             takeOffCF(local);
    //             finished_take_off = false;
    //             ROS_INFO("[FLYING AGENT CLIENT] STATE_TAKE_OFF");
    //             timer_takeoff = nodeHandle.createTimer(ros::Duration(yaml_duration_take_off), takeOffTimerCallback, true);
    //         }
    //         if(finished_take_off)
    //         {
    //             finished_take_off = false;
    //             setInstantController(getControllerNominallySelected());
    //             changeFlyingStateTo(STATE_FLYING);
    //         }
    //         break;
    //     case STATE_FLYING:
    //         if(m_changed_flying_state_flag) // stuff that will be run only once when changing state
    //         {
    //             m_changed_flying_state_flag = false;
    //             // need to change setpoint to the controller one
    //             goToControllerSetpoint();
    //             ROS_INFO("[FLYING AGENT CLIENT] STATE_FLYING");
    //         }
    //         break;
    //     case STATE_LAND:
    //         if(m_changed_flying_state_flag) // stuff that will be run only once when changing state
    //         {
    //             m_changed_flying_state_flag = false;
    //             landCF(local);
    //             finished_land = false;
    //             ROS_INFO("[FLYING AGENT CLIENT] STATE_LAND");
    //             timer_takeoff = nodeHandle.createTimer(ros::Duration(yaml_duration_landing), landTimerCallback, true);
    //         }
    //         if(finished_land)
    //         {
    //             finished_land = false;
    //             setInstantController(getControllerNominallySelected());
    //             changeFlyingStateTo(STATE_MOTORS_OFF);
    //         }
    //         break;
    //     default:
    //         break;
    // }

    // Only continue if:
    // (1) the pose for this agent was found, and
    // (2) the flying state is something other than MOTORS-OFF
    if (
        (m_poseDataIndex >= 0)
        and
        (m_flying_state != STATE_MOTORS_OFF)
        and
        (m_controllers_avialble)
    )
    {
        // Initliase the "Contrller" service call variable
        Controller controllerCall;

        // Fill in the pose data for this agent
        controllerCall.request.ownCrazyflie = poseDataForThisAgent;

        // Initialise a node handle, needed for starting timers
        ros::NodeHandle nodeHandle("~");

        // If the object is NOT occluded,
        // then proceed to make the "Controller Service Call" that
        // compute the controller output.
        if(!poseDataForThisAgent.occluded)
        {
            // PERFORM THE SAFTY CHECK (IF NOT THE DEFAULT CONTROLLER)
            if ( getInstantController() != DEFAULT_CONTROLLER )
            {
                if ( !safetyCheck(poseDataForThisAgent) )
                {
                    setInstantController(DEFAULT_CONTROLLER);
                    ROS_INFO_STREAM("[FLYING AGENT CLIENT] safety check failed, switching to DEFAULT CONTROLLER");
                }
            }

            // Initialise a local boolean success variable
            bool isSuccessful_controllerCall = false;

            // switch( getInstantController() )
            // {
            //     case DEFAULT_CONTROLLER
            //         isSuccessful_controllerCall = defaultController.call(controllerCall);
            //         break;
            //     case DEMO_CONTROLLER:
            //         isSuccessful_controllerCall = demoController.call(controllerCall);
            //         break;
            //     case STUDENT_CONTROLLER:
            //         isSuccessful_controllerCall = studentController.call(controllerCall);
            //         break;
            //     case MPC_CONTROLLER:
            //         isSuccessful_controllerCall = mpcController.call(controllerCall);
            //         break;
            //     case REMOTE_CONTROLLER:
            //         isSuccessful_controllerCall = remoteController.call(controllerCall);
            //         break;
            //     case TUNING_CONTROLLER:
            //         isSuccessful_controllerCall = tuningController.call(controllerCall);
            //         break;
            //     case PICKER_CONTROLLER:
            //         isSuccessful_controllerCall = pickerController.call(controllerCall);
            //         break;
            //     case TEMPLATE_CONTROLLER:
            //         isSuccessful_controllerCall = templateController.call(controllerCall);
            //         break;
            //     default:
            //         ROS_ERROR("[FLYING AGENT CLIENT] the current controller was not recognised");
            //         break;
            // }


            isSuccessful_controllerCall = m_instant_controller_service_client->call(controllerCall);

            // Ensure success and enforce safety
            if(!isSuccessful_controllerCall)
            {
                //ROS_ERROR("[FLYING AGENT CLIENT] Failed to call a 'non-safe' controller, switching to safe controller");
                //ROS_ERROR_STREAM("[FLYING AGENT CLIENT] 'non-safe' controller status: valid: " << demoController.isValid() << ", exists: " << demoController.exists());
                //ROS_ERROR_STREAM("[FLYING AGENT CLIENT] 'non-safe' controller name: " << demoController.getService());

                // Let the user know that the controller call failed
                ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Failed to call controller, valid: " << m_instant_controller_service_client->isValid() << ", exists: " << m_instant_controller_service_client->exists());

                // Switch to the default controller,
                // if it was not already the active controller
                if ( getInstantController() != DEFAULT_CONTROLLER )
                {
                    // Set the DEFAULT controller to be active
                    setInstantController(DEFAULT_CONTROLLER);
                    // Try the controller call
                    isSuccessful_controllerCall = m_instant_controller_service_client->call(controllerCall);
                    // Inform the user is not successful
                    if ( !isSuccessful_controllerCall )
                    {
                        ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Also failed to call the DEFAULT controller, valid: " << m_instant_controller_service_client->isValid() << ", exists: " << m_instant_controller_service_client->exists());
                    }
                }
            }

            // Send the command to the CrazyRadio
            // > IF SUCCESSFUL
            if (isSuccessful_controllerCall)
            {
                commandForSendingToCrazyfliePublisher.publish(controllerCall.response.controlOutput);
            }
            // > ELSE SEND ZERO OUTPUT COMMAND
            else
            {
                // Send the command to turn the motors off
                ControlCommand zeroOutput = ControlCommand(); //everything set to zero
                zeroOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS; //set to motor_mode
                commandForSendingToCrazyfliePublisher.publish(zeroOutput);
                // And change the state to motor-off
                changeFlyingStateTo(STATE_MOTORS_OFF);
            }



            // else if(!safetyCheck(global, controllerCall.response.controlOutput))
            // {
            //     setInstantController(SAFE_CONTROLLER);
            //     ROS_INFO_STREAM("[FLYING AGENT CLIENT] safety check failed, switching to safe controller");
            // }



            // // Ensure success and enforce safety
            // if(!success)
            // {
            //     ROS_ERROR("[FLYING AGENT CLIENT] Failed to call a 'non-safe' controller, switching to safe controller");
            //     //ROS_ERROR_STREAM("[FLYING AGENT CLIENT] 'non-safe' controller status: valid: " << demoController.isValid() << ", exists: " << demoController.exists());
            //     //ROS_ERROR_STREAM("[FLYING AGENT CLIENT] 'non-safe' controller name: " << demoController.getService());
            //     setInstantController(SAFE_CONTROLLER);
            // }
            // else if(!safetyCheck(global, controllerCall.response.controlOutput))
            // {
            //     setInstantController(SAFE_CONTROLLER);
            //     ROS_INFO_STREAM("[FLYING AGENT CLIENT] safety check failed, switching to safe controller");
            // }

                
            // }
            // // SAFE_CONTROLLER and state is different from flying
            // else
            // {
            //     calculateDistanceToCurrentSafeSetpoint(local); // update distance, it also updates the unit vector
            //     // ROS_INFO_STREAM("distance: " << distance);
            //     // here, detect if euclidean distance between setpoint and current position is higher than a threshold
            //     if(distance > yaml_distance_threshold)
            //     {
            //         // DEBUGGING: display a message that the crazyflie is inside the thresholds
            //         //ROS_INFO("inside threshold");
            //         // Declare a local variable for the "setpoint message" to be published
            //         Setpoint setpoint_msg;
            //         // here, where we are now, or where we were in the beginning?
            //         setpoint_msg.x = local.x + yaml_distance_threshold * unit_vector[0];
            //         setpoint_msg.y = local.y + yaml_distance_threshold * unit_vector[1];
            //         setpoint_msg.z = local.z + yaml_distance_threshold * unit_vector[2];

            //         // yaw is better divided by the number of steps?
            //         setpoint_msg.yaw = current_safe_setpoint.yaw;
            //         safeControllerServiceSetpointPublisher.publish(setpoint_msg);
            //         was_in_threshold = true;
            //     }
            //     else
            //     {
            //         if(was_in_threshold)
            //         {
            //             was_in_threshold = false;
            //             safeControllerServiceSetpointPublisher.publish(current_safe_setpoint);
            //             // goToControllerSetpoint(); //maybe this is a bit repetitive?
            //         }
            //     }

            //     bool success = safeController.call(controllerCall);
            //     if(!success) {
            //         ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Failed to call safe controller, valid: " << safeController.isValid() << ", exists: " << safeController.exists());
            //     }
            // }

            // Send the command to the CrazyRadio
            //commandForSendingToCrazyfliePublisher.publish(controllerCall.response.controlOutput);

        } // END OF: "if(!global.occluded)"

    }
    else
    {
        // Send the command to turn the motors off
        ControlCommand zeroOutput = ControlCommand(); //everything set to zero
        zeroOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS; //set to motor_mode
        commandForSendingToCrazyfliePublisher.publish(zeroOutput);

	} // END OF: "if ( (m_poseDataIndex >= 0) and (m_flying_state != STATE_MOTORS_OFF) )"

}





int getPoseDataForObjectNameWithExpectedIndex(const ViconData& viconData, std::string name , int expected_index , CrazyflieData& pose_to_fill_in)
{
    // Initialise an integer for the index where the object
    // "name" is found
    // > Initialise an negative to indicate not found
    int object_index = -1;

    // Get the length of the "pose data array"
    int length_poseData = viconData.crazyflies.size();

    // If the "expected index" is non-negative and less than
    // the length of the data array, then attempt to check
    // for a name match
    if (
        (0 <= expected_index)
        and
        (expected_index < length_poseData)
    )
    {
        // Check if the names match
        if (viconData.crazyflies[expected_index].crazyflieName == m_context.crazyflieName)
        {
            object_index = expected_index;
        }
    }

    // If not found, then iterate the data array looking
    // for a name match
    if (object_index < 0)
    {
        for( int i=0 ; i<length_poseData ; i++ )
        {    
            // Check if the names match
            if(viconData.crazyflies[i].crazyflieName == m_context.crazyflieName)
            {
                object_index = i;
            }
        }
    }

    // If not found, then retrun, else fill in the pose data
    if (object_index < 0)
    {
        return object_index;
    }
    else
    {
        pose_to_fill_in = viconData.crazyflies[object_index];
        coordinatesToLocal(pose_to_fill_in);
        return object_index;
    }
}


void coordinatesToLocal(CrazyflieData& cf)
{
    AreaBounds area = m_context.localArea;
    float originX = (area.xmin + area.xmax) / 2.0;
    float originY = (area.ymin + area.ymax) / 2.0;
    // change Z origin to zero, i.e., to the table height, zero of global coordinates, instead of middle of the box
    float originZ = 0.0;
    // float originZ = (area.zmin + area.zmax) / 2.0;

    cf.x -= originX;
    cf.y -= originY;
    cf.z -= originZ;
}









void changeFlyingStateTo(int new_state)
{
    if(crazyradio_status == CRAZY_RADIO_STATE_CONNECTED)
    {
        ROS_INFO("[FLYING AGENT CLIENT] Change state to: %d", new_state);
        m_flying_state = new_state;
    }
    else
    {
        ROS_INFO("[FLYING AGENT CLIENT] Disconnected and trying to change state. State goes to MOTORS OFF");
        m_flying_state = STATE_MOTORS_OFF;
    }

    // Set the class variable flag that the
    // flying state was changed
    m_changed_flying_state_flag = true;

    // Publish a message with the new flying state
    std_msgs::Int32 flying_state_msg;
    flying_state_msg.data = m_flying_state;
    flyingStatePublisher.publish(flying_state_msg);
}


void takeOffTimerCallback(const ros::TimerEvent&)
{
    //finished_take_off = true;
}

void landTimerCallback(const ros::TimerEvent&)
{
    //finished_land = true;
}

























//    ----------------------------------------------------------------------------------
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//    ----------------------------------------------------------------------------------





// void loadSafeController() {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string safeControllerName;
//     if(!nodeHandle.getParam("safeController", safeControllerName)) {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get safe controller name");
//         return;
//     }

//     ros::service::waitForService(safeControllerName);
//     safeController = ros::service::createClient<Controller>(safeControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] loaded safe controller: " << safeController.getService());
// }

// void loadDemoController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string demoControllerName;
//     if(!nodeHandle.getParam("demoController", demoControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get demo controller name");
//         return;
//     }

//     demoController = ros::service::createClient<Controller>(demoControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded demo controller " << demoController.getService());
// }

// void loadStudentController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string studentControllerName;
//     if(!nodeHandle.getParam("studentController", studentControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get student controller name");
//         return;
//     }

//     studentController = ros::service::createClient<Controller>(studentControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded student controller " << studentController.getService());
// }

// void loadMpcController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string mpcControllerName;
//     if(!nodeHandle.getParam("mpcController", mpcControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get mpc controller name");
//         return;
//     }

//     mpcController = ros::service::createClient<Controller>(mpcControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded mpc controller " << mpcController.getService());
// }

// void loadRemoteController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string remoteControllerName;
//     if(!nodeHandle.getParam("remoteController", remoteControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get remote controller name");
//         return;
//     }

//     remoteController = ros::service::createClient<Controller>(remoteControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded remote controller " << remoteController.getService());
// }

// void loadTuningController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string tuningControllerName;
//     if(!nodeHandle.getParam("tuningController", tuningControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get tuning controller name");
//         return;
//     }

//     tuningController = ros::service::createClient<Controller>(tuningControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded tuning controller " << tuningController.getService());
// }

// void loadPickerController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string pickerControllerName;
//     if(!nodeHandle.getParam("pickerController", pickerControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get picker controller name");
//         return;
//     }

//     pickerController = ros::service::createClient<Controller>(pickerControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded picker controller " << pickerController.getService());
// }

// void loadTemplateController()
// {
//     //ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
//     ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

//     std::string templateControllerName;
//     if(!nodeHandle.getParam("templateController", templateControllerName))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get template controller name");
//         return;
//     }

//     templateController = ros::service::createClient<Controller>(templateControllerName, true);
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded template controller " << templateController.getService());
// }





// CREATE A SERVICE CLIENT
// NOTE: the second argument is a boolean that specifies whether the 
//       service is persistent or not. In the ROS documentation a
//       persistent connection is described as:
//   "Persistent connections should be used carefully. They greatly
//    improve performance for repeated requests, but they also make
//    your client more fragile to service failures. Clients using
//    persistent connections should implement their own reconnection
//    logic in the event that the persistent connection fails."
void loadController( std::string paramter_name , ros::ServiceClient& serviceClient )
{
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
    ros::NodeHandle nodeHandle(nodeHandle_to_own_agent_parameter_service, "ClientConfig");

    std::string controllerName;
    if(!nodeHandle.getParam(paramter_name, controllerName))
    {
        ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Failed to get \"" << paramter_name << "\" paramter");
        return;
    }

    serviceClient = ros::service::createClient<Controller>(controllerName, true);
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Loaded service: " << serviceClient.getService() <<  ", valid: " << serviceClient.isValid() << ", exists: " << serviceClient.exists() );
}





void sendMessageUsingController(int controller)
{
    // Send a message on the topic for informing the Flying
    // Agent GUI about this update
    std_msgs::Int32 controller_used_msg;
    controller_used_msg.data = controller;
    controllerUsedPublisher.publish(controller_used_msg);
}

void setInstantController(int controller) //for right now, temporal use
{
    // Update the class variable
    m_instant_controller = controller;

    

    switch(controller)
    {
        case DEFAULT_CONTROLLER:
            m_instant_controller_service_client = &defaultController;
            break;
        case STUDENT_CONTROLLER:
            m_instant_controller_service_client = &studentController;
            break;
        case TUNING_CONTROLLER:
            m_instant_controller_service_client = &tuningController;
            break;
        case PICKER_CONTROLLER:
            m_instant_controller_service_client = &pickerController;
            break;
        case TEMPLATE_CONTROLLER:
            m_instant_controller_service_client = &templateController;
            break;
        default:
            break;
    }

    sendMessageUsingController(controller);
    // switch(controller)
    // {
    //     case SAFE_CONTROLLER:
    //         loadSafeController();
    //         break;
    //     case DEMO_CONTROLLER:
    //         loadDemoController();
    //         break;
    //     case STUDENT_CONTROLLER:
    //         loadStudentController();
    //         break;
    //     case MPC_CONTROLLER:
    //         loadMpcController();
    //         break;
    //     case REMOTE_CONTROLLER:
    //         loadRemoteController();
    //         break;
    //     case TUNING_CONTROLLER:
    //         loadTuningController();
    //         break;
    //     case PICKER_CONTROLLER:
    //         loadPickerController();
    //         break;
    //     case TEMPLATE_CONTROLLER:
    //         loadTemplateController();
    //         break;
    //     default:
    //         break;
    // }
}

int getInstantController()
{
    return m_instant_controller;
}

void setControllerNominallySelected(int controller)
{
    // Update the class variable
    m_controller_nominally_selected = controller;

    // If in state "MOTORS-OFF" or "FLYING",
    // then the change is instant.
    if(m_flying_state == STATE_MOTORS_OFF || m_flying_state == STATE_FLYING)
    {

        setInstantController(controller); 
    }
}

int getControllerNominallySelected()
{
    return m_controller_nominally_selected;
}










void flyingStateRequestCallback(const IntWithHeader & msg) {

    // Check whether the message is relevant
    bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

    // Continue if the message is relevant
    if (isRevelant)
    {
        // Extract the data
        int cmd = msg.data;

        switch(cmd) {
            case CMD_USE_DEFAULT_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_DEFAULT_CONTROLLER Command received");
                setControllerNominallySelected(DEFAULT_CONTROLLER);
                break;

            case CMD_USE_DEMO_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_DEMO_CONTROLLER Command received");
                setControllerNominallySelected(DEMO_CONTROLLER);
                break;

            case CMD_USE_STUDENT_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_STUDENT_CONTROLLER Command received");
                setControllerNominallySelected(STUDENT_CONTROLLER);
                break;

            case CMD_USE_MPC_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_MPC_CONTROLLER Command received");
                setControllerNominallySelected(MPC_CONTROLLER);
                break;

            case CMD_USE_REMOTE_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_REMOTE_CONTROLLER Command received");
                setControllerNominallySelected(REMOTE_CONTROLLER);
                break;

            case CMD_USE_TUNING_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_TUNING_CONTROLLER Command received");
                setControllerNominallySelected(TUNING_CONTROLLER);
                break;

            case CMD_USE_PICKER_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_PICKER_CONTROLLER Command received");
                setControllerNominallySelected(PICKER_CONTROLLER);
                break;

            case CMD_USE_TEMPLATE_CONTROLLER:
                ROS_INFO("[FLYING AGENT CLIENT] USE_TEMPLATE_CONTROLLER Command received");
                setControllerNominallySelected(TEMPLATE_CONTROLLER);
                break;

            case CMD_CRAZYFLY_TAKE_OFF:
                ROS_INFO("[FLYING AGENT CLIENT] TAKE_OFF Command received");
                if(m_flying_state == STATE_MOTORS_OFF)
                {
                    changeFlyingStateTo(STATE_TAKE_OFF);
                }
                break;

            case CMD_CRAZYFLY_LAND:
                ROS_INFO("[FLYING AGENT CLIENT] LAND Command received");
                if(m_flying_state != STATE_MOTORS_OFF)
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








void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] Received message with Crazy Radio Status = " << msg.data );
    crazyradio_status = msg.data;
}






void emergencyStopReceivedCallback(const IntWithHeader & msg)
{
    ROS_INFO("[FLYING AGENT CLIENT] Received message to EMERGENCY STOP");
    flyingStateRequestCallback(msg);
}






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
    response.data = m_flying_state;
    // Return
    return true;
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
        if (m_flying_state != STATE_MOTORS_OFF)
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







//    ----------------------------------------------------------------------------------
//     SSSS    A    FFFFF  EEEEE  TTTTT  Y   Y
//    S       A A   F      E        T     Y Y
//     SSS   A   A  FFF    EEE      T      Y
//        S  AAAAA  F      E        T      Y
//    SSSS   A   A  F      EEEEE    T      Y
//
//     CCCC  H   H  EEEEE   CCCC  K   K   SSSS
//    C      H   H  E      C      K  K   S
//    C      HHHHH  EEE    C      KKK     SSS
//    C      H   H  E      C      K  K       S
//     CCCC  H   H  EEEEE   CCCC  K   K  SSSS
//    ----------------------------------------------------------------------------------

// Checks if crazyflie is within allowed area
bool safetyCheck(CrazyflieData data)
{
    // Check on the X position
    if((data.x < m_context.localArea.xmin) or (data.x > m_context.localArea.xmax))
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] x safety failed");
        return false;
    }
    // Check on the Y position
    if((data.y < m_context.localArea.ymin) or (data.y > m_context.localArea.ymax))
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] y safety failed");
        return false;
    }
    // Check on the Z position
    if((data.z < m_context.localArea.zmin) or (data.z > m_context.localArea.zmax))
    {
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] z safety failed");
        return false;
    }

    // Check the title angle (if required)
    // > The tilt anlge between the body frame and inertial frame z-axis is
    //   give by:
    //   tilt angle  =  1 / ( cos(roll)*cos(pitch) )
    // > But this would be too sensitve to a divide by zero error, so instead
    //   we just check if each angle separately exceeds the limit
    if(yaml_isEnabled_strictSafety)
    {
        // Check on the ROLL angle
        if(
            (data.roll > m_maxTiltAngle_for_strictSafety_redians)
            or
            (data.roll < -m_maxTiltAngle_for_strictSafety_redians)
        )
        {
            ROS_INFO_STREAM("[FLYING AGENT CLIENT] roll too big.");
            return false;
        }
        // Check on the PITCH angle
        if(
            (data.pitch > m_maxTiltAngle_for_strictSafety_redians)
            or
            (data.pitch < -m_maxTiltAngle_for_strictSafety_redians)
        )
        {
            ROS_INFO_STREAM("[FLYING AGENT CLIENT] pitch too big.");
            return false;
        }
    }

    // If the code makes it to here then all the safety checks passed,
    // Hence return "true"
    return true;
}





//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//     CCCC   OOO   N   N  TTTTT  EEEEE  X   X  TTTTT
//    C      O   O  NN  N    T    E       X X     T
//    C      O   O  N N N    T    EEE      X      T
//    C      O   O  N  NN    T    E       X X     T
//     CCCC   OOO   N   N    T    EEEEE  X   X    T
//    ----------------------------------------------------------------------------------


void crazyflieContextDatabaseChangedCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("[FLYING AGENT CLIENT] Received message that the Context Database Changed");
    loadCrazyflieContext();
}



void loadCrazyflieContext()
{
    CMQuery contextCall;
    contextCall.request.studentID = m_agentID;
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] AgentID:" << m_agentID);

    CrazyflieContext new_context;

    centralManager.waitForExistence(ros::Duration(-1));

    if(centralManager.call(contextCall)) {
        new_context = contextCall.response.crazyflieContext;
        ROS_INFO_STREAM("[FLYING AGENT CLIENT] CrazyflieContext:\n" << new_context);

        if((m_context.crazyflieName != "") && (new_context.crazyflieName != m_context.crazyflieName)) //linked crazyflie name changed and it was not empty before
        {

            // Motors off is done in python script now everytime we disconnect

            // send motors OFF and disconnect before setting m_context = new_context
            // std_msgs::Int32 msg;
            // msg.data = CMD_CRAZYFLY_MOTORS_OFF;
            // commandPublisher.publish(msg);

            ROS_INFO("[FLYING AGENT CLIENT] CF is now different for this student. Disconnect and turn it off");

            IntWithHeader msg;
            msg.shouldCheckForAgentID = false;
            msg.data = CMD_DISCONNECT;
            crazyRadioCommandPublisher.publish(msg);
        }

        m_context = new_context;

        ros::NodeHandle nh("CrazyRadio");
        nh.setParam("crazyFlieAddress", m_context.crazyflieAddress);
    }
    else
    {
        ROS_ERROR("[FLYING AGENT CLIENT] Failed to load context. Waiting for next Save in DB by teacher");
    }
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



// void isReadySafeControllerYamlCallback(const IntWithHeader & msg)
// {
//     // Check whether the message is relevant
//     bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

//     // Continue if the message is relevant
//     if (isRevelant)
//     {
//         // Extract the data
//         int parameter_service_to_load_from = msg.data;
//         // Initialise a local variable for the namespace
//         std::string namespace_to_use;
//         // Load from the respective parameter service
//         switch(parameter_service_to_load_from)
//         {
//             // > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
//             case LOAD_YAML_FROM_AGENT:
//             {
//                 ROS_INFO("[FLYING AGENT CLIENT] Now fetching the SafeController YAML parameter values from this agent.");
//                 namespace_to_use = m_namespace_to_own_agent_parameter_service;
//                 break;
//             }
//             // > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
//             case LOAD_YAML_FROM_COORDINATOR:
//             {
//                 ROS_INFO("[FLYING AGENT CLIENT] Now fetching the SafeController YAML parameter values from this agent's coordinator.");
//                 namespace_to_use = m_namespace_to_coordinator_parameter_service;
//                 break;
//             }

//             default:
//             {
//                 ROS_ERROR("[FLYING AGENT CLIENT] Paramter service to load from was NOT recognised.");
//                 namespace_to_use = m_namespace_to_own_agent_parameter_service;
//                 break;
//             }
//         }
//         // Create a node handle to the selected parameter service
//         ros::NodeHandle nodeHandle_to_use(namespace_to_use);
//         // Call the function that fetches the parameters
//         fetchSafeControllerYamlParameters(nodeHandle_to_use);
//     }
// }



// void fetchSafeControllerYamlParameters(ros::NodeHandle& nodeHandle)
// {
//     // Here we load the parameters that are specified in the file:
//     // SafeController.yaml

//     // Add the "SafeController" namespace to the "nodeHandle"
//     ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "SafeController");

//     // take off and landing parameters (in meters and seconds)
//     yaml_take_off_distance = getParameterFloat(nodeHandle_for_paramaters,"takeOffDistance");
//     yaml_landing_distance = getParameterFloat(nodeHandle_for_paramaters,"landingDistance");
//     yaml_duration_take_off = getParameterFloat(nodeHandle_for_paramaters,"durationTakeOff");
//     yaml_duration_landing = getParameterFloat(nodeHandle_for_paramaters,"durationLanding");
//     yaml_distance_threshold = getParameterFloat(nodeHandle_for_paramaters,"distanceThreshold");

//     // setpoint in meters (x, y, z, yaw)
//     getParameterFloatVector(nodeHandle_for_paramaters,"defaultSetpoint",yaml_default_setpoint,4);

//     // DEBUGGING: Print out one of the parameters that was loaded
//     ROS_INFO_STREAM("[FLYING AGENT CLIENT] DEBUGGING: the fetched SafeController/durationTakeOff = " << yaml_duration_take_off);

//     /*
//     // Here we load the parameters that are specified in the SafeController.yaml file

//     // Add the "SafeController" namespace to the "nodeHandle"
//     ros::NodeHandle nodeHandle_for_safeController(nodeHandle, "SafeController");

//     if(!nodeHandle_for_safeController.getParam("takeOffDistance", take_off_distance))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get takeOffDistance");
//     }

//     if(!nodeHandle_for_safeController.getParam("landingDistance", landing_distance))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get landing_distance");
//     }

//     if(!nodeHandle_for_safeController.getParam("durationTakeOff", duration_take_off))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get duration_take_off");
//     }

//     if(!nodeHandle_for_safeController.getParam("durationLanding", duration_landing))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get duration_landing");
//     }

//     if(!nodeHandle_for_safeController.getParam("distanceThreshold", distance_threshold))
//     {
//         ROS_ERROR("[FLYING AGENT CLIENT] Failed to get distance_threshold");
//     }
//     */
// }




void isReadyClientConfigYamlCallback(const IntWithHeader & msg)
{
    // Check whether the message is relevant
    bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

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
    yaml_isEnabled_strictSafety = getParameterBool(nodeHandle_for_paramaters,"isEnabled_strictSafety");
    yaml_maxTiltAngle_for_strictSafety_degrees = getParameterFloat(nodeHandle_for_paramaters,"maxTiltAngle_for_strictSafety_degrees");
    
    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] DEBUGGING: the fetched ClientConfig/isEnabled_strictSafety = " << yaml_isEnabled_strictSafety);



    // PROCESS THE PARAMTERS
    // Convert from degrees to radians
    m_maxTiltAngle_for_strictSafety_redians = DEG2RAD * yaml_maxTiltAngle_for_strictSafety_degrees;

    // DEBUGGING: Print out one of the processed values
    ROS_INFO_STREAM("[PICKER CONTROLLER] DEBUGGING: after processing m_maxTiltAngle_for_strictSafety_redians = " << m_maxTiltAngle_for_strictSafety_redians);
}




void timerCallback_for_loadAllControllers(const ros::TimerEvent&)
{
    // INITIALISE ALL THE CONTROLLER SERVICE CLIENTS
    loadController( "defaultController"  , defaultController );
    loadController( "studentController"  , studentController );
    loadController( "tuningController"   , tuningController );
    loadController( "pickerController"   , pickerController );
    loadController( "templateController" , templateController );

    if (defaultController)
    {
        m_controllers_avialble = true;
    }
    else
    {
        m_controllers_avialble = false;
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

    //ros::Subscriber safeController_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "SafeController", 1, isReadySafeControllerYamlCallback);
    //ros::Subscriber safeController_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("SafeController", 1, isReadySafeControllerYamlCallback);



    // GIVE YAML VARIABLES AN INITIAL VALUE

	// This can be done either here or as part of declaring the
	// variables in the header file



	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

    // Call the class function that loads the parameters for this class.
    fetchClientConfigYamlParameters(nodeHandle_to_own_agent_parameter_service);
    //fetchSafeControllerYamlParameters(nodeHandle_to_own_agent_parameter_service);


    // INITIALISE ALL THE CONTROLLER SERVICE CLIENTS
    // loadController( "defaultController"  , defaultController );
    // loadController( "studentController"  , studentController );
    // loadController( "tuningController"   , tuningController );
    // loadController( "pickerController"   , pickerController );
    // loadController( "templateController" , templateController );
    m_controllers_avialble = false;
    timer_for_loadAllControllers = nodeHandle.createTimer(ros::Duration(3), timerCallback_for_loadAllControllers, true);
    





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
    ros::Subscriber emergencyStopSubscriber = nodeHandle_dfall_root.subscribe("emergencyStop", 1, emergencyStopReceivedCallback);

    // LOCALISATION DATA FROM MOTION CAPTURE SYSTEM
	//keeps 100 messages because otherwise ViconDataPublisher would override the data immediately
	ros::Subscriber viconSubscriber = nodeHandle_dfall_root.subscribe("ViconDataPublisher/ViconData", 100, viconCallback);



    // PUBLISHER FOR COMMANDING THE CRAZYFLIE
    // i.e., motorCmd{1,2,3,4}, roll, pitch, yaw, and onboardControllerType
	//ros::Publishers to advertise the control output
	commandForSendingToCrazyfliePublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);

	//this topic lets the FlyingAgentClient listen to the terminal commands
    //commandPublisher = nodeHandle.advertise<std_msgs::Int32>("Command", 1);



    // SUBSCRIBER FOR THE CHANGE STATE COMMANDS
    // i.e., {TAKE-OFF,LAND,MOTORS-OFF,CONTROLLER SELECTION}
    // > for the agent GUI
    ros::Subscriber commandSubscriber_to_agent = nodeHandle.subscribe("Command", 1, flyingStateRequestCallback);
    // > for the coord GUI
    ros::Subscriber commandSubscriber_to_coord = nodeHandle_to_coordinator.subscribe("FlyingAgentClient/Command", 1, flyingStateRequestCallback);




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
    flying_state_msg.data = m_flying_state;
    flyingStatePublisher.publish(flying_state_msg);

    // SafeControllerServicePublisher:
    ros::NodeHandle namespaceNodeHandle = ros::NodeHandle();
    //safeControllerServiceSetpointPublisher = namespaceNodeHandle.advertise<dfall_pkg::Setpoint>("SafeControllerService/Setpoint", 1);
    //ros::Subscriber controllerSetpointSubscriber = namespaceNodeHandle.subscribe("student_GUI/ControllerSetpoint", 1, controllerSetPointCallback);
    //ros::Subscriber safeSetpointSubscriber = namespaceNodeHandle.subscribe("SafeControllerService/Setpoint", 1, safeSetPointCallback);

    

    
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
    m_flying_state = STATE_MOTORS_OFF;
    setControllerNominallySelected(DEFAULT_CONTROLLER);
    setInstantController(DEFAULT_CONTROLLER); //initialize this also, so we notify GUI


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
