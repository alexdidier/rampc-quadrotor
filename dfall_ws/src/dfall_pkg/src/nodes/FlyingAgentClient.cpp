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






//    ----------------------------------------------------------------------------------
//    M   M   OOO   TTTTT  III   OOO   N   N
//    MM MM  O   O    T     I   O   O  NN  N
//    M M M  O   O    T     I   O   O  N N N
//    M   M  O   O    T     I   O   O  N  NN
//    M   M   OOO     T    III   OOO   N   N
//
//     CCCC    A    PPPP   TTTTT  U   U  RRRR   EEEEE
//    C       A A   P   P    T    U   U  R   R  E
//    C      A   A  PPPP     T    U   U  RRRR   EEE
//    C      AAAAA  P        T    U   U  R   R  E
//     CCCC  A   A  P        T     UUU   R   R  EEEEE
//    ----------------------------------------------------------------------------------

// CALLBACK RUN EVERY TIME NEW MOTION CAPTURE DATA IS RECEIVED
void viconCallback(const ViconData& viconData)
{
	// NOTE: THIS FUNCTION IS CALL AT THE FREQUENCY OF THE MOTION
	//       CAPTURE SYSTEM. HENCE ANY OPTERATIONS PERFORMED IN
	//       THIS FUNCTION MUST BE NON-BLOCKING.

	// Initialise a counter of consecutive frames of motion
	// capture data where the agent is occuled
	static int number_of_consecutive_occulsions = 0;

	// Initilise a variable for the pose data of this agent
	CrazyflieData poseDataForThisAgent;

	// Extract the pose data from the full motion capture array
	// NOTE: that if the return index is a negative then this
	//       indicates that the pose data was not found.
	m_poseDataIndex = getPoseDataForObjectNameWithExpectedIndex( viconData, m_context.crazyflieName , m_poseDataIndex , poseDataForThisAgent );


	// Detecting time-out of the motion capture data
	// > Update the flag
	m_isAvailable_mocap_data = true;
	// > Stop any previous instance that might still be running
	m_timer_mocap_timeout_check.stop();
	// > Set the period again (second argument is reset)
	m_timer_mocap_timeout_check.setPeriod( ros::Duration(yaml_mocap_timeout_duration), true);
	// > Start the timer again
	m_timer_mocap_timeout_check.start();


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

		// If the object is NOT occluded,
		// then proceed to make the "Controller Service Call" that
		// compute the controller output.
		if(!poseDataForThisAgent.occluded)
		{
			// Reset the "consecutive occulsions" flag
			number_of_consecutive_occulsions = 0;

			// PERFORM THE SAFTY CHECK (IF NOT THE DEFAULT CONTROLLER)
			if ( getInstantController() != DEFAULT_CONTROLLER )
			{
				if ( !safetyCheck_on_positionAndTilt(poseDataForThisAgent) )
				{
					setInstantController(DEFAULT_CONTROLLER);
					ROS_INFO_STREAM("[FLYING AGENT CLIENT] safety check failed, switching to DEFAULT CONTROLLER");
				}
			}

			// Initialise a local boolean success variable
			bool isSuccessful_controllerCall = false;

			// Call the controller service client
			isSuccessful_controllerCall = m_instant_controller_service_client->call(controllerCall);

			// Ensure success and enforce safety
			if(!isSuccessful_controllerCall)
			{
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
				m_commaNdfOrsendiNgTocrazyflIepublisher.publish(controllerCall.response.controlOutput);
			}
			// > ELSE SEND ZERO OUTPUT COMMAND
			else
			{
				// Send the command to turn the motors off
				sendZeroOutputCommandForMotors();
				// And change the state to motor-off
				requestChangeFlyingStateTo(STATE_MOTORS_OFF);
			}
		}
		else
		{
			// Increment the number of consective occulations
			number_of_consecutive_occulsions++;
			// Update the flag if this exceeds the threshold
			if (
				(!m_isOcculded_mocap_data)
				and
				(number_of_consecutive_occulsions > yaml_consecutive_occulsions_threshold)
			)
			{
				// Update the flag
				m_isOcculded_mocap_data = true;
				// Send the command to turn the motors off
				sendZeroOutputCommandForMotors();
				// Update the flying state
				requestChangeFlyingStateTo( STATE_MOTORS_OFF );
			}
		} // END OF: "if(!poseDataForThisAgent.occluded)"

	}
	else
	{
		// Send the command to turn the motors off
		sendZeroOutputCommandForMotors();

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


void timerCallback_mocap_timeout_check(const ros::TimerEvent&)
{
	// Update the flag
	m_isAvailable_mocap_data = false;
	// Inform the user
	ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Motion Capture Data has been unavailable for " << yaml_mocap_timeout_duration << " seconds." );
	// Ensure that the motors are turned off
	if ( !(m_flying_state==STATE_MOTORS_OFF) )
	{
		// Send the command to turn the motors off
        sendZeroOutputCommandForMotors();
        // Update the flying state
        requestChangeFlyingStateTo( STATE_MOTORS_OFF );
	}
}


void sendZeroOutputCommandForMotors()
{
	ControlCommand zeroOutput = ControlCommand(); //everything set to zero
    zeroOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS; //set to motor_mode
    m_commaNdfOrsendiNgTocrazyflIepublisher.publish(zeroOutput);
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
bool safetyCheck_on_positionAndTilt(CrazyflieData data)
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
//    FFFFF L      Y   Y  III  N   N   GGGG
//    F     L       Y Y    I   NN  N  G
//    FFF   L        Y     I   N N N  G
//    F     L        Y     I   N  NN  G   G
//    F     LLLLL    Y    III  N   N   GGGG
//
//     SSSS  TTTTT    A    TTTTT  EEEEE
//    S        T     A A     T    E
//     SSS     T    A   A    T    EEE
//        S    T    AAAAA    T    E
//    SSSS     T    A   A    T    EEEEE
//    ----------------------------------------------------------------------------------


void requestChangeFlyingStateTo(int new_state)
{
    if(m_crazyradio_status != CRAZY_RADIO_STATE_CONNECTED)
    {
        ROS_INFO("[FLYING AGENT CLIENT] Disconnected and trying to change state. State goes to MOTORS OFF");
        m_flying_state = STATE_MOTORS_OFF;
    }
    else
    {
        // Switch between the possible new states
        switch (new_state)
        {
        	case STATE_TAKE_OFF:
        	{
        		requestChangeFlyingStateToTakeOff();
        		break;
        	}

        	case STATE_FLYING:
        	{
        		// This should never be called
        		break;
        	}

        	case STATE_LAND:
        	{
        		requestChangeFlyingStateToLand();
        		break;
        	}

        	case STATE_MOTORS_OFF:
        	{
        		ROS_INFO("[FLYING AGENT CLIENT] Change state to MOTORS OFF");
        		m_flying_state = new_state;
        		break;
        	}

        	default:
        	{
        		ROS_ERROR_STREAM("[FLYING AGENT CLIENT] Request state of " << new_state << " not recognised. Hence changing to MOTORS OFF.");
        		m_flying_state = new_state;
        		break;
        	}
        }

    }

    // Publish a message with the new flying state
    std_msgs::Int32 flying_state_msg;
    flying_state_msg.data = m_flying_state;
    flyingStatePublisher.publish(flying_state_msg);
}

void requestChangeFlyingStateToTakeOff()
{
	// Only allow taking off from the MOTORS OFF state
	if (m_flying_state != STATE_MOTORS_OFF)
	{
		ROS_ERROR("[FLYING AGENT CLIENT] Request to TAKE OFF was not implemented because the current state is NOT the MOTORS OFF state.");
	}
	else
	{
		// Check that the Motion Capture data is available
		if ( m_isAvailable_mocap_data and !m_isOcculded_mocap_data )
		{
			// Check whether a "controller" take-off should
			// be performed, and that not already in the
			// "take-off" state
			if (
				(yaml_shouldPerfom_takeOff_with_defaultController)
				and
				(m_flying_state != STATE_TAKE_OFF)
			)
			{
				// Call the service offered by the default
				// controller for how long a take-off will take
				dfall_pkg::IntIntService requestManoeurveCall;
				requestManoeurveCall.request.data = DEFAULT_CONTROLLER_REQUEST_TAKE_OFF;
				if(m_defaultController_requestManoeuvre.call(requestManoeurveCall))
				{
					// Extract the duration
					float take_off_duration = float(requestManoeurveCall.response.data) / 1000.0;
					// Start the timer
					// > Stop any previous instance
					m_timer_takeoff_complete.stop();
					// > Set the period again (second argument is reset)
					m_timer_takeoff_complete.setPeriod( ros::Duration(take_off_duration), true);
					// > Start the timer
					m_timer_takeoff_complete.start();
					// Inform the user
					ROS_INFO_STREAM("[FLYING AGENT CLIENT] Changed state to STATE_TAKE_OFF for a duration of " << take_off_duration << " seconds.");
					// Update the class variable
					m_flying_state = STATE_TAKE_OFF;
				}
				else
				{
					// Inform the user
					ROS_INFO("[FLYING AGENT CLIENT] Failed to get take-off duration from Default controller. Switching to MOTORS-OFF.");
					// Update the class variable
					m_flying_state = STATE_MOTORS_OFF;
				}
			}
			// Otherwise, just switch straight to the
			// "flying" state
			else
			{
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Changed state directly to STATE_FLYING");
				// Update the class variable
				m_flying_state = STATE_FLYING;
			}
		}
		else
		{
			// Inform the user of the problem
			if (!m_isAvailable_mocap_data)
			{
				ROS_ERROR("[FLYING AGENT CLIENT] Take-off not possible because the motion capture data is unavailable.");
			}
			if (m_isOcculded_mocap_data)
			{
				ROS_ERROR("[FLYING AGENT CLIENT] Take-off not possible because the object is \"long-term\" occuled.");
			}
		}
	}
}


void requestChangeFlyingStateToLand()
{
	// Only allow landing from the TAKE-OFF and FLYING state
	if (
		(m_flying_state != STATE_TAKE_OFF)
		and
		(m_flying_state != STATE_FLYING)
	)
	{
		ROS_ERROR("[FLYING AGENT CLIENT] Request to LAND was not implemented because the current state is NOT the TAKE-OFF or FLYING state.");
	}
	else
	{
		// Check whether a "controller" take-off should
		// be performed, and that not already in the
		// "land" state
		if (
			(yaml_shouldPerfom_landing_with_defaultController)
			and
			(m_flying_state != STATE_LAND)
		)
		{
			// Call the service offered by the default
			// controller for how long a landing will take
			dfall_pkg::IntIntService requestManoeurveCall;
			requestManoeurveCall.request.data = DEFAULT_CONTROLLER_REQUEST_LANDING;
			if(m_defaultController_requestManoeuvre.call(requestManoeurveCall))
			{
				// Extract the duration
				float land_duration = float(requestManoeurveCall.response.data) / 1000.0;
				// Start the timer
				// > Stop any previous instance
				m_timer_land_complete.stop();
				// > Set the period again (second argument is reset)
				m_timer_land_complete.setPeriod( ros::Duration(take_off_duration), true);
				// > Start the timer
				m_timer_land_complete.start();
				// Inform the user
				ROS_INFO_STREAM("[FLYING AGENT CLIENT] Changed state to STATE_TAKE_OFF for a duration of " << land_duration << " seconds.");
				// Update the class variable
				m_flying_state = STATE_LAND;
			}
			else
			{
				// Inform the user
				ROS_INFO("[FLYING AGENT CLIENT] Failed to get take-off duration from Default controller. Switching to MOTORS-OFF.");
				// Update the class variable
				m_flying_state = STATE_MOTORS_OFF;
			}
		}
		// Otherwise, just switch straight to the
		// "motors off" state
		else
		{
			// Inform the user
			ROS_INFO("[FLYING AGENT CLIENT] Changed state directly to STATE_MOTORS_OFF");
			// Update the class variable
			m_flying_state = STATE_MOTORS_OFF;
		}
	}
}


void timerCallback_takeoff_complete(const ros::TimerEvent&)
{
    // Only change to flying if still in the take-off state
    if (m_flying_state == STATE_TAKE_OFF)
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Take-off complete, changed state to STATE_FLYING");
		// Update the class variable
		m_flying_state = STATE_FLYING;
	}
	else
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Take-off duration elapsed but the agent is no longer in STATE_TAKE_OFF.");
	}
}

void timerCallback_land_complete(const ros::TimerEvent&)
{
    // Only change to flying if still in the take-off state
    if (m_flying_state == STATE_LAND)
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Land complete, changed state to STATE_MOTORS_OFF");
		// Update the class variable
		m_flying_state = STATE_MOTORS_OFF;
	}
	else
	{
		// Inform the user
		ROS_INFO("[FLYING AGENT CLIENT] Land duration elapsed but the agent is no longer in STATE_LAND.");
	}
}














//    ----------------------------------------------------------------------------------
//     SSSS  EEEEE  L      EEEEE   CCCC  TTTTT
//    S      E      L      E      C        T
//     SSS   EEE    L      EEE    C        T
//        S  E      L      E      C        T
//    SSSS   EEEEE  LLLLL  EEEEE   CCCC    T
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//    ----------------------------------------------------------------------------------


// THIS SETS THE CONTROLLER THAT IT ACTUALLY BEING USED
// > During take-off and landing this function is
//   called to switch to the "Default" controller
// > The highlighting in the "Flying Agent GUI" should
//   reflect the instant controller   
void setInstantController(int controller)
{
    // Update the class variable
    m_instant_controller = controller;

    // Point to the correct service client
    switch(controller)
    {
        case DEFAULT_CONTROLLER:
            m_instant_controller_service_client = &m_defaultController;
            break;
        case STUDENT_CONTROLLER:
            m_instant_controller_service_client = &m_studentController;
            break;
        case TUNING_CONTROLLER:
            m_instant_controller_service_client = &m_tuningController;
            break;
        case PICKER_CONTROLLER:
            m_instant_controller_service_client = &m_pickerController;
            break;
        case TEMPLATE_CONTROLLER:
            m_instant_controller_service_client = &m_templateController;
            break;
        default:
            break;
    }

    // Publish a message that informs the "Flying Agent
    // GUI" about this update to the instant controller
    std_msgs::Int32 controller_used_msg;
    controller_used_msg.data = controller;
    controllerUsedPublisher.publish(controller_used_msg);
}


// THIS SIMPLY RETRIEVES THE CLASS VARIABLE
int getInstantController()
{
    return m_instant_controller;
}


// THIS SETS THE CONTROLLER THAT IS NOMINALLY SELECTED
// > This is the controller that will be use as the
//   "instant controller" when in the "flying" state.
// > But during take-off and landing, the "Default"
//   controller is used, and this keeps track of which
//   controller to switch to after those phases are
//   complete
void setControllerNominallySelected(int controller)
{
    // Update the class variable
    m_controller_nominally_selected = controller;

    // If in state "MOTORS-OFF" or "FLYING",
    // then the change is instant.
    if (
    	(m_flying_state == STATE_MOTORS_OFF)
    	or
    	(m_flying_state == STATE_FLYING)
	)
    {

        setInstantController(controller); 
    }
}


// THIS SIMPLY RETRIEVES THE CLASS VARIABLE
int getControllerNominallySelected()
{
    return m_controller_nominally_selected;
}










//    ----------------------------------------------------------------------------------
//     SSSS  TTTTT    A    TTTTT  EEEEE
//    S        T     A A     T    E
//     SSS     T    A   A    T    EEE
//        S    T    AAAAA    T    E
//    SSSS     T    A   A    T    EEEEE
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//
//    FFFFF  RRRR    OOO   M   M      GGGG  U   U  III
//    F      R   R  O   O  MM MM     G      U   U   I
//    FFF    RRRR   O   O  M M M     G      U   U   I
//    F      R   R  O   O  M   M     G   G  U   U   I
//    F      R   R   OOO   M   M      GGGG   UUU   III
//    ----------------------------------------------------------------------------------



// THE CALLBACK THAT A NEW FLYING STATE WAS REQUESTED
// > These requests come from the "Flying Agent GUI"
// > The options are: {take-off,land,motor-off,controller}
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
                requestChangeFlyingStateTo(STATE_TAKE_OFF);
                break;

            case CMD_CRAZYFLY_LAND:
                ROS_INFO("[FLYING AGENT CLIENT] LAND Command received");
                requestChangeFlyingStateTo(STATE_LAND);
                break;
            case CMD_CRAZYFLY_MOTORS_OFF:
                ROS_INFO("[FLYING AGENT CLIENT] MOTORS_OFF Command received");
                requestChangeFlyingStateTo(STATE_MOTORS_OFF);
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
    m_crazyradio_status = msg.data;
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
            requestChangeFlyingStateTo(STATE_LAND);
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
//     CCCC  RRRR   EEEEE    A    TTTTT  EEEEE
//    C      R   R  E       A A     T    E
//    C      RRRR   EEE    A   A    T    EEE
//    C      R   R  E      AAAAA    T    E
//     CCCC  R   R  EEEEE  A   A    T    EEEEE
//
//     SSSS  EEEEE  RRRR   V   V  III   CCCC  EEEEE
//    S      E      R   R  V   V   I   C      E
//     SSS   EEE    RRRR   V   V   I   C      EEE
//        S  E      R   R   V V    I   C      E
//    SSSS   EEEEE  R   R    V    III   CCCC  EEEEE
//
//     CCCC  L      III  EEEEE  N   N  TTTTT
//    C      L       I   E      NN  N    T
//    C      L       I   EEE    N N N    T
//    C      L       I   E      N  NN    T
//     CCCC  LLLLL  III  EEEEE  N   N    T
//    ----------------------------------------------------------------------------------


// CREATE A "CONTROLLER" TYPE SERVICE CLIENT
// NOTE: that in the "ros::service::createClient" function the
//       second argument is a boolean that specifies whether the 
//       service is persistent or not. In the ROS documentation a
//       persistent connection is described as:
//   "Persistent connections should be used carefully. They greatly
//    improve performance for repeated requests, but they also make
//    your client more fragile to service failures. Clients using
//    persistent connections should implement their own reconnection
//    logic in the event that the persistent connection fails."
void createControllerServiceClientFromParameterName( std::string paramter_name , ros::ServiceClient& serviceClient )
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



void timerCallback_for_createAllcontrollerServiceClients(const ros::TimerEvent&)
{
    // INITIALISE ALL THE CONTROLLER SERVICE CLIENTS
    createControllerServiceClientFromParameterName( "defaultController"  , m_defaultController );
    createControllerServiceClientFromParameterName( "studentController"  , m_studentController );
    createControllerServiceClientFromParameterName( "tuningController"   , m_tuningController );
    createControllerServiceClientFromParameterName( "pickerController"   , m_pickerController );
    createControllerServiceClientFromParameterName( "templateController" , m_templateController );

    // INITIALISE THE SERVICE FOR REQUESTING THE DEFAULT
    // CONTROLLER TO PERFORM MANOEUVRES
    createControllerServiceClientFromParameterName( "defaultController_requestManoeuvre" , m_defaultController_requestManoeuvre );

    // Check that at least the default controller is available
    // > Setting the flag accordingly
    if (defaultController)
    {
        m_controllers_avialble = true;
    }
    else
    {
        m_controllers_avialble = false;
        // Inform the user of the problem
        ROS_ERROR("[FLYING AGENT CLIENT] The default controller service client (and pressumably all other controllers) could NOT be created.");
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

    // Number of consecutive occulsions that will deem the
    // object as "long-term" occuled
	consecutive_occulsions_threshold = getParameterInt(nodeHandle_for_paramaters,"consecutive_occulsions_threshold");

	// Flag for whether the take-off should be performed
	//  with the default controller
	yaml_shouldPerfom_takeOff_with_defaultController = getParameterBool(nodeHandle_for_paramaters,"shouldPerfom_takeOff_with_defaultController");

	// Flag for whether the landing should be performed
	// with the default controller
	yaml_shouldPerfom_landing_with_defaultController = getParameterBool(nodeHandle_for_paramaters,"shouldPerfom_landing_with_defaultController");
    
    // DEBUGGING: Print out one of the parameters that was loaded
    ROS_INFO_STREAM("[FLYING AGENT CLIENT] DEBUGGING: the fetched ClientConfig/isEnabled_strictSafety = " << yaml_isEnabled_strictSafety);



    // PROCESS THE PARAMTERS
    // Convert from degrees to radians
    m_maxTiltAngle_for_strictSafety_redians = DEG2RAD * yaml_maxTiltAngle_for_strictSafety_degrees;

    // DEBUGGING: Print out one of the processed values
    ROS_INFO_STREAM("[FLYING AGENT CLIENT CONTROLLER] DEBUGGING: after processing m_maxTiltAngle_for_strictSafety_redians = " << m_maxTiltAngle_for_strictSafety_redians);
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
    // > This cannot be done directly here because the other nodes may
    //   be currently unavailable. Hence, we start a timer for a few
    //   seconds and in the call back all the controller service
    //   clients are created.
    m_controllers_avialble = false;
    m_timer_for_createAllControllerServiceClients = nodeHandle.createTimer(ros::Duration(3), timerCallback_for_createAllcontrollerServiceClients, true);





    // INITIALISE THE MOTION CAPTURE TIME-OUT TIMER
    // > And stop it immediately
    m_isAvailable_mocap_data = false;
    m_timer_mocap_timeout_check = nodeHandle.createTimer(ros::Duration(yaml_mocap_timeout_duration), timerCallback_mocap_timeout_check, true);
    m_timer_mocap_timeout_check.stop();
    





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
	m_commaNdfOrsendiNgTocrazyflIepublisher = nodeHandle.advertise <ControlCommand>("ControlCommand", 1);

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
    m_crazyradio_status = CRAZY_RADIO_STATE_DISCONNECTED;

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
