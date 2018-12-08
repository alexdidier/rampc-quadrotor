//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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
//    The service that manages the loading of YAML parameters
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/BatteryMonitor.h"











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



void agentOperatingStateCallback(const std_msgs::Int32& msg)
{
	// Extract the data
	m_agent_operating_state = msg.data;
}





void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
	// Extract the data
	int crazyradio_status = msg.data;

	// I the Crazyradio DISCONNECTED
	if (crazyradio_status == CRAZY_RADIO_STATE_DISCONNECTED)
	{
		// Then publish that the battery level is unavailable
		std_msgs::Int32 battery_level_msg;
		battery_level_msg.data = BATTERY_LEVEL_UNAVAILABLE;
		batteryLevelPublisher.publish(battery_level_msg);
	}
}





void newBatteryVoltageCallback(const std_msgs::Float32& msg)
{
	// Extract the data
	float battery_voltage = msg.data;

	// Provide the new measurement to the filter
	float filtered_battery_voltage = newBatteryVoltageForFilter(battery_voltage);

	// Convert the filtered voltage to a percentage
	// > Note that this depending on the operating state
	float filtered_battery_voltage_as_percentage = fromVoltageToPercent(filtered_battery_voltage,m_agent_operating_state);

	// Convert the battery percentage to a level
	int filtered_battery_voltage_as_level = convertPercentageToLevel(filtered_battery_voltage_as_percentage);

	// Publish that the battery voltage
	std_msgs::Float32 filtered_battery_voltage_msg;
	filtered_battery_voltage_msg.data = filtered_battery_voltage;
	filteredBatteryVoltagePublisher.publish(filtered_battery_voltage_msg);

	// Publish that the battery level
	std_msgs::Int32 battery_level_msg;
	battery_level_msg.data = filtered_battery_voltage_as_level;
	batteryLevelPublisher.publish(battery_level_msg);

	// Update the battery state using the level
	// > Note that the function called sends a message
	//   only if the battery state changes
	updateBatteryStateBasedOnLevel(filtered_battery_voltage_as_level);
}




// > For filtering the battery voltage
float newBatteryVoltageForFilter(float new_value)
{
	return 0.0f;
}





// > For converting a voltage to a percentage, depending on the current "my_flying_state" value
float fromVoltageToPercent(float voltage , float operating_state )
{
	// INITIALISE THE LOCAL VARIABLE FOR THE VOLTAGE WHEN FULL/EMPTY
	float voltage_when_full;
	float voltage_when_empty;

	// COMPUTE THE PERCENTAGE DIFFERENTLY DEPENDING ON
	// THE CURRENT FLYING STATE
	if (operating_state == AGENT_OPERATING_STATE_MOTORS_OFF)
	{
		// Voltage limits for a "standby" type of state
		voltage_when_empty = yaml_battery_voltage_threshold_lower_while_standby;
		voltage_when_full  = yaml_battery_voltage_threshold_upper_while_standby;
	}
	else
	{
		// Voltage limits for a "flying" type of state
		voltage_when_empty = yaml_battery_voltage_threshold_lower_while_flying;
		voltage_when_full  = yaml_battery_voltage_threshold_upper_while_flying;
	}

	// COMPUTE THE PERCENTAGE
	float percentage = 100.0f * (voltage-voltage_when_empty)/(voltage_when_full-voltage_when_empty);

	// CLIP THE PERCENTAGE TO BE BETWEEN [0,100]
	// > This should not happen to often
	if(percentage > 100.0f)
	{
		percentage = 100.0f;
	}
	if(percentage < 0.0f)
	{
		percentage = 0.0f;
	}

	// RETURN THE PERCENTAGE
	return percentage;
}




// > For converting the percentage to a level
int convertPercentageToLevel(float percentage)
{
	// Initialise the battery level
	static int battery_level = BATTERY_LEVEL_UNAVAILABLE;

	// Iterate through the levels
	for (int i_level=0 ; i_level<10 ; i_level++)
	{
		// Compute the threshold for this level
		float threshold = float(i_level) * 10.0f;
		// Add a buffer to the threshold to prevent
		// high frequency changes to the level
		if (battery_level==i_level)
		{
			threshold += 2.0f;
		}
		// Return the current index if appropriate
		if (percentage <= threshold)
		{
			return i_level;
		}
	}
	// If the function made it to this point without
	// returning, then the percentage is at or above
	// the maximum level
	return BATTERY_LEVEL_100;
}




// > For updating the battery state based on the battery level
//   Possible states are {normal,low}, and changes are delayed
void updateBatteryStateBasedOnLevel(int level)
{
	// Initialise the battery state
	static int battery_state = BATTERY_STATE_NORMAL;

	// Initialise a counter for delaying a change of state
	static int num_since_change = 0;

	if (level == 0)
	{
		if (battery_state == BATTERY_STATE_NORMAL)
		{
			num_since_change++;
		}
		else
		{
			num_since_change = 0;
		}
	}
	else if (level >= 1)
	{
		if (battery_state == BATTERY_STATE_LOW)
		{
			num_since_change++;
		}
		else
		{
			num_since_change = 0;
		}
	}

	// Check if the "delay-to-change" threshold is reached
	if (num_since_change >= yaml_battery_delay_threshold_to_change_state)
	{
		if (battery_state == BATTERY_STATE_NORMAL)
		{
			battery_state = BATTERY_STATE_LOW;
		}
		else
		{
			battery_state = BATTERY_STATE_NORMAL;
		}
		// Publish the change
		std_msgs::Int32 battery_state_msg;
		battery_state_msg.data = battery_state;
		batteryStateChangedPublisher.publish(battery_state_msg);
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


// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void yamlReadyForFetchCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForID , msg.agentIDs );

	// Continue if the message is relevant
	if  (isRevelant)
	{
		// Extract the data
		int parameter_service_to_load_from = msg.data;
		// Load from the respective parameter service
		// Switch between fetching for the different controllers and from different locations
		switch(parameter_service_to_load_from)
		{

			// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
			case LOAD_YAML_FROM_AGENT:
			{
				// Let the user know that this message was received
				ROS_INFO("[BATTERY MONITOR] Now fetching the YAML parameter values from this agent.");
				// Create a node handle to the parameter service of this agent
				ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
				// Call the function that fetches the parameters
				fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);
				break;
			}

			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				// Let the user know that this message was received
				ROS_INFO("[BATTERY MONITOR] Now fetching the YAML parameter values from this agent's coordinator.");
				// Create a node handle to the parameter service of this agent's coordinator
				ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);
				// Call the function that fetches the parameters
				fetchYamlParameters(nodeHandle_to_coordinator_parameter_service);
				break;
			}

			default:
			{
				// Let the user know that the command was not relevant
				//ROS_INFO("The StudentControllerService received the message that YAML parameters were (re-)loaded");
				//ROS_INFO("> However the parameters do not relate to this controller, hence nothing will be fetched.");
				break;
			}
		}
	}
}



// Check the header of a message for whether it is relevant
bool checkMessageHeader( int agentID , bool shouldCheckForID , const std::vector<uint> & agentIDs )
{
	// The messag is by default relevant if the "shouldCheckForID"
	// flag is false
	if (!shouldCheckForID)
	{
		return true;
	}
	else
	{
		// Iterate through the vector of IDs
		for ( int i_ID=0 ; i_ID < agentIDs.size() ; i_ID++ )
		{
			if ( agentIDs[i_ID] == agentID )
			{
				return true;
			}
		}
	}
	// If the function made it to here, then the message is
	// NOT relevant, hence return false
	return false;
}



// This function CAN BE edited for successful completion of the PPS exercise, and the
// use of parameters fetched from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void fetchYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the StudentController.yaml file

	// Add the "StudentController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "BatteryMonitor");



	// Frequency of requesting the battery voltage, in [milliseconds]
	//yaml_battery_polling_period = getParameterFloat(nodeHandle_for_paramaters,"battery_polling_period");

	// Battery thresholds while in the "motors off" state, in [Volts]
	yaml_battery_voltage_threshold_lower_while_standby = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_lower_while_standby");
	yaml_battery_voltage_threshold_upper_while_standby = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_upper_while_standby");

	// Battery thresholds while in the "flying" state, in [Volts]
	yaml_battery_voltage_threshold_lower_while_flying = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_lower_while_flying");
	yaml_battery_voltage_threshold_upper_while_flying = getParameterFloat(nodeHandle_for_paramaters,"battery_voltage_threshold_upper_while_flying");

	// Delay before changing the state of the battery, in [number of measurements]
	// > Note that the delay in seconds therefore depends on the polling period
	yaml_battery_delay_threshold_to_change_state = getParameterInt(nodeHandle_for_paramaters,"battery_delay_threshold_to_change_state");



	// DEBUGGING: Print out one of the parameters that was loaded
	ROS_INFO_STREAM("[BATTERY MONITOR] DEBUGGING: the fetched BatteryMonitor/battery_polling_period = " << yaml_battery_voltage_threshold_lower_while_flying);

	// Call the function that computes details an values that are needed from these
	// parameters loaded above
	processFetchedParameters();
}





// This function CAN BE edited for successful completion of the PPS exercise, and the
// use of parameters loaded from the YAML file is highly recommended to make tuning of
// your controller easier and quicker.
void processFetchedParameters()
{
    // Compute the feed-forward force that we need to counteract gravity (i.e., mg)
    // > in units of [Newtons]
    //cf_weight_in_newtons = cf_mass_in_grams * 9.81/1000.0;
    
    // DEBUGGING: Print out one of the computed quantities
	//ROS_INFO_STREAM("[STUDENT CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << cf_weight_in_newtons);
}





//    ----------------------------------------------------------------------------------
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------


// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}




bool getAgentIDandCoordIDfromClientNode( std::string client_namespace , int * agentID_ref , int * coordID_ref)
{
	// Initialise the return variable as success
	bool return_was_successful = true;

	// Create a node handle to the client
	ros::NodeHandle client_nodeHandle(client_namespace);

	// Get the value of the "agentID" parameter
	int agentID_fetched;
	if(!client_nodeHandle.getParam("agentID", agentID_fetched))
	{
		return_was_successful = false;
	}
	else
	{
		*agentID_ref = agentID_fetched;
	}

	// Get the value of the "coordID" parameter
	int coordID_fetched;
	if(!client_nodeHandle.getParam("coordID", coordID_fetched))
	{
		return_was_successful = false;
	}
	else
	{
		*coordID_ref = coordID_fetched;
	}

	// Return
	return return_was_successful;
}



void getConstructNamespaceForCoordinatorParameterService( int coordID, std::string & coord_param_service_namespace )
{
	// Set the class variable "nodeHandle_to_coordinator_parameter_service" to be a node handle
	// for the parameter service that is running on the coordinate machine
	// NOTE: the backslash here (i.e., "/") before the name of the node ("ParameterService")
	//       is very important because it specifies that the name is global
	// Convert the agent ID to a zero padded string
	std::ostringstream str_stream;
	str_stream << std::setw(3) << std::setfill('0') << coordID;
	std::string coordID_as_string(str_stream.str());
	coord_param_service_namespace = "/dfall/coord" + coordID_as_string + "/ParameterService";
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
	ros::init(argc, argv, "BatteryMonitor");

	// Create a "ros::NodeHandle" type local variable named "nodeHandle",
	// the "~" indcates that "self" is the node handle assigned.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[BATTERY MONITOR] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/PPSClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[BATTERY SERVICE] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[BATTERY MONITOR] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}



	// PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

	// Set the class variable "m_namespace_to_own_agent_parameter_service",
	// i.e., the namespace of parameter service for this agent
	m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	getConstructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[BATTERY MONITOR] The namespace string for accessing the Paramter Services are:");
	ROS_INFO_STREAM("[BATTERY MONITOR] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[BATTERY MONITOR] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameters service publish messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber batteryMonitor_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "BatteryMonitor", 1, yamlReadyForFetchCallback);
	ros::Subscriber batteryMonitor_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("BatteryMonitor", 1, yamlReadyForFetchCallback);


	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// Call the class function that loads the parameters for this class.
	fetchYamlParameters(nodeHandle_to_own_agent_parameter_service);



	// PUBLISHERS

	// Publisher for the filtered battery voltage
	ros::Publisher filteredBatteryVoltagePublisher = nodeHandle.advertise<std_msgs::Float32>("FilteredVoltage",1);

	// Publisher for the battery level
	batteryLevelPublisher = nodeHandle.advertise<std_msgs::Int32>("Level",1);

	// Publisher for changes in the battery state
	ros::Publisher batteryStateChangedPublisher = nodeHandle.advertise<std_msgs::Int32>("ChangedStateTo",1);



	// SUBSCRIBERS

	// Subscribe to the voltage of the battery
	ros::Subscriber newBatteryVoltageSubscriber = nodeHandle.subscribe("CrazyRadio/CFBattery", 1, newBatteryVoltageCallback);

	// Subscribe to the status of the Crazyradio: connected, connecting or disconnected
	ros::Subscriber crazyRadioStatusSubscriber = nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, crazyRadioStatusCallback);

	// Subscribe to the flying state of the agent
	ros::Subscriber agentOperatingStateSubscriber = nodeHandle.subscribe("PPS_Client/flyingState", 1, agentOperatingStateCallback);

	// Initialise the operating state
	m_agent_operating_state = AGENT_OPERATING_STATE_MOTORS_OFF;



	// Inform the user the this node is ready
	ROS_INFO("[BATTERY MONITOR] Ready :-)");
	// Spin as a single-thread node
	ros::spin();

	return 0;
}
