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





// SPECIFY THE PACKAGE NAMESPACE
using namespace d_fall_pps;
//using namespace std;





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




void crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
	// Extract the data
	int crazyradio_status = msg.data;

	// I the Crazyradio DISCONNECTED
	if (crazyradio_status == DISCONNECTED)
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
	m_battery_voltage = msg.data;

	// Provide the new measurement to the filter
	float filtered_battery_voltage = newBatteryVoltageForFilter(m_battery_voltage); //need to perform filtering here

	// Convert the filtered voltage to a percentage
	// > Note that this depending on the operating state
	float filtered_battery_voltage_as_percentage = convertVoltageToLevel(filtered_battery_voltage,m_agent_operating_state);

	// Convert the battery percentage to a level
	int filtered_battery_voltage_as_level = convertPercentageToLevel(filtered_battery_voltage_as_percentage);

	// Then publish that the battery level is unavailable
	std_msgs::Int32 battery_level_msg;
	battery_level_msg.data = BATTERY_LEVEL_UNAVAILABLE;
	batteryLevelPublisher.publish(battery_level_msg);

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
		ROS_INFO("[PPS CLIENT] low level battery triggered");
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



void agentOperatingStateCallback(const std_msgs::Int32& msg)
{
	// Extract the data
	m_agent_operating_state = msg.data;
}


int convertPercentageToLevel(float percentage)
{
	// Iterate through the levels
	for (i_level=0;i_level<10;i++)
	{
		// Compute the threshold for this level
		float threshold = float(i_level) * 10.0f;
		// Add a buffer to the threshold to prevent
		// high frequency changes to the level
		if (m_battery_level==i_level)
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


// > For converting a voltage to a percentage, depending on the current "my_flying_state" value
float fromVoltageToPercent(float voltage , float operating_state )
{
	// INITIALISE THE LOCAL VARIABLE FOR THE VOLTAGE WHEN FULL/EMPTY
	float voltage_when_full;
	float voltage_when_empty;

	// COMPUTE THE PERCENTAGE DIFFERENTLY DEPENDING ON
	// THE CURRENT FLYING STATE
	if (operating_state == STATE_MOTORS_OFF)
	{
		// Voltage limits for a "standby" type of state
		voltage_when_empty = m_battery_threshold_lower_while_standby;
		voltage_when_full  = m_battery_threshold_upper_while_standby;
	}
	else
	{
		// Voltage limits for a "flying" type of state
		voltage_when_empty = m_battery_threshold_lower_while_flying;
		voltage_when_full  = m_battery_threshold_upper_while_flying;
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

	// Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
	// the "~" indcates that "self" is the node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "ParameterService" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[BATTERY MONITOR] ros::this_node::getNamespace() =  " << m_namespace);





	// Advertise the service for loading Yaml Files
	ros::ServiceServer service = nodeHandle.advertiseService("LoadYamlFiles", loadYamlFiles);

	// Publish the battery level
	batteryLevelPublisher = nodeHandle.advertise<std_msgs::Int32>("BatteryLevel",1);

	// Subscribe to the voltage of the battery
	ros::Subscriber newBatteryVoltageSubscriber = nodeHandle.subscribe("CrazyRadio/CFBattery", 1, newBatteryVoltageCallback);

	// Subscribe to the status of the Crazyradio: connected, connecting or disconnected
	ros::Subscriber crazyRadioStatusSubscriber = nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, crazyRadioStatusCallback);

	// Subscribe to the flying state of the agent
	ros::Subscriber agentOperatingStateSubscriber = nodeHandle.subscribe("PPS_Client/flyingState", 1, agentOperatingStateCallback);

	// Initialise the battery level
	m_battery_level = BATTERY_LEVEL_UNAVAILABLE;

	// Initialise the operating state
	m_agent_operating_state = AGENT_OPERATING_STATE_MOTORS_OFF;


	ROS_INFO("[BATTERY MONITOR] Ready :-)");
	ros::spin();

	return 0;
}
