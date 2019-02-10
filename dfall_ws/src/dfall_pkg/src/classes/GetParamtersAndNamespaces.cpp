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
//    A class for having the standard functions in one place
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "classes/GetParamtersAndNamespaces.h"











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
//     GGGG  EEEEE  TTTTT  PPPP     A    RRRR     A    M   M   ( )
//    G      E        T    P   P   A A   R   R   A A   MM MM  (   )
//    G      EEE      T    PPPP   A   A  RRRR   A   A  M M M  (   )
//    G   G  E        T    P      AAAAA  R  R   AAAAA  M   M  (   )
//     GGGG  EEEEE    T    P      A   A  R   R  A   A  M   M   ( )
//    ----------------------------------------------------------------------------------



float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}



int getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}



bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name)
{
    bool val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}



void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}



void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}



int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val.size();
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





void constructNamespaceForCoordinator( int coordID, std::string & coord_namespace )
{
	// Convert the ID to a zero padded string
	std::ostringstream str_stream;
	str_stream << std::setw(3) << std::setfill('0') << coordID;
	std::string coordID_as_string(str_stream.str());
	// Construct the namespace
	coord_namespace = "/dfall/coord" + coordID_as_string;
}


void constructNamespaceForCoordinatorParameterService( int coordID, std::string & coord_param_service_namespace )
{
	// Convert the ID to a zero padded string
	std::ostringstream str_stream;
	str_stream << std::setw(3) << std::setfill('0') << coordID;
	std::string coordID_as_string(str_stream.str());
	// Construct the namespace
	coord_param_service_namespace = "/dfall/coord" + coordID_as_string + "/ParameterService";
}





// Check the header of a message for whether it is relevant
bool checkMessageHeader( int agentID , bool shouldCheckForAgentID , const std::vector<uint> & agentIDs )
{
	// The messag is by default relevant if the "shouldCheckForAgentID"
	// flag is false
	if (!shouldCheckForAgentID)
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