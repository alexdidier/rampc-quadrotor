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
//    The service that manages the loading of YAML parameters
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/ParameterService.h"





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



bool requestLoadYamlFilenameCallbackServiceCallback(LoadYamlFromFilename::Request &request, LoadYamlFromFilename::Response &response)
{
    // Put the flying state into the response variable
    requestLoadYamlFilenamePublisher.publish(request.stringWithHeader);

    // Put success into the response
    response.data = 1;

    // Return
    return true;
}

void requestLoadYamlFilenameCallback(const StringWithHeader& yaml_filename_to_load_with_header)
{
    // LOAD THE YAML FILE

    // Get the yaml file name requested
    std::string yaml_filename_to_load = yaml_filename_to_load_with_header.data;
    // Get the node handle to this parameter service
    ros::NodeHandle nodeHandle("~");
    // Instantiate a local variable for the command string that will be passed to the "system":
    std::string cmd;
    // Get the abolute path to "dfall_pkg":
    std::string dfall_pkg_path = ros::package::getPath("dfall_pkg");
    // Construct the system command string for (re-)loading the parameters:
    cmd = "rosparam load " + dfall_pkg_path + "/param" + "/" + yaml_filename_to_load + ".yaml " + m_base_namespace + "/" + yaml_filename_to_load;
    // Let the user know what is about to happen
    ROS_INFO_STREAM("[PARAMETER SERVICE] The following path will be used for locating the .yaml file: " << dfall_pkg_path  << " The comand line string sent to the 'system' is: " << cmd );
    // Send the "load yaml" command to the system
    system(cmd.c_str());



    // PUBLISH A MESSAGE THAT THE YAML FILE WAS LOADED

    // Create publisher as a local variable, using the filename
    // as the name of the message
    ros::Publisher yamlParametersReadyForFetchPublisher = nodeHandle.advertise<IntWithHeader>(yaml_filename_to_load, 1, true);
    //ros::spinOnce();
    // Create a local variable for the message
    IntWithHeader yaml_ready_msg;
    // Specify with the data the "type" of this parameter service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            yaml_ready_msg.data = LOAD_YAML_FROM_AGENT;
            break;
        }
        case TYPE_COORDINATOR:
        {
            yaml_ready_msg.data = LOAD_YAML_FROM_COORDINATOR;
            break;
        }
        default:
        {
            // Throw an error if the type is not recognised
            ROS_ERROR("[PARAMETER SERVICE] The 'm_type' variable was not recognised.");
            // Specify to load from the agent by default
            yaml_ready_msg.data = LOAD_YAML_FROM_AGENT;
            break;
        }
    }
    // Copy across the boolean field
    yaml_ready_msg.shouldCheckForAgentID = yaml_filename_to_load_with_header.shouldCheckForAgentID;
    // Copy across the vector of IDs
    if (yaml_filename_to_load_with_header.shouldCheckForAgentID)
    {
        for ( int i_ID=0 ; i_ID<yaml_filename_to_load_with_header.agentIDs.size() ; i_ID++ )
        {
            yaml_ready_msg.agentIDs.push_back(yaml_filename_to_load_with_header.agentIDs[i_ID]);
        }
    }
    // Sleep to make the publisher known to ROS (in seconds)
    ros::Duration(1.0).sleep();
    // Send the message
    yamlParametersReadyForFetchPublisher.publish(yaml_ready_msg);

    // Inform the user that this was published
    ROS_INFO_STREAM("[PARAMETER SERVICE] Published message that " << yaml_filename_to_load << " yaml parameters are ready." );
}





bool getTypeAndIDParameters()
{
    // Initialise the return variable as success
    bool return_was_successful = true;

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the value of the "type" parameter into a local string variable
    std::string type_string;
    if(!nodeHandle.getParam("type", type_string))
    {
        // Throw an error if the agent ID parameter could not be obtained
        ROS_ERROR("[PARAMETER SERVICE] Failed to get type");
    }

    // Set the "m_type" class variable based on this string loaded
    if ((!type_string.compare("coordinator")))
    {
        m_type = TYPE_COORDINATOR;
    }
    else if ((!type_string.compare("agent")))
    {
        m_type = TYPE_AGENT;
    }
    else
    {
        // Set "m_type" to the value indicating that it is invlid
        m_type = TYPE_INVALID;
        return_was_successful = false;
        ROS_ERROR("[PARAMETER SERVICE] The 'type' parameter retrieved was not recognised.");
    }


    // Construct the string to the namespace of this Paramater Service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            // Get the value of the "agentID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("agentID", m_ID))
            {
                // Throw an error if the agent ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[PARAMETER SERVICE] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[PARAMETER SERVICE] Is of type AGENT with ID = " << m_ID);
            }
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            // Get the value of the "coordID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("coordID", m_ID))
            {
                // Throw an error if the coord ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[PARAMETER SERVICE] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[PARAMETER SERVICE] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[PARAMETER SERVICE] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}





bool constructNamespaces()
{
    // Initialise the return variable as success
    bool return_was_successful = true;

    // Get the namespace of this "ParameterService" node
    std::string this_node_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[PARAMETER SERVICE] ros::this_node::getNamespace() =  " << this_node_namespace);

    // Construct the string to the namespace of this Paramater Service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            //m_base_namespace = ros::this_node::getNamespace();
            //m_base_namespace = "/agent" + m_Id + '/' + "ParameterService";
            m_base_namespace = this_node_namespace + '/' + "ParameterService";
            ROS_INFO_STREAM("[PARAMETER SERVICE] .yaml file parameters will be loaded into the 'base' namespace: " << m_base_namespace);
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            //m_base_namespace = ros::this_node::getNamespace();
            //m_base_namespace = "/ParameterService";
            m_base_namespace = this_node_namespace + '/' + "ParameterService";
            ROS_INFO_STREAM("[PARAMETER SERVICE] .yaml file parameters will be loaded into the 'base' namespace: " << m_base_namespace);
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[PARAMETER SERVICE] The 'm_type' type parameter was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
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
    ros::init(argc, argv, "ParameterService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the type and ID of this parameter service
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Construct the namespace into which this parameter service
    // loads yaml parameters
    bool isValid_namespaces = constructNamespaces();

    // Stall if the TYPE and ID are not valid
    if ( !( isValid_type_and_ID && isValid_namespaces ) )
    {
        ROS_ERROR("[PARAMETER SERVICE] Service NOT FUNCTIONING :-)");
        ros::spin();
    }

    // Subscribe to the messages that request loading a yaml file
    ros::Subscriber requestLoadYamlFilenameSubscriber = nodeHandle.subscribe("requestLoadYamlFilename", 1, requestLoadYamlFilenameCallback);

    // Publisher for publishing "internally" to the subscriber above
    requestLoadYamlFilenamePublisher = nodeHandle.advertise <StringWithHeader>("requestLoadYamlFilename", 1);

    // Advertise the service for requests to load a yaml file
    ros::ServiceServer getCurrentFlyingStateService = nodeHandle.advertiseService("requestLoadYamlFilename", requestLoadYamlFilenameCallbackServiceCallback);

    // Inform the user the this node is ready
    ROS_INFO("[PARAMETER SERVICE] Service ready :-)");
    // Spin as a single-thread node
    ros::spin();

    return 0;
}
