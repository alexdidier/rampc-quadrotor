//    Copyright (C) 2017, ETH Zurich, D-ITET, Cyrill Burgener, Marco Mueller, Philipp Friedli
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





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

#include <stdlib.h>
#include <ros/ros.h>

#include "ParameterService.h"



//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------


// For which controller parameters to load
#define LOAD_YAML_SAFE_CONTROLLER_AGENT           1
#define LOAD_YAML_CUSTOM_CONTROLLER_AGENT         2
#define LOAD_YAML_SAFE_CONTROLLER_COORDINATOR     3
#define LOAD_YAML_CUSTOM_CONTROLLER_COORDINATOR   4

#define FETCH_YAML_SAFE_CONTROLLER_AGENT          1
#define FETCH_YAML_CUSTOM_CONTROLLER_AGENT        2
#define FETCH_YAML_SAFE_CONTROLLER_COORDINATOR    3
#define FETCH_YAML_CUSTOM_CONTROLLER_COORDINATOR  4

#define TYPE_COORDINATOR  0
#define TYPE_AGENT        1


using namespace d_fall_pps;





//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// The type of this node, i.e., agent or a coordinator, specified as a parameter in the
// "*.launch" file
int my_type = 0;

// The ID of this agent, i.e., the ID of this computer in the case that this computer is
// and agent
int my_agentID = 0;

// Publisher that notifies the relevant nodes when the YAML paramters have been loaded
// from file into ram/cache, and hence are ready to be fetched
ros::Publisher controllerYamlReadyForFetchPublihser;


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

void requestLoadControllerYamlCallback(const std_msgs::Int32& msg)



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




void requestLoadControllerYamlCallback(const std_msgs::Int32& msg)
{
    // Extract from the "msg" for which controller the YAML
    // parameters should be loaded
    int controller_to_load_yaml = msg.data;


    // Instantiate a local varaible to confirm that something was actually loaded from
    // a YAML file
    bool isReadyForFetch = true;

    // Switch between loading for the different controllers
    switch(controller_to_load_yaml)
    {
        case ( LOAD_YAML_SAFE_CONTROLLER_COORDINATOR && (my_type == TYPE_COORDINATOR) ):
            // Re-load the parameters of the safe controller:
            cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + ros_namespace + "/SafeControllerService";
            system(cmd.c_str());
            ROS_INFO_STREAM(cmd);
            break;

        case ( LOAD_YAML_SAFE_CONTROLLER_AGENT && (my_type == TYPE_AGENT) ):
            // Re-load the parameters of the safe controller:
            cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + ros_namespace + "/SafeControllerService";
            system(cmd.c_str());
            ROS_INFO_STREAM(cmd);
            break;

        case ( LOAD_YAML_CUSTOM_CONTROLLER_COORDINATOR && (my_type == TYPE_COORDINATOR) ):
            // Re-load the parameters of the custom controller:
            cmd = "rosparam load " + d_fall_pps_path + "/param/CustomController.yaml " + ros_namespace + "/CustomControllerService";
            system(cmd.c_str());
            ROS_INFO_STREAM(cmd);
            break;

        case ( LOAD_YAML_CUSTOM_CONTROLLER_AGENT && (my_type == TYPE_AGENT) ):
            // Re-load the parameters of the custom controller:
            cmd = "rosparam load " + d_fall_pps_path + "/param/CustomController.yaml " + ros_namespace + "/CustomControllerService";
            system(cmd.c_str());
            ROS_INFO_STREAM(cmd);
            break;

        default:
            // Let the user know that the command was not recognised
            ROS_INFO("Unknown 'controller to load yaml' command, thus nothing will be loaded");
            // Set the boolean that prevents the fetch message from being sent
            isReadyForFetch = false
            break;
    }

    // Pause breifly to ensure that the yaml file is fully loaded
    ros::Duration(0.2).sleep();

    // Only bother with sending the "ready for fetch" message if something was actually
    // loaded from a YAML file
    if (isReadyForFetch)
    {
        // Instantiate a local variable for the fetch message
        std_msgs::Int32 fetch_msg;
        // Fill in the data of the fetch message
        switch(controller_to_load_yaml)
        {
            case (LOAD_YAML_SAFE_CONTROLLER_COORDINATOR):
                fetch_msg.data = FETCH_YAML_SAFE_CONTROLLER_FROM_COORDINATOR;
                break;
            case (LOAD_YAML_CUSTOM_CONTROLLER_COORDINATOR):
                fetch_msg.data = FETCH_YAML_CUSTOM_CONTROLLER_FROM_COORDINATOR;
                break;
            case (LOAD_YAML_SAFE_CONTROLLER_AGENT):
                fetch_msg.data = FETCH_YAML_SAFE_CONTROLLER_FROM_OWN_AGENT;
                break;
            case (LOAD_YAML_CUSTOM_CONTROLLER_AGENT):
                fetch_msg.data = FETCH_YAML_CUSTOM_CONTROLLER_FROM_OWN_AGENT;
                break;
            default:
                // Let the user know that the command was not recognised
                ROS_INFO("Unknown 'controller to load yaml' command, thus nothing will be loaded");
                // Set the boolean that prevents the fetch message from being sent
                isReadyForFetch = false;
                break;
        }
        // Send a message that the YAML parameter have been loaded and hence are
        // ready to be fetched (i.e., using getparam())
        controllerYamlReadyForFetchPublihser.publish(fetch_msg);
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
    ros::init(argc, argv, "ParameterService");

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the value of the "type" parameter into a local string variable
    std::string type_string;
    if(!nodeHandle.getParam("type", type_string))
    {
        // Throw an error if the agent ID parameter could not be obtained
        ROS_ERROR("Failed to get type from ParameterService");
    }

    // Set the "my_type" instance variable based on this string loaded
    switch type_string
    {
        case 'coordinator':
            my_type = TYPE_COORDINATOR

        case 'agent':
            my_type = TYPE_AGENT

        default:
            ROS_ERROR("The retrieve type parameter was no recognised.");
    }


    // Get the value of the "agentID" parameter into the instance variable "my_agentID"
    if(!nodeHandle.getParam("agentID", my_agentID))
    {
        // Throw an error if the agent ID parameter could not be obtained
        ROS_ERROR("Failed to get agentID from ParameterService");
    }

    // Publisher that notifies the relevant nodes when the YAML paramters have been loaded
    // from file into ram/cache, and hence are ready to be fetched
    controllerYamlReadyForFetchPublihser = nodeHandle.advertise<std_msgs::Int32>("controllerYamlReadyForFetch", 1);
    
    // Get the handle to the namespace in which this coordinator is launched
    ros::NodeHandle namespaceNodeHandle = ros::NodeHandle();




    // SUBSCRIBE TO THE APPROPRIATE "request" MESSAGES DEPENDING ON THE "my_type"
    switch type_string
    {
        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case 'coordinator':
            // Get the node handles required
            ros::NodeHandle coordinator_agent_namespace_nodeHandle(ros::this_node::getNamespace());
            // > Subscribe to requests from: the master GUI
            ros::Subscriber requestLoadControllerYamlSubscriber_coordinator = coordinator_agent_namespace_nodeHandle.subscribe("my_GUI/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);


        // AN AGENT TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        // > The agent's own "PPSClient" node
        case 'agent':
            // Get the node handles required
            ros::NodeHandle agent_nodeHandle = ros::NodeHandle();
            ros::NodeHandle agent_namespace_nodeHandle(ros::this_node::getNamespace());
            // > Subscribe to requests from: the master GUI
            ros::Subscriber requestLoadControllerYamlSubscriber_agent_to_master = agent_nodeHandle.subscribe("/my_GUI/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);
            // > Subscribe to requests from: the agent's own "PPSClient" node
            ros::Subscriber requestLoadControllerYamlSubscriber_agent_to_self = agent_namespace_nodeHandle.subscribe("PPSClient/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);

        default:
            ROS_ERROR("The retrieve type parameter was no recognised.");
    }

    
    // Subscriber for requests that the controller parameters should be re-loaded from
    // the .YAML files on the coordinators machine, and then all the agents should be
    // request to fetch the parameters from itself, i.e., fetch parameters from the
    // coordinator.
    

    // SUBSCRIBE TO THE "request" MESSAGES FROM THE "master GUI"


    ROS_INFO("CentralManagerService ready");
    ros::spin();

    return 0;
}
