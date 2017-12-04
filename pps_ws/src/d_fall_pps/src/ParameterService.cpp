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
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

#include "d_fall_pps/Controller.h"


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

#define FETCH_YAML_SAFE_CONTROLLER_FROM_OWN_AGENT      1
#define FETCH_YAML_CUSTOM_CONTROLLER_FROM_OWN_AGENT    2
#define FETCH_YAML_SAFE_CONTROLLER_FROM_COORDINATOR    3
#define FETCH_YAML_CUSTOM_CONTROLLER_FROM_COORDINATOR  4

#define TYPE_INVALID      -1
#define TYPE_COORDINATOR   1
#define TYPE_AGENT         2


// Namespacing the package
using namespace d_fall_pps;
//using namespace std;




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


std::string m_ros_namespace;

ros::Subscriber requestLoadControllerYamlSubscriber_agent_to_self;


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

void requestLoadControllerYamlCallback(const std_msgs::Int32& msg);



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

    ROS_INFO_STREAM("The Parameter Service node received the message to load YAML parameters from file into cache");


    // Instantiate a local varaible to confirm that something can actually be loaded
    // from a YAML file
    bool isValidToAttemptLoad = true;

    // Instantiate a local variable for the string that will be passed to the "system"
    std::string cmd;

    // Get the abolute path to "d_fall_pps"
    std::string d_fall_pps_path = ros::package::getPath("d_fall_pps");

    // Switch between loading for the different controllers
    if ( (controller_to_load_yaml==LOAD_YAML_SAFE_CONTROLLER_COORDINATOR) && (my_type==TYPE_COORDINATOR) )
    {
        // Re-load the parameters of the safe controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + m_ros_namespace + "/SafeControllerService";
    }
    else if ( (controller_to_load_yaml==LOAD_YAML_SAFE_CONTROLLER_AGENT) && (my_type==TYPE_AGENT) )
    {
        // Re-load the parameters of the safe controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + m_ros_namespace + "/SafeControllerService";
    }
    else if ( (controller_to_load_yaml==LOAD_YAML_CUSTOM_CONTROLLER_COORDINATOR) && (my_type==TYPE_COORDINATOR) )
    {
        // Re-load the parameters of the custom controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/CustomController.yaml " + m_ros_namespace + "/CustomControllerService";
    }
    else if ( (controller_to_load_yaml==LOAD_YAML_CUSTOM_CONTROLLER_AGENT) && (my_type==TYPE_AGENT) )
    {
        // Re-load the parameters of the custom controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/CustomController.yaml " + m_ros_namespace + "/CustomControllerService";
    }
    else
    {
        // Let the user know that the command was not recognised
        ROS_INFO("> Unknown 'controller to load yaml' command, thus nothing will be loaded");
        // Set the boolean that prevents the fetch message from being sent
        isValidToAttemptLoad = false;
    }


    // Only bother with ttempting to loaded the .yaml file, and subseuently send the "ready for fetch"
    // message if something can actually be loaded from a YAML file
    if (isValidToAttemptLoad)
    {
        // Let the user know what is about to happen
        ROS_INFO_STREAM("> The following path will be used for locating the .yaml file: " << d_fall_pps_path  << " The comand line string sent to the 'system' is: " << cmd );

        // Re-load the parameters by pass the command line string via a "system" call
        // > i.e., this replicates pasting this string in a new terminal window and pressing enter
        system(cmd.c_str());

        // Pause breifly to ensure that the yaml file is fully loaded
        ros::Duration(0.5).sleep();

        // Instantiate a local varaible to confirm that something was actually loaded from
        // a YAML file
        bool isReadyForFetch = true;
    
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
                ROS_INFO("Unknown 'controller to load yaml' command, thus a 'ready to fetch' message will NOT be published.");
                // Set the boolean that prevents the fetch message from being sent
                isReadyForFetch = false;
                break;
        }
        // Send a message that the YAML parameter have been loaded and hence are
        // ready to be fetched (i.e., using getparam())
        if (isReadyForFetch)
        {
            controllerYamlReadyForFetchPublihser.publish(fetch_msg);
        }
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
    if ((!type_string.compare("coordinator")))
    {
        my_type = TYPE_COORDINATOR;
    }
    else if ((!type_string.compare("agent")))
    {
        my_type = TYPE_AGENT;
    }
    else
    {
        // Set "my_type" to the value indicating that it is invlid
        my_type = TYPE_INVALID;
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
    

    // Construct the string to the namespace of this Paramater Service
    // SUBSCRIBE TO THE APPROPRIATE "request" MESSAGES DEPENDING ON THE "my_type"
    switch (my_type)
    {
        case TYPE_AGENT:
        {
            //m_ros_namespace = ros::this_node::getNamespace();
            m_ros_namespace = "/" + std::to_string(my_agentID) + '/' + "ParameterService";
            ROS_INFO_STREAM("This Paramter Sercice will load .yaml file parameters into the 'base' namespace: " << m_ros_namespace);
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            //m_ros_namespace = ros::this_node::getNamespace();
            m_ros_namespace = '/' + "ParameterService";
            ROS_INFO_STREAM("This Paramter Sercice will load .yaml file parameters into the 'base' namespace: " << m_ros_namespace);
            break;
        }

        default:
        {
            ROS_ERROR("The 'my_type' type parameter was not recognised.");
            break;
        }
    }

    


    // Delare the subscribers as local variables here so that they persist for the life
    // time of this main() function. The varaibles will be assigned in the switch case below
    // > Subscribers for when this Parameter Service node is: TYPE_AGENT
    ros::Subscriber requestLoadControllerYamlSubscriber_agent_to_self;
    ros::Subscriber requestLoadControllerYamlSubscriber_agent_to_coordinator;
    // > Subscribers for when this Parameter Service node is: TYPE_COORDINATOR
    ros::Subscriber requestLoadControllerYamlSubscriber_coordinator_to_self;

    // SUBSCRIBE TO THE APPROPRIATE "request" MESSAGES DEPENDING ON THE "my_type"
    switch (my_type)
    {
        // AN AGENT TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        // > The agent's own "PPSClient" node
        case TYPE_AGENT:
        {
            // Subscribing to the agent's own PPSclient
            // > First: Construct a node handle to the PPSclient
            ros::NodeHandle nh_PPSClient_for_this_agent("PPSClient");
            // > Second: Subscribe to the "requestLoadControllerYaml" topic
            requestLoadControllerYamlSubscriber_agent_to_self = nh_PPSClient_for_this_agent.subscribe("requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);

            // Subscribing to the coordinator
            // > First: construct a node handle to the coordinator
            ros::NodeHandle nh_coordinator_for_this_agent = ros::NodeHandle();
            // > Second: Subscribe to the "requestLoadControllerYaml" topic
            requestLoadControllerYamlSubscriber_agent_to_coordinator = nh_coordinator_for_this_agent.subscribe("my_GUI/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);            

            // Inform the user what was subscribed to:
            ROS_INFO_STREAM("This Parameter Service has subscribed to 'requestLoadControllerYaml' messages from both the 'my_GUI' and the 'PPSClient'");
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            // Subscribing to the coordinator's own "my_GUI" 
            // > First: Get the node handle required
            ros::NodeHandle nh_coordinator_for_this_coordinator(ros::this_node::getNamespace());
            // > Second: Subscribe to requests from: the master GUI
            requestLoadControllerYamlSubscriber_coordinator_to_self = nh_coordinator_for_this_coordinator.subscribe("my_GUI/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);

            // Inform the user what was subscribed to:
            ROS_INFO_STREAM("This Parameter Service has subscribed to 'requestLoadControllerYaml' messages from 'my_GUI'");
            break;
        }

        default:
        {
            ROS_ERROR("The 'my_type' type parameter was not recognised.");
            break;
        }
    }


    ROS_INFO("CentralManagerService ready");
    ros::spin();

    return 0;
}
