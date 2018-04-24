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
#include "ParameterService.h"





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
        cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + m_base_namespace + "/SafeController";
    }
    else if ( (controller_to_load_yaml==LOAD_YAML_SAFE_CONTROLLER_AGENT) && (my_type==TYPE_AGENT) )
    {
        // Re-load the parameters of the safe controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + m_base_namespace + "/SafeController";
    }
    else if ( (controller_to_load_yaml==LOAD_YAML_DEMO_CONTROLLER_COORDINATOR) && (my_type==TYPE_COORDINATOR) )
    {
        // Re-load the parameters of the demo controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/DemoController.yaml " + m_base_namespace + "/DemoController";
    }
    else if ( (controller_to_load_yaml==LOAD_YAML_DEMO_CONTROLLER_AGENT) && (my_type==TYPE_AGENT) )
    {
        // Re-load the parameters of the demo controller:
        cmd = "rosparam load " + d_fall_pps_path + "/param/DemoController.yaml " + m_base_namespace + "/DemoController";
    }
    else
    {
        // Let the user know that the command was not recognised
        ROS_INFO_STREAM("> Nothing to load for this parameter service with.");
        ROS_INFO_STREAM("> The message received has 'controller_to_load_yaml'   =  " << controller_to_load_yaml);
        ROS_INFO_STREAM("> And the type of this Parameter Service is 'my_type'  =  " << my_type);
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
            case (LOAD_YAML_DEMO_CONTROLLER_COORDINATOR):
                fetch_msg.data = FETCH_YAML_DEMO_CONTROLLER_FROM_COORDINATOR;
                break;
            case (LOAD_YAML_SAFE_CONTROLLER_AGENT):
                fetch_msg.data = FETCH_YAML_SAFE_CONTROLLER_FROM_OWN_AGENT;
                break;
            case (LOAD_YAML_DEMO_CONTROLLER_AGENT):
                fetch_msg.data = FETCH_YAML_DEMO_CONTROLLER_FROM_OWN_AGENT;
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

    // Get the namespace of this "ParameterService" node
    std::string m_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("For ParameterService, ros::this_node::getNamespace() =  " << m_namespace);



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
        ROS_ERROR("The 'type' parameter retrieved was not recognised.");
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
    switch (my_type)
    {
        case TYPE_AGENT:
        {
            //m_base_namespace = ros::this_node::getNamespace();
            //m_base_namespace = "/agent" + my_agentID + '/' + "ParameterService";
            m_base_namespace = m_namespace + '/' + "ParameterService";
            ROS_INFO_STREAM("This Paramter Sercice will load .yaml file parameters into the 'base' namespace: " << m_base_namespace);
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            //m_base_namespace = ros::this_node::getNamespace();
            //m_base_namespace = "/ParameterService";
            m_base_namespace = m_namespace + '/' + "ParameterService";
            ROS_INFO_STREAM("This Paramter Sercice will load .yaml file parameters into the 'base' namespace: " << m_base_namespace);
            break;
        }

        default:
        {
            ROS_ERROR("The 'my_type' type parameter was not recognised.");
            break;
        }
    }

    


    // SUBSCRIBE TO THE APPROPRIATE "request" MESSAGES DEPENDING ON THE "my_type"
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
            requestLoadControllerYamlSubscriber_agent_to_coordinator = nh_coordinator_for_this_agent.subscribe("/my_GUI/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);            

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
            ros::NodeHandle nh_coordinator_for_this_coordinator = ros::NodeHandle();
            // > Second: Subscribe to requests from: the master GUI
            requestLoadControllerYamlSubscriber_coordinator_to_self = nh_coordinator_for_this_coordinator.subscribe("/my_GUI/requestLoadControllerYaml", 1, requestLoadControllerYamlCallback);

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
