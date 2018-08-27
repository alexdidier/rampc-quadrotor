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
//    The service that manages the context of the student groups.
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/CentralManagerService.h"





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
//    DDDD     A    TTTTT    A    BBBB     A     SSSS  EEEEE
//    D   D   A A     T     A A   B   B   A A   S      E
//    D   D  A   A    T    A   A  BBBB   A   A   SSS   EEE
//    D   D  AAAAA    T    AAAAA  B   B  AAAAA      S  E
//    DDDD   A   A    T    A   A  BBBB   A   A  SSSS   EEEEE
//    ----------------------------------------------------------------------------------

bool cmRead(CMRead::Request &request, CMRead::Response &response)
{
    response.crazyflieDB = crazyflieDB;
	return true;
}

int findEntryByStudID(unsigned int studID)
{
    for(int i = 0; i < crazyflieDB.crazyflieEntries.size(); i++)
    {
        CrazyflieEntry entry = crazyflieDB.crazyflieEntries[i];
        if(entry.studentID == studID)
        {
            return i;
        }
    }
    return -1;
}

bool cmQuery(CMQuery::Request &request, CMQuery::Response &response)
{
    int cfIndex = findEntryByStudID(request.studentID);
    if(cfIndex != -1)
    {
        response.crazyflieContext = crazyflieDB.crazyflieEntries[cfIndex].crazyflieContext;
        return true;
    }
    else
    {
        return false;
    }
}

int findEntryByCF(string name)
{
    for(int i = 0; i < crazyflieDB.crazyflieEntries.size(); i++)
    {
        CrazyflieEntry entry = crazyflieDB.crazyflieEntries[i];
        string cfName = entry.crazyflieContext.crazyflieName;
        if(cfName == name)
        {
            return i;
        }
    }
    return -1;
}

bool cmUpdate(CMUpdate::Request &request, CMUpdate::Response &response)
{
    switch(request.mode)
    {
        case ENTRY_INSERT_OR_UPDATE:
        {
            string cfName = request.crazyflieEntry.crazyflieContext.crazyflieName;
            int cfIndex = findEntryByCF(cfName);
            if(cfIndex == -1)
            {
                crazyflieDB.crazyflieEntries.push_back(request.crazyflieEntry);
            }
            else
            {
                crazyflieDB.crazyflieEntries[cfIndex] = request.crazyflieEntry;
            }
            return true;
        }

        case ENTRY_REMOVE: {
            string cfName = request.crazyflieEntry.crazyflieContext.crazyflieName;
            int cfIndex = findEntryByCF(cfName);
            if(cfIndex == -1)
            {
                return false;
            }
            else
            {
                crazyflieDB.crazyflieEntries.erase(crazyflieDB.crazyflieEntries.begin() +cfIndex);
                return true;
            }
        }

        default: return false;
    }
}

bool cmCommand(CMCommand::Request &request, CMCommand::Response &response)
{
    switch(request.command)
    {
        case CMD_SAVE:
        {
            writeCrazyflieDB(crazyflieDB);
            return true;
        }

        case CMD_RELOAD:
        {
            crazyflieDB.crazyflieEntries.clear();
            readCrazyflieDB(crazyflieDB);
            return true;
        }

        default: return false;
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
    ros::init(argc, argv, "CentralManagerService");

    ros::NodeHandle nodeHandle("~");
    
    readCrazyflieDB(crazyflieDB);

    ros::ServiceServer readService = nodeHandle.advertiseService("Read", cmRead);
    ros::ServiceServer queryService = nodeHandle.advertiseService("Query", cmQuery);
    ros::ServiceServer updateService = nodeHandle.advertiseService("Update", cmUpdate);
    ros::ServiceServer commandService = nodeHandle.advertiseService("Command", cmCommand);

    // Get the handle to the namespace in which this coordinator is launched
    //ros::NodeHandle namespaceNodeHandle = ros::NodeHandle();

    // Subscriber for requests that the controller parameters should be re-loaded from
    // the .YAML files on the coordinators machine, and then all the agents should be
    // request to fetch the parameters from itself, i.e., fetch parameters from the
    // coordinator.
    //ros::Subscriber controllerYamlReadyForFetchSubscriber = namespaceNodeHandle.subscribe("/ParameterService/controllerYamlReadyForFetch", 1, controllerYamlReadyForFetchCallback);


    ROS_INFO("CentralManagerService ready");
    ros::spin();

    return 0;
}
