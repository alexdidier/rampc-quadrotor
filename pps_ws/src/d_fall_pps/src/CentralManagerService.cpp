//    The service that manages the context of the student groups.
//    Copyright (C) 2017  Cyrill Burgener, Marco Mueller, Philipp Friedli
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdlib.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "rosbag/bag.h"
#include "d_fall_pps/CentralManager.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CrazyflieDB.h"
#include "d_fall_pps/CrazyflieContext.h"

#include "d_fall_pps/CMRead.h"
#include "d_fall_pps/CMQuery.h"
#include "d_fall_pps/CMUpdate.h"
#include "d_fall_pps/CMCommand.h"
#include "CentralManagerService.h"

using namespace d_fall_pps;
using namespace std;

CrazyflieDB crazyflieDB;
rosbag::Bag bag;

void saveCrazyflieDB() {
    string packagePath = ros::package::getPath("d_fall_pps") + "/";
    string dbFile = packagePath + "crazyflie.db";
    bag.open(dbFile, rosbag::bagmode::Write);
    bag.write("crazyflie_db", ros::Time::now(), crazyflieDB);
}

void loadCrazyflieDB() {
//
}

bool cmRead(CMRead::Request &request, CMRead::Response &response) {
    response.crazyflieDB = crazyflieDB;
	return true;
}

int findEntryByStudID(unsigned int studID) {
    for(int i = 0; i < crazyflieDB.crazyflieEntries.size(); i++) {
        CrazyflieEntry entry = crazyflieDB.crazyflieEntries[i];
        if(entry.studentID == studID) {
            return i;
        }
    }
    return -1;
}

bool cmQuery(CMQuery::Request &request, CMQuery::Response &response) {
    int cfIndex = findEntryByStudID(request.studentID);
    if(cfIndex != -1) {
        response.crazyflieContext = crazyflieDB.crazyflieEntries[cfIndex].crazyflieContext;
        return true;
    } else {
        return false;
    }
}

int findEntryByCF(string name) {
    for(int i = 0; i < crazyflieDB.crazyflieEntries.size(); i++) {
        CrazyflieEntry entry = crazyflieDB.crazyflieEntries[i];
        string cfName = entry.crazyflieContext.crazyflieName;
        if(cfName == name) {
            return i;
        }
    }
    return -1;
}

bool cmUpdate(CMUpdate::Request &request, CMUpdate::Response &response) {
    switch(request.mode) {
        case ENTRY_INSERT_OR_UPDATE: {
            string cfName = request.crazyflieEntry.crazyflieContext.crazyflieName;
            int cfIndex = findEntryByCF(cfName);
            if(cfIndex == -1) {
                crazyflieDB.crazyflieEntries.push_back(request.crazyflieEntry);
            } else {
                crazyflieDB.crazyflieEntries[cfIndex] = request.crazyflieEntry;
            }
            return true;
        }

        case ENTRY_REMOVE: {
            string cfName = request.crazyflieEntry.crazyflieContext.crazyflieName;
            int cfIndex = findEntryByCF(cfName);
            if(cfIndex == -1) {
                return false;
            } else {
                crazyflieDB.crazyflieEntries.erase(crazyflieDB.crazyflieEntries.begin() +cfIndex);
                return true;
            }
        }

        default: return false;
    }
}

bool cmCommand(CMCommand::Request &request, CMCommand::Response &response) {
    switch(request.command) {
        case CMD_SAVE: {
            //writeCrazyflieDB(crazyflieDB);
            return true;
        }

        case CMD_RELOAD: {
            //crazyflieDB = readCrazyflieDB();
            return true;
        }

        default: return false;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "CentralManagerService");

    ros::NodeHandle nodeHandle("~");

    ros::ServiceServer readService = nodeHandle.advertiseService("Read", cmRead);
    ros::ServiceServer queryService = nodeHandle.advertiseService("Query", cmQuery);
    ros::ServiceServer updateService = nodeHandle.advertiseService("Update", cmUpdate);
    ros::ServiceServer commandService = nodeHandle.advertiseService("Command", cmCommand);

    ROS_INFO("CentralManagerService ready");
    ros::spin();

    return 0;
}
