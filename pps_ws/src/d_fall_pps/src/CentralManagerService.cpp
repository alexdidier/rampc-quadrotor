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
#include "d_fall_pps/CentralManager.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CrazyflieDB.h"
#include "d_fall_pps/CrazyflieContext.h"

#include "d_fall_pps/CMRead.h"
#include "d_fall_pps/CMQuery.h"
#include "d_fall_pps/CMUpdate.h"
#include "d_fall_pps/CMCommand.h"

using namespace d_fall_pps;
using namespace std;

//reads crazyflie db from the file specified in params
/*CrazyflieDB readCrazyflieDB() {
	return 0;
}*/

//writes crazyflie db to the file specified in params
void writeCrazyflieDB(CrazyflieDB& cfDB) {

}

bool cmRead(CMRead::Request &request, CMRead::Response &response) {

	return true;
}

bool cmQuery(CMQuery::Request &request, CMQuery::Response &response) {

    return true;
}

bool cmUpdate(CMUpdate::Request &request, CMUpdate::Response &response) {

	return true;
}

bool cmCommand(CMCommand::Request &request, CMCommand::Response &response) {

	return true;
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
