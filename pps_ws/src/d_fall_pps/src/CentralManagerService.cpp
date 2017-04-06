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

#include "ros/ros.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CentralManager.h"

using namespace d_fall_pps;


/*TODO:
request from paul
Specificy controller in a file



*/

bool returnCrazyflieContext(CentralManager::Request &request, CentralManager::Response &response) {
    ROS_INFO("cnetal manager");

	return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "CentralManagerService");

    ros::NodeHandle nodeHandle("~");

    ros::ServiceServer service = nodeHandle.advertiseService("CentralManager", returnCrazyflieContext);
    ROS_INFO("CentralManagerService ready");
    ros::spin();

    return 0;
}
