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

using namespace d_fall_pps;
using namespace std;

//reads crazyflie db from the file specified in params
/*CrazyflieDB readCrazyflieDB() {
	return 0;
}*/

//writes crazyflie db to the file specified in params
void writeCrazyflieDB(CrazyflieDB& cfDB) {

}
/*

//receive request from students (containing (maybe among other things) their name)
//send them back the CrazyflieContext (containing (maybe among other things) the area to fly in)
bool returnCrazyflieContext(CentralManager::Request &request, CentralManager::Response &response) {
    ROS_INFO("central manager");
	
	
	//TBD: crazyflie-dependent area assignment instead of hardcoding
		//request contains string crazyflieName
	//respond with area boundaries upon request
	response.context.localArea.xmin = -1.5;
	response.context.localArea.xmax = 1.5;
	response.context.localArea.ymin = -1.5;
	response.context.localArea.ymax = 1.5;
	response.context.localArea.zmin = -0.25;
	response.context.localArea.zmax = 0.8;

	return true;
}

bool cmRead(CMRead::Request &request, CMRead::Response &response) {
    ROS_INFO("central manager");
	
	
	//TBD: crazyflie-dependent area assignment instead of hardcoding
		//request contains string crazyflieName
	//respond with area boundaries upon request
	response.context.localArea.xmin = -1.5;
	response.context.localArea.xmax = 1.5;
	response.context.localArea.ymin = -1.5;
	response.context.localArea.ymax = 1.5;
	response.context.localArea.zmin = -0.25;
	response.context.localArea.zmax = 0.8;

	return true;
}

bool cmQuery(CMQuery::Request &request, CMQuery::Response &response) {
    ROS_INFO("central manager");
	
	
	//TBD: crazyflie-dependent area assignment instead of hardcoding
		//request contains string crazyflieName
	//respond with area boundaries upon request
	response.context.localArea.xmin = -1.5;
	response.context.localArea.xmax = 1.5;
	response.context.localArea.ymin = -1.5;
	response.context.localArea.ymax = 1.5;
	response.context.localArea.zmin = -0.25;
	response.context.localArea.zmax = 0.8;

	return true;
}*/

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "CentralManagerService");

    ros::NodeHandle nodeHandle("~");

    //ros::ServiceServer service = nodeHandle.advertiseService("CMRead", cmRead);
    ROS_INFO("CentralManagerService ready");
    ros::spin();

    return 0;
}
