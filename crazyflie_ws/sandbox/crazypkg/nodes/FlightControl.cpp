#include "CrazyFlieInclude.h"

//TODO callback for changing the controller or sample rate. The callback should define a new
//timer attached to the callbackRunInnerControlLoop (and also to outer if needed) with the
//specified sample time. Delete the previous timer object first.




int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "FlightControl");
    ros::NodeHandle nodeHandle("~");
    ros::Time::init();


    CControlMgr* pControlMgr=pGetControlMgr();

    pControlMgr->init(&nodeHandle);




    while(ros::ok())
    {
//        callbackQueueFlightControl.callAvailable(ros::WallDuration(0));
        //pControlMgr->runFlagged();
    }

    pControlMgr->closeLog();

    ROS_INFO("Exiting FlightControl");
}
