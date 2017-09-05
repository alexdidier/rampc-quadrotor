// Place for students to implement their controller
// The ROS::service is set to work


//some useful libraries
#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"

//the generated structs from the msg-files have to be included
#include "d_fall_pps/ViconData.h"
#include "d_fall_pps/Setpoint.h"
#include "d_fall_pps/ControlCommand.h"
#include "d_fall_pps/Controller.h"

#include <std_msgs/Int32.h>

//constants
#define PI 3.1415926535
#define RATE_MODE 0
#define ANGLE_MODE 1
#define MOTOR_MODE 2

//constants that you most probably need for your controller to work properly
//see: 2015-08 - Forster - System ID of Crazyflie 2.pdf Chapter 3.3.1: Input Command → Thrust
const float FEEDFORWARD_MOTOR[4] = {37000, 37000, 37000, 37000};
const float MOTOR_REGRESSION_POLYNOMIAL[3] = {5.484560e-4, 1.032633e-6, 2.130295e-11};
const float SATURATION_THRUST = MOTOR_REGRESSION_POLYNOMIAL[2] * 12000 * 12000 + MOTOR_REGRESSION_POLYNOMIAL[1] * 12000 + MOTOR_REGRESSION_POLYNOMIAL[1];

//namespacing the package
using namespace d_fall_pps;


// load parameters from corresponding YAML file

void loadParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length) {
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

void loadCustomParameters(ros::NodeHandle& nodeHandle)
{
    // here we load the parameters that are in the CustomController.yaml
}


// ********* important function that has to be used!! *********
//-est- is an array with the estimated values : x,y,z,vx,vy,vz,roll,pitch,yaw
//-estBody- is an EMPTY array which will then contain the values in the body frame used by the crazyflie
//-yaw_measured- is the value that came from Vicon
void convertIntoBodyFrame(float est[9], float (&estBody)[9], int yaw_measured) {
    float sinYaw = sin(yaw_measured);
    float cosYaw = cos(yaw_measured);

    estBody[0] = est[0] * cosYaw + est[1] * sinYaw;
    estBody[1] = -est[0] * sinYaw + est[1] * cosYaw;
    estBody[2] = est[2];

    estBody[3] = est[3] * cosYaw + est[4] * sinYaw;
    estBody[4] = -est[3] * sinYaw + est[4] * cosYaw;
    estBody[5] = est[5];

    estBody[6] = est[6];
    estBody[7] = est[7];
    estBody[8] = est[8];
}




/* --- the data students can work with ---
    -request- contains data provided by Vicon. Check d_fall_pps/msg/ViconData.msg what it includes.
    -response- is where you have to write your calculated data into.
*/
bool calculateControlOutput(Controller::Request &request, Controller::Response &response) {
    //writing the data from -request- to command line
    //might be useful for debugging
    ROS_INFO_STREAM("x-coordinates: " << request.ownCrazyflie.x);
    ROS_INFO_STREAM("y-coordinates: " << request.ownCrazyflie.y);
    ROS_INFO_STREAM("z-coordinates: " << request.ownCrazyflie.z);
    ROS_INFO_STREAM("roll: " << request.ownCrazyflie.roll);
    ROS_INFO_STREAM("pitch: " << request.ownCrazyflie.pitch);
    ROS_INFO_STREAM("yaw: " << request.ownCrazyflie.yaw);
    ROS_INFO_STREAM("Delta t: " << request.ownCrazyflie.acquiringTime);



    // ********* do your calculations here *********
    //Tip: create functions that you call here to keep you code cleaner







    //for students to set the newly calculated commands for the controller
    response.controlOutput.roll = 0;
    response.controlOutput.pitch = 0;
    response.controlOutput.yaw = 0; //in [rad] --> will be converted to degree in CrazyRadio.py before sending to Crazyflie
    response.controlOutput.motorCmd1 = 0;
    response.controlOutput.motorCmd2 = 0;
    response.controlOutput.motorCmd3 = 0;
    response.controlOutput.motorCmd4 = 0;


    /*choosing the Crazyflie onBoard controller type.
    it can either be Motor, Rate or Angle based */
    response.controlOutput.onboardControllerType = MOTOR_MODE;
    //response.controlOutput.onboardControllerType = RATE_MODE;
    //response.controlOutput.onboardControllerType = ANGLE_MODE;

	return true;
}

void customYAMLloadedCallback(const std_msgs::Int32& msg)
{
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("received msg custom loaded YAML");
    loadCustomParameters(nodeHandle);
}

void setpointCallback(const Setpoint& newSetpoint) {
    // setpoint[0] = newSetpoint.x;
    // setpoint[1] = newSetpoint.y;
    // setpoint[2] = newSetpoint.z;
    // setpoint[3] = newSetpoint.yaw;
}


int main(int argc, char* argv[]) {
    //starting the ROS-node
    ros::init(argc, argv, "CustomControllerService");
    ros::NodeHandle nodeHandle("~");

    ros::Subscriber setpointSubscriber = nodeHandle.subscribe("Setpoint", 1, setpointCallback);

    ros::ServiceServer service = nodeHandle.advertiseService("CustomController", calculateControlOutput);

    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());
    ros::Subscriber customYAMLloadedSubscriber = namespace_nodeHandle.subscribe("student_GUI/customYAMLloaded", 1, customYAMLloadedCallback);

    ROS_INFO("CustomControllerService ready");
    ros::spin();

    return 0;
}
