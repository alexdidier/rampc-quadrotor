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

//namespacing the package
using namespace d_fall_pps;

// variables for controller
float cf_mass;                  //crazyflie mass in grams
std::vector<float>  motorPoly(3);
float control_frequency;
float gravity_force;

CrazyflieData previous_location;

std::vector<float>  setpoint(4);

const float gainMatrixRoll[9] = {0, -1.714330725, 0, 0, -1.337107465, 0, 5.115369735, 0, 0};
const float gainMatrixPitch[9] = {1.714330725, 0, 0, 1.337107465, 0, 0, 0, 5.115369735, 0};
const float gainMatrixYaw[9] = {0, 0, 0, 0, 0, 0, 0, 0, 2.843099534};
const float gainMatrixThrust[9] = {0, 0, 0.22195826, 0, 0, 0.12362477, 0, 0, 0};



// load parameters from corresponding YAML file

void loadParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}

float getFloatParameter(ros::NodeHandle& nodeHandle, std::string name)
{

    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}


void loadCustomParameters(ros::NodeHandle& nodeHandle)
{
    // here we load the parameters that are in the CustomController.yaml

    cf_mass = getFloatParameter(nodeHandle, "mass");
    control_frequency = getFloatParameter(nodeHandle, "control_frequency");
    loadParameterFloatVector(nodeHandle, "motorPoly", motorPoly, 3);

    // compute things that we will need after from these parameters

    // force that we need to counteract gravity (mg)
    gravity_force = cf_mass * 9.81/(1000*4); // in N
}

float computeMotorPolyBackward(float thrust) {
    return (-motorPoly[1] + sqrt(motorPoly[1] * motorPoly[1] - 4 * motorPoly[2] * (motorPoly[0] - thrust))) / (2 * motorPoly[2]);
}


// ********* important function that has to be used!! *********
//-est- is an array with the estimated values : x,y,z,vx,vy,vz,roll,pitch,yaw
//-estBody- is an EMPTY array which will then contain the values in the body frame used by the crazyflie
//-yaw_measured- is the value that came from Vicon
void convertIntoBodyFrame(float est[9], float (&estBody)[9], int yaw_measured)
{
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
    // ROS_INFO_STREAM("x-coordinates: " << request.ownCrazyflie.x);
    // ROS_INFO_STREAM("y-coordinates: " << request.ownCrazyflie.y);
    // ROS_INFO_STREAM("z-coordinates: " << request.ownCrazyflie.z);
    // ROS_INFO_STREAM("roll: " << request.ownCrazyflie.roll);
    // ROS_INFO_STREAM("pitch: " << request.ownCrazyflie.pitch);
    // ROS_INFO_STREAM("yaw: " << request.ownCrazyflie.yaw);
    // ROS_INFO_STREAM("Delta t: " << request.ownCrazyflie.acquiringTime);

    // ********* do your calculations here *********
    //Tip: create functions that you call here to keep you code cleaner
    ROS_INFO("custom controller loop");

    // calculate the velocity based in the derivative of the position

    //move coordinate system to make setpoint origin
    request.ownCrazyflie.x -= setpoint[0];
    request.ownCrazyflie.y -= setpoint[1];
    request.ownCrazyflie.z -= setpoint[2];
    float yaw = request.ownCrazyflie.yaw - setpoint[3];

    while(yaw > PI) {yaw -= 2 * PI;}
    while(yaw < -PI) {yaw += 2 * PI;}
    request.ownCrazyflie.yaw = yaw;

    float est[9];               // vector for the estimation of the state

    est[0] = request.ownCrazyflie.x;
    est[1] = request.ownCrazyflie.y;
    est[2] = request.ownCrazyflie.z;

    // estimate speed of crazyflie. Simplest way: discrete derivative
    est[3] = (request.ownCrazyflie.x - previous_location.x) * control_frequency;
    est[4] = (request.ownCrazyflie.y - previous_location.y) * control_frequency;
    est[5] = (request.ownCrazyflie.z - previous_location.z) * control_frequency;

    est[6] = request.ownCrazyflie.roll;
    est[7] = request.ownCrazyflie.pitch;
    est[8] = request.ownCrazyflie.yaw;

    float state[9];
    convertIntoBodyFrame(est, state, request.ownCrazyflie.yaw);

    // calculate feedback
    float outRoll = 0;
    float outPitch = 0;
    float outYaw = 0;
    float thrustIntermediate = 0;
    for(int i = 0; i < 9; ++i)
    {
    	outRoll -= gainMatrixRoll[i] * state[i];
    	outPitch -= gainMatrixPitch[i] * state[i];
    	outYaw -= gainMatrixYaw[i] * state[i];
    	thrustIntermediate -= gainMatrixThrust[i] * state[i];
    }

    ROS_INFO_STREAM("thrustIntermediate = " << thrustIntermediate);

    response.controlOutput.roll = outRoll;
    response.controlOutput.pitch = outPitch;
    response.controlOutput.yaw = outYaw;

    ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
    ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
    ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);

    response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrustIntermediate + gravity_force);
    response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrustIntermediate + gravity_force);

    ROS_INFO_STREAM("motorPoly 0:" << motorPoly[0]);
    ROS_INFO_STREAM("motorPoly 0:" << motorPoly[1]);
    ROS_INFO_STREAM("motorPoly 0:" << motorPoly[2]);

    ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
    ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
    ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
    ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);

    /*choosing the Crazyflie onBoard controller type.
    it can either be Motor, Rate or Angle based */
    // response.controlOutput.onboardControllerType = MOTOR_MODE;
    response.controlOutput.onboardControllerType = RATE_MODE;
    // response.controlOutput.onboardControllerType = ANGLE_MODE;

    previous_location = request.ownCrazyflie; // we have already used previous location, update it
	return true;
}

void customYAMLloadedCallback(const std_msgs::Int32& msg)
{
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("received msg custom loaded YAML");
    loadCustomParameters(nodeHandle);
}

void setpointCallback(const Setpoint& newSetpoint) {
    setpoint[0] = newSetpoint.x;
    setpoint[1] = newSetpoint.y;
    setpoint[2] = newSetpoint.z;
    setpoint[3] = newSetpoint.yaw;
}


int main(int argc, char* argv[]) {
    //starting the ROS-node
    ros::init(argc, argv, "CustomControllerService");
    ros::NodeHandle nodeHandle("~");
    loadCustomParameters(nodeHandle);

    ros::Subscriber setpointSubscriber = nodeHandle.subscribe("Setpoint", 1, setpointCallback);

    ros::ServiceServer service = nodeHandle.advertiseService("CustomController", calculateControlOutput);

    ros::NodeHandle namespace_nodeHandle(ros::this_node::getNamespace());
    ros::Subscriber customYAMLloadedSubscriber = namespace_nodeHandle.subscribe("student_GUI/customYAMLloaded", 1, customYAMLloadedCallback);

    ROS_INFO("CustomControllerService ready");
    ros::spin();

    return 0;
}
