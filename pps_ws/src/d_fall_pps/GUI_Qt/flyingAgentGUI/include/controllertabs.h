#ifndef CONTROLLERTABS_H
#define CONTROLLERTABS_H

#include <QWidget>
#include <QVector>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
//#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
//#include "d_fall_pps/IntWithHeader.h"
//#include "d_fall_pps/SetpointWithHeader.h"
#include "d_fall_pps/CrazyflieData.h"

// Include the shared definitions
//#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace d_fall_pps;

#else
// Include the shared definitions
//#include "include/Constants_for_Qt_compile.h"

#endif

namespace Ui {
class ControllerTabs;
}

class ControllerTabs : public QWidget
{
    Q_OBJECT

public:
    explicit ControllerTabs(QWidget *parent = 0);
    ~ControllerTabs();


signals:
    void measuredPoseValueChanged(QVector<float> measuredPose);


private:
    Ui::ControllerTabs *ui;


#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE VARIABLES FOR ROS

    // SUBSRIBER
    // > For the pose data from a motion capture system
    ros::Subscriber poseDataSubscriber;


    // --------------------------------------------------- //
    // PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

    // > For the controller currently operating, received on
    //   "controllerUsedSubscriber"
    void poseDataReceivedCallback(const d_fall_pps::CrazyflieData& msg);


#endif


};

#endif // CONTROLLERTABS_H
