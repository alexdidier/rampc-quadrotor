#ifndef CONTROLLERTABS_H
#define CONTROLLERTABS_H

#include <QWidget>
#include <QMutex>
#include <QVector>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
//#include "d_fall_pps/IntWithHeader.h"
//#include "d_fall_pps/SetpointWithHeader.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/ViconData.h"

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

    // PUBLIC METHODS FOR TOGGLING THE VISISBLE CONTROLLERS
    void showHideController_toggle(QString qstr_label, QWidget * tab_widget_to_toggle);
    void showHideController_default_changed();
    void showHideController_student_changed();
    void showHideController_picker_changed();
    void showHideController_tuning_changed();
    void showHideController_safe_changed();


public slots:
    void setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll);
    void setObjectNameForDisplayingPoseData( QString object_name );


signals:
    void agentIDsToCoordinateChanged(QVector<int> agentIDs , bool shouldCoordinateAll);
    void measuredPoseValueChanged(float x , float y , float z , float roll , float pitch , float yaw , bool occluded);
    void poseDataUnavailableSignal();


private:
    Ui::ControllerTabs *ui;

    // --------------------------------------------------- //
    // PRIVATE VARIABLES

    // The type of this node, i.e., agent or a coordinator,
    // specified as a parameter in the "*.launch" file
    int m_type = 0;

    // The ID  of this node
    int m_ID;

    // For coordinating multiple agents
    std::vector<int> m_vector_of_agentIDs_toCoordinate;
    bool m_shouldCoordinateAll = true;
    QMutex m_agentIDs_toCoordinate_mutex;

    // The object name for which motion capture pose data
    // will be "emitted" using the "measuredPoseValueChanged"
    // signal
    std::string m_object_name_for_emitting_pose_data;

    // Flag for whether pose data should be emitted, this is
    // to save looking through the data when it is unnecessary
    bool m_should_search_pose_data_for_object_name = false;
    QMutex m_should_search_pose_data_for_object_name_mutex;

    // The color for normal and highlighted tabs
    QColor m_tab_text_colour_normal;
    QColor m_tab_text_colour_highlight;


#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE VARIABLES FOR ROS

    // SUBSRIBER
    // > For the pose data from a motion capture system
    ros::Subscriber m_poseDataSubscriber;
    // > For the controller that is currently operating
    ros::Subscriber controllerUsedSubscriber;
#endif


#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

    // > For the controller currently operating, received on
    //   "controllerUsedSubscriber"
    void poseDataReceivedCallback(const d_fall_pps::ViconData& viconData);

    void controllerUsedChangedCallback(const std_msgs::Int32& msg);

    // Get the paramters that specify the type and ID
    bool getTypeAndIDParameters();
#endif


    void setControllerEnabled(int new_controller);

    void setAllTabLabelsToNormalColouring();

    void setTextColourOfTabLabel(QColor color , QWidget * tab_widget);

};

#endif // CONTROLLERTABS_H
