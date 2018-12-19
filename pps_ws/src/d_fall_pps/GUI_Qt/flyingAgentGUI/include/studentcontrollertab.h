#ifndef STUDENTCONTROLLERTAB_H
#define STUDENTCONTROLLERTAB_H

#include <QWidget>
#include <QMutex>
#include <QVector>
#include <QTextStream>

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
#include "d_fall_pps/SetpointWithHeader.h"
#include "d_fall_pps/CustomButtonWithHeader"

// Include the DFALL service types
#include "d_fall_pps/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace d_fall_pps;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

#endif



namespace Ui {
class StudentControllerTab;
}

class StudentControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit StudentControllerTab(QWidget *parent = 0);
    ~StudentControllerTab();



public slots:
    void setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll);
    void setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded);
    void poseDataUnavailableSlot();



private slots:
    void on_lineEdit_setpoint_new_x_returnPressed();
    void on_lineEdit_setpoint_new_y_returnPressed();
    void on_lineEdit_setpoint_new_z_returnPressed();
    void on_lineEdit_setpoint_new_yaw_returnPressed();

    void on_set_setpoint_button_clicked();

    void on_default_setpoint_button_clicked();

    void on_x_increment_plus_button_clicked();
    void on_x_increment_minus_button_clicked();
    void on_y_increment_plus_button_clicked();
    void on_y_increment_minus_button_clicked();
    void on_z_increment_plus_button_clicked();
    void on_z_increment_minus_button_clicked();
    void on_yaw_increment_plus_button_clicked();
    void on_yaw_increment_minus_button_clicked();

    void on_custom_button_1_clicked();
    void on_custom_button_2_clicked();
    void on_custom_button_3_clicked();
    void on_custom_button_4_clicked();
    void on_custom_button_5_clicked();



private:
    Ui::StudentControllerTab *ui;

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



#ifdef CATKIN_MAKE
    // PUBLISHER
    // > For requesting the setpoint to be changed
    ros::Publisher requestSetpointChangePublisher;

    // SUBSCRIBER
    // > For being notified when the setpoint is changed
    ros::Subscriber setpointChangedSubscriber;

    // PUBLISHER
    // > For notifying that a custom button is pressed
    ros::Publisher customButtonPublisher;
#endif



    // --------------------------------------------------- //
    // PRIVATE FUNCTIONS


#ifdef CATKIN_MAKE
    // For receiving message that the setpoint was changed
    void setpointChangedCallback(const d_fall_pps::SetpointWithHeader& newSetpoint);

    // Publish a message when a custom button is pressed
    void publish_custom_button_command(int button_index , QLineEdit * lineEdit_pointer);

    // Fill the header for a message
    void fillSetpointMessageHeader( d_fall_pps::SetpointWithHeader & msg );
    void fillCustomButtonMessageHeader( d_fall_pps::CustomButtonWithHeader & msg );

    // Get the paramters that specify the type and ID
    bool getTypeAndIDParameters();
#endif

    void publishSetpoint(float x, float y, float z, float yaw);

};

#endif // STUDENTCONTROLLERTAB_H
