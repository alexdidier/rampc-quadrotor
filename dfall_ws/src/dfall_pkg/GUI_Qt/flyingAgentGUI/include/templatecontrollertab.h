//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
//
//    This file is part of D-FaLL-System.
//
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    The GUI for a Template Controller for students build from
//
//    ----------------------------------------------------------------------------------





#ifndef TEMPLATECONTROLLERTAB_H
#define TEMPLATECONTROLLERTAB_H

#include <QWidget>
#include <QMutex>
#include <QVector>
#include <QLineEdit>
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
//#include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/SetpointWithHeader.h"
#include "dfall_pkg/CustomButtonWithHeader.h"

// Include the DFALL service types
#include "dfall_pkg/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace dfall_pkg;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

#endif





namespace Ui {
class TemplateControllerTab;
}

class TemplateControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit TemplateControllerTab(QWidget *parent = 0);
    ~TemplateControllerTab();



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

    void on_custom_button_1_clicked();
    void on_custom_button_2_clicked();
    void on_custom_button_3_clicked();





private:
    Ui::TemplateControllerTab *ui;

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
    void setpointChangedCallback(const dfall_pkg::SetpointWithHeader& newSetpoint);

    // Publish a message when a custom button is pressed
    void publish_custom_button_command(int button_index , QLineEdit * lineEdit_pointer);

    // Fill the header for a message
    void fillSetpointMessageHeader( dfall_pkg::SetpointWithHeader & msg );
    void fillCustomButtonMessageHeader( dfall_pkg::CustomButtonWithHeader & msg );

    // Get the paramters that specify the type and ID
    bool getTypeAndIDParameters();
#endif

    void publishSetpoint(float x, float y, float z, float yaw_degrees);

};

#endif // TEMPLATECONTROLLERTAB_H
