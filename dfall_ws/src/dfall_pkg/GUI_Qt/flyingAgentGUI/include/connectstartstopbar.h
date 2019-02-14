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
//    The bar of the GUI for (dis-)connecting (from)to the Crazyradio
//    and for sending the {take-off,land,motors-off} commands//
//
//    ----------------------------------------------------------------------------------





#ifndef CONNECTSTARTSTOPBAR_H
#define CONNECTSTARTSTOPBAR_H

#include <string>
#include <QWidget>
#include <QMutex>

#include <QTextStream>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/AreaBounds.h"
#include "dfall_pkg/CrazyflieContext.h"
#include "dfall_pkg/IntIntService.h"
#include "dfall_pkg/CMQuery.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace dfall_pkg;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

#endif

// BATTERY LABEL IMAGE INDEX
#define BATTERY_LABEL_IMAGE_INDEX_UNVAILABLE  -1
#define BATTERY_LABEL_IMAGE_INDEX_EMPTY        0
#define BATTERY_LABEL_IMAGE_INDEX_20           1
#define BATTERY_LABEL_IMAGE_INDEX_40           2
#define BATTERY_LABEL_IMAGE_INDEX_60           3
#define BATTERY_LABEL_IMAGE_INDEX_80           4
#define BATTERY_LABEL_IMAGE_INDEX_FULL         5
#define BATTERY_LABEL_IMAGE_INDEX_UNKNOWN      6

namespace Ui {
class ConnectStartStopBar;
}

class ConnectStartStopBar : public QWidget
{
    Q_OBJECT

public:
    explicit ConnectStartStopBar(QWidget *parent = 0);
    ~ConnectStartStopBar();


public slots:
    void setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll);


private:
	// --------------------------------------------------- //
    // PRIVATE VARIABLES
    Ui::ConnectStartStopBar *ui;

    // > For the type of this node,
    //   i.e., an agent or a coordinator
	int m_type = 0;

	// > For the ID of this node
	int m_ID = 0;

    // For coordinating multiple agents
    std::vector<int> m_vector_of_agentIDs_toCoordinate;
    bool m_shouldCoordinateAll = true;
    QMutex m_agentIDs_toCoordinate_mutex;




	// > For keeping track of the current battery state
    int m_battery_state;
    // > For keeping track of which image is currently displayed
    int m_battery_status_label_image_current_index;



    // MUTEX FOR HANDLING ACCESS
    // > For the "rf_status_label" UI element
    //QMutex m_rf_status_label_mutex;
    // > For the "battery_voltage_lineEdit" UI element
    //QMutex m_battery_voltage_lineEdit_mutex;
    // > For the "battery_status_label" UI element
    QMutex m_battery_status_label_mutex;



    // --------------------------------------------------- //
    // PRIVATE FUNCTIONS

    // > For making the "enable flight" and "disable flight"
    //   buttons (un-)available
    void disableFlyingStateButtons();
    void enableFlyingStateButtons();

    // > For updating the RF Radio status shown in the UI element
    //   of "rf_status_label"
    void setCrazyRadioStatus(int radio_status);

    // > For updating the battery state
    void setBatteryState(int new_battery_state);
    // > For updating the battery voltage shown in the UI elements 
    //   of "battery_voltage_lineEdit" and "battery_status_label"
    void setBatteryVoltageText(float battery_voltage);
    void setBatteryImageBasedOnLevel(int battery_level);

    // > For updating the "my_flying_state" variable, and the
    //   UI element of "flying_state_label"
    void setFlyingState(int new_flying_state);



#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE VARIABLES FOR ROS


    // PUBLISHERS AND SUBSRIBERS
    // > For Crazyradio commands based on button clicks
    ros::Publisher crazyRadioCommandPublisher;
    // > For updating the "rf_status_label" picture
    ros::Subscriber crazyRadioStatusSubscriber;

    // > For updating the current battery voltage
    ros::Subscriber batteryVoltageSubscriber;
    // > For updating the current battery state
    //ros::Subscriber batteryStateSubscriber;
    // > For updating the current battery level
    ros::Subscriber batteryLevelSubscriber;

    // > For Flying state commands based on button clicks
    ros::Publisher flyingStateCommandPublisher;
    // > For updating the "flying_state_label" picture
    ros::Subscriber flyingStateSubscriber;


    // --------------------------------------------------- //
    // PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

    // Get the type and ID of this node
    bool getTypeAndIDParameters();
	// Fill the head for a message
    void fillIntMessageHeader( dfall_pkg::IntWithHeader & msg );

    // > For the CrazyRadio status, received on the
    //   "crazyRadioStatusSubscriber"
    void crazyRadioStatusCallback(const std_msgs::Int32& msg);

    // > For the Battery Voltage, received on the
    //   "batteryVoltageSubscriber"
    void batteryVoltageCallback(const std_msgs::Float32& msg);
    // > For the Battery State, receieved on the
    //   "batteryStateSubscriber"
    void batteryStateChangedCallback(const std_msgs::Int32& msg);
    // > For the Battery Level, receieved on the
    //   "batteryLevelSubscriber"
    void batteryLevelCallback(const std_msgs::Int32& msg);

    // > For the Flying State, received on the
    //   "flyingStateSubscriber"
    void flyingStateChangedCallback(const std_msgs::Int32& msg);

#endif





private slots:

	// PRIVATE METHODS FOR BUTTON CALLBACKS
    // > For the RF Crazyradio connect/disconnect buttons
	void on_rf_connect_button_clicked();
    void on_rf_disconnect_button_clicked();
    // > For the enable, disable, motors-off buttons
    void on_enable_flying_button_clicked();
    void on_disable_flying_button_clicked();
    void on_motors_off_button_clicked();


};

#endif // CONNECTSTARTSTOPBAR_H
