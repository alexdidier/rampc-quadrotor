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
//    An individual row in the coordinator part of the Flying Agent
//    GUI. This is essentially a condensed version of the
//    "Connect Start Stop Bar"
//
//    ----------------------------------------------------------------------------------





#ifndef COORDINATORROW_H
#define COORDINATORROW_H

#include <string>
#include <QWidget>
#include <QMutex>

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
#include "dfall_pkg/CMQuery.h"
#include "dfall_pkg/IntIntService.h"

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
class CoordinatorRow;
}

class CoordinatorRow : public QWidget
{
    Q_OBJECT

public:
    explicit CoordinatorRow(QWidget *parent = 0, int agentID = 0);
    ~CoordinatorRow();

    // PUBLIC METHODS FOR SETTING PROPERTIES
    // > Set the state of the checkbox
    void setShouldCoordinate(bool shouldCoordinate);
    // > Set the level of detail to display
    void setLevelOfDetailToDisplay(int level);


signals:
    void shouldCoordinateThisAgentValueChanged(int agentID , bool shouldCoordinate);


private:
    // --------------------------------------------------- //
    // PRIVATE VARIABLES
    Ui::CoordinatorRow *ui;

    // > For the ID of which agent this "coordinator row" relates to
    int m_agentID;
    // > For using the agent ID in constructing namespaces
    QString m_agentID_as_string;
    // > For the name of the allocated Crazyflie
    QString m_crazyflie_name_as_string;


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

    // > For making the "enable flight" and "disable flight" buttons
    //   (un-)available
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

    // > For loading the "context" for this agent,
    //   i.e., the {agentID,cfID,flying zone} tuple
    void loadCrazyflieContext();

    // > For requesting the current flying state
    //   i.e., using the service advertised by the Flying Agent Client
    void getCurrentFlyingState();

    // > For requesting the current state of the Crazy Radio
    //   i.e., using the service advertised by the Flying Agent Client
    void getCurrentCrazyRadioState();

    // > For updating the text in the UI element of
    //   "controller_enabled_label"
    void setControllerEnabled(int new_controller);



#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE VARIABLES FOR ROS

    // > For the "context" of this agent
    dfall_pkg::CrazyflieContext my_context;

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

    // > For changes in the database that defines {agentID,cfID,flying zone} links
    ros::Subscriber databaseChangedSubscriber;
    ros::ServiceClient centralManagerDatabaseService;
    // > For updating the controller that is currently operating
    ros::Subscriber controllerUsedSubscriber;

    // > For requesting the current flying state,
    //   this is used only for initialising the icon
    ros::ServiceClient getCurrentFlyingStateService;

    // > For requesting the current state of the Crazy Radio,
    //   this is used only for initialising the icon
    ros::ServiceClient getCurrentCrazyRadioStateService;


    // --------------------------------------------------- //
    // PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

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

    // > For the notification that the database was changes,
    //   received on the "DatabaseChangedSubscriber"
    void databaseChangedCallback(const std_msgs::Int32& msg);

    // > For the controller currently operating, received on
    //   "controllerUsedSubscriber"
    void controllerUsedChangedCallback(const std_msgs::Int32& msg);


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

    // > For the "should coordinate" checkbox
    void on_shouldCoordinate_checkBox_clicked();

};

#endif // COORDINATORROW_H
