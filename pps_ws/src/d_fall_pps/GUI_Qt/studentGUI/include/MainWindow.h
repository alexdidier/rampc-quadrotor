//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Angel Romero
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
//    Main window of the Student's GUI
//
//    ----------------------------------------------------------------------------------


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "rosNodeThread.h"

#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/Setpoint.h"


// Types of controllers being used:
#define SAFE_CONTROLLER   0
#define CUSTOM_CONTROLLER 1

// Commands for CrazyRadio
#define CMD_RECONNECT  0
#define CMD_DISCONNECT 1

// CrazyRadio states:
#define CONNECTED        0
#define CONNECTING       1
#define DISCONNECTED     2

// The constants that "command" changes in the
// operation state of this agent. These "commands"
// are sent from this GUI node to the "PPSClient"
// node where the command is enacted
#define CMD_USE_SAFE_CONTROLLER   1
#define CMD_USE_CUSTOM_CONTROLLER 2
#define CMD_CRAZYFLY_TAKE_OFF     3
#define CMD_CRAZYFLY_LAND         4
#define CMD_CRAZYFLY_MOTORS_OFF   5

// Flying States
#define STATE_MOTORS_OFF 1
#define STATE_TAKE_OFF   2
#define STATE_FLYING     3
#define STATE_LAND       4

// Battery states
#define BATTERY_STATE_NORMAL 0
#define BATTERY_STATE_LOW    1

// For which controller parameters to load
#define LOAD_YAML_SAFE_CONTROLLER_AGENT           1
#define LOAD_YAML_CUSTOM_CONTROLLER_AGENT         2
#define LOAD_YAML_SAFE_CONTROLLER_COORDINATOR     3
#define LOAD_YAML_CUSTOM_CONTROLLER_COORDINATOR   4

// Universal constants
#define PI 3.141592653589

#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void updateNewViconData(const ptrToMessage& p_msg);
    void on_RF_Connect_button_clicked();

    void on_take_off_button_clicked();

    void on_land_button_clicked();

    void on_motors_OFF_button_clicked();

    void on_set_setpoint_button_clicked();
    void on_set_setpoint_button_2_clicked();

    void on_RF_disconnect_button_clicked();

    void on_load_custom_yaml_button_clicked();
    void on_load_safe_yaml_button_clicked();

    void on_en_custom_controller_clicked();

    void on_en_safe_controller_clicked();

private:
    Ui::MainWindow *ui;

    rosNodeThread* m_rosNodeThread;
    int m_radio_status;
    float m_battery_voltage;
    int m_battery_level;

    std::string m_ros_namespace;

    ros::Timer m_timer_yaml_file_for_safe_controller;
    ros::Timer m_timer_yaml_file_for_custom_controlller;

    int m_student_id;
    CrazyflieContext m_context;

    Setpoint m_safe_setpoint;
    Setpoint m_custom_setpoint;

    int m_battery_state;

    ros::Publisher crazyRadioCommandPublisher;
    ros::Subscriber crazyRadioStatusSubscriber;
    ros::Publisher PPSClientCommandPublisher;
    ros::Subscriber CFBatterySubscriber;
    ros::Subscriber flyingStateSubscriber;
    ros::Subscriber batteryStateSubscriber;

    ros::Publisher controllerSetpointPublisher;
    ros::Subscriber safeSetpointSubscriber;

    ros::Publisher customSetpointPublisher;
    ros::Subscriber customSetpointSubscriber;

    ros::Subscriber DBChangedSubscriber;



    // > For publishing a message that requests the 
    //   YAML parameters to be re-loaded from file
    // > The message contents specify which controller
    //   the parameters should be re-loaded for
    ros::Publisher requestLoadControllerYamlPublisher;

    // Subscriber for locking the load the controller YAML
    // parameters when the Coordintor GUI requests a load
    ros::Subscriber requestLoadControllerYaml_from_my_GUI_Subscriber;


    ros::Subscriber controllerUsedSubscriber;

    ros::ServiceClient centralManager;

    // callbacks
    void crazyRadioStatusCallback(const std_msgs::Int32& msg);
    void CFBatteryCallback(const std_msgs::Float32& msg);
    void flyingStateChangedCallback(const std_msgs::Int32& msg);
    void safeSetpointCallback(const Setpoint& newSetpoint);
    void customSetpointCallback(const Setpoint& newSetpoint);
    void DBChangedCallback(const std_msgs::Int32& msg);
    void customYamlFileTimerCallback(const ros::TimerEvent&);
    void safeYamlFileTimerCallback(const ros::TimerEvent&);
    void requestLoadControllerYaml_from_my_GUI_Callback(const std_msgs::Int32& msg);
    void controllerUsedChangedCallback(const std_msgs::Int32& msg);
    void batteryStateChangedCallback(const std_msgs::Int32& msg);

    float fromVoltageToPercent(float voltage);
    void updateBatteryVoltage(float battery_voltage);
    void setCrazyRadioStatus(int radio_status);
    void loadCrazyflieContext();
    void coordinatesToLocal(CrazyflieData& cf);
    void initialize_custom_setpoint();


    void disableGUI();
    void enableGUI();
    void highlightSafeControllerTab();
    void highlightCustomControllerTab();

    const std::vector<float> m_cutoff_voltages {3.1966,        3.2711,        3.3061,        3.3229,        3.3423,        3.3592,        3.3694,        3.385,        3.4006,        3.4044,        3.4228,        3.4228,        3.4301,        3.4445,        3.4531,        3.4677,        3.4705,        3.4712,        3.4756,        3.483,        3.4944,        3.5008,        3.5008,        3.5084,        3.511,        3.5122,        3.5243,        3.5329,        3.5412,        3.5529,        3.5609,        3.5625,        3.5638,        3.5848,        3.6016,        3.6089,        3.6223,        3.628,        3.6299,        3.6436,        3.6649,        3.6878,        3.6983,        3.7171,        3.7231,        3.7464,        3.7664,        3.7938,        3.8008,        3.816,        3.8313,        3.8482,        3.866,        3.8857,        3.8984,        3.9159,        3.9302,        3.9691,        3.997,        4.14    };
};

#endif // MAINWINDOW_H
