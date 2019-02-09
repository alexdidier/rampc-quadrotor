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
#include <QShortcut>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "rosNodeThread_for_studentGUI.h"

#include "dfall_pkg/CrazyflieContext.h"
#include "dfall_pkg/CrazyflieData.h"
#include "dfall_pkg/Setpoint.h"
#include "dfall_pkg/SetpointV2.h"
#include "dfall_pkg/ViconSubscribeObjectName.h"


// Types of controllers being used:
#define SAFE_CONTROLLER    1
#define DEMO_CONTROLLER    2
#define STUDENT_CONTROLLER 3
#define MPC_CONTROLLER     4
#define REMOTE_CONTROLLER  5
#define TUNING_CONTROLLER  6
#define PICKER_CONTROLLER  7


// Commands for CrazyRadio
#define CMD_RECONNECT  0
#define CMD_DISCONNECT 1

// CrazyRadio states:
#define CONNECTED        0
#define CONNECTING       1
#define DISCONNECTED     2

// The constants that "command" changes in the
// operation state of this agent. These "commands"
// are sent from this GUI node to the "FlyingAgentClient"
// node where the command is enacted
#define CMD_USE_SAFE_CONTROLLER      1
#define CMD_USE_DEMO_CONTROLLER      2
#define CMD_USE_STUDENT_CONTROLLER   3
#define CMD_USE_MPC_CONTROLLER       4
#define CMD_USE_REMOTE_CONTROLLER    5
#define CMD_USE_TUNING_CONTROLLER    6
#define CMD_USE_PICKER_CONTROLLER    7

#define CMD_CRAZYFLY_TAKE_OFF        11
#define CMD_CRAZYFLY_LAND            12
#define CMD_CRAZYFLY_MOTORS_OFF      13

// Flying States
#define STATE_MOTORS_OFF 1
#define STATE_TAKE_OFF   2
#define STATE_FLYING     3
#define STATE_LAND       4

// Battery states
#define BATTERY_STATE_NORMAL 0
#define BATTERY_STATE_LOW    1

// Battery label image index
#define BATTERY_LABEL_IMAGE_INDEX_EMPTY     0
#define BATTERY_LABEL_IMAGE_INDEX_20        1
#define BATTERY_LABEL_IMAGE_INDEX_40        2
#define BATTERY_LABEL_IMAGE_INDEX_60        3
#define BATTERY_LABEL_IMAGE_INDEX_80        4
#define BATTERY_LABEL_IMAGE_INDEX_FULL      5
#define BATTERY_LABEL_IMAGE_INDEX_UNKNOWN   6

// For which controller parameters to load
#define LOAD_YAML_SAFE_CONTROLLER_AGENT           1
#define LOAD_YAML_DEMO_CONTROLLER_AGENT           2
#define LOAD_YAML_STUDENT_CONTROLLER_AGENT        3
#define LOAD_YAML_MPC_CONTROLLER_AGENT            4
#define LOAD_YAML_REMOTE_CONTROLLER_AGENT         5
#define LOAD_YAML_TUNING_CONTROLLER_AGENT         6
#define LOAD_YAML_PICKER_CONTROLLER_AGENT         7

#define LOAD_YAML_SAFE_CONTROLLER_COORDINATOR     11
#define LOAD_YAML_DEMO_CONTROLLER_COORDINATOR     12
#define LOAD_YAML_STUDENT_CONTROLLER_COORDINATOR  13
#define LOAD_YAML_MPC_CONTROLLER_COORDINATOR      14
#define LOAD_YAML_REMOTE_CONTROLLER_COORDINATOR   15
#define LOAD_YAML_TUNING_CONTROLLER_COORDINATOR   16
#define LOAD_YAML_PICKER_CONTROLLER_COORDINATOR   17


// FOR WHICH BUTTON WAS PRESSED IN THE PICKER CONTOLLER
#define PICKER_BUTTON_GOTOSTART     1
#define PICKER_BUTTON_ATTACH        2
#define PICKER_BUTTON_PICKUP        3
#define PICKER_BUTTON_GOTOEND       4
#define PICKER_BUTTON_PUTDOWN       5
#define PICKER_BUTTON_SQUAT         6
#define PICKER_BUTTON_JUMP          7

#define PICKER_BUTTON_1             11
#define PICKER_BUTTON_2             12
#define PICKER_BUTTON_3             13
#define PICKER_BUTTON_4             14


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

    // # RF Crazyradio Connect Disconnect
    void on_RF_Connect_button_clicked();
    void on_RF_disconnect_button_clicked();

    // # Take off, lanf, motors off
    void on_take_off_button_clicked();
    void on_land_button_clicked();
    void on_motors_OFF_button_clicked();

    // # Setpoint
    void on_set_setpoint_button_safe_clicked();
    void on_set_setpoint_button_demo_clicked();
    void on_set_setpoint_button_student_clicked();
    void on_set_setpoint_button_mpc_clicked();

    // # Load Yaml when acting as the GUI for an Agent
    void on_load_safe_yaml_button_clicked();
    void on_load_demo_yaml_button_clicked();
    void on_load_student_yaml_button_clicked();
    void on_load_mpc_yaml_button_clicked();
    void on_load_remote_yaml_button_clicked();
    void on_load_tuning_yaml_button_clicked();
    void on_load_picker_yaml_button_clicked();

    // # Enable controllers
    void on_enable_safe_controller_clicked();
    void on_enable_demo_controller_clicked();
    void on_enable_student_controller_clicked();
    void on_enable_mpc_controller_clicked();
    void on_enable_remote_controller_clicked();
    void on_enable_tuning_controller_clicked();
    void on_enable_picker_controller_clicked();

    

    void on_demoButton_1_clicked();
    void on_demoButton_2_clicked();
    void on_demoButton_3_clicked();

    void on_studentButton_1_clicked();
    void on_studentButton_2_clicked();
    void on_studentButton_3_clicked();

    // Buttons within the REMOTE controller tab
    void on_remote_subscribe_button_clicked();
    void on_remote_unsubscribe_button_clicked();
    void on_remote_activate_button_clicked();
    void on_remote_deactivate_button_clicked();

    // Buttons within the TUNING controller tab
    void on_tuning_test_horizontal_button_clicked();
    void on_tuning_test_vertical_button_clicked();
    void on_tuning_test_heading_button_clicked();
    void on_tuning_test_all_button_clicked();
    void on_tuning_test_circle_button_clicked();
    void on_tuning_slider_horizontal_valueChanged(int value);
    void on_tuning_slider_vertical_valueChanged(int value);
    void on_tuning_slider_heading_valueChanged(int value);

    // Interations with the PICKER controller tab
    // > For the buttons
    void on_picker_gotostart_button_clicked();
    void on_picker_attach_button_clicked();
    void on_picker_pickup_button_clicked();
    void on_picker_gotoend_button_clicked();
    void on_picker_putdown_button_clicked();
    void on_picker_squat_button_clicked();
    void on_picker_jump_button_clicked();
    void on_picker_1_button_clicked();
    void on_picker_2_button_clicked();
    void on_picker_3_button_clicked();
    void on_picker_4_button_clicked();
    // > For the sliders
    void on_picker_x_slider_valueChanged(int value);
    void on_picker_y_slider_valueChanged(int value);
    void on_picker_z_slider_valueChanged(int value);
    void on_picker_mass_slider_valueChanged(int value);
    // > For the dial
    void on_picker_yaw_dial_valueChanged(int value);





private:
    Ui::MainWindow *ui;

    QShortcut* m_close_GUI_shortcut;

    rosNodeThread* m_rosNodeThread;
    int m_radio_status;
    float m_battery_voltage;
    int m_battery_level;

    std::string m_ros_namespace;

    ros::Timer m_timer_yaml_file_for_safe_controller;
    ros::Timer m_timer_yaml_file_for_demo_controller;
    ros::Timer m_timer_yaml_file_for_student_controller;
    ros::Timer m_timer_yaml_file_for_mpc_controller;
    ros::Timer m_timer_yaml_file_for_remote_controller;
    ros::Timer m_timer_yaml_file_for_tuning_controller;
    ros::Timer m_timer_yaml_file_for_picker_controller;


    int m_student_id;
    CrazyflieContext m_context;

    Setpoint m_safe_setpoint;
    Setpoint m_demo_setpoint;
    Setpoint m_student_setpoint;
    Setpoint m_mpc_setpoint;
    Setpoint m_picker_setpoint;

    int m_flying_state;
    QMutex m_flying_state_mutex;
    QMutex voltage_field_mutex;
    QMutex battery_status_label_mutex;
    QMutex rf_status_label_mutex;
    int m_battery_state;
    // BATTERY EMPTY VOLTAGES (THESE SHOULD BE READ IN AS PARAMTERS)
    //const std::vector<float> m_cutoff_voltages {3.1966,        3.2711,        3.3061,        3.3229,        3.3423,        3.3592,        3.3694,        3.385,        3.4006,        3.4044,        3.4228,        3.4228,        3.4301,        3.4445,        3.4531,        3.4677,        3.4705,        3.4712,        3.4756,        3.483,        3.4944,        3.5008,        3.5008,        3.5084,        3.511,        3.5122,        3.5243,        3.5329,        3.5412,        3.5529,        3.5609,        3.5625,        3.5638,        3.5848,        3.6016,        3.6089,        3.6223,        3.628,        3.6299,        3.6436,        3.6649,        3.6878,        3.6983,        3.7171,        3.7231,        3.7464,        3.7664,        3.7938,        3.8008,        3.816,        3.8313,        3.8482,        3.866,        3.8857,        3.8984,        3.9159,        3.9302,        3.9691,        3.997,        4.14    };
    const float battery_voltage_empty_while_flying      =  2.80;   // in Volts
    const float battery_voltage_empty_while_motors_off  =  3.30;  // in Volts
    // BATTERY FULL VOLTAGES
    const float battery_voltage_full_while_flying      =  3.70;   // in Volts
    const float battery_voltage_full_while_motors_off  =  4.20;   // in Volts

    int m_battery_label_image_current_index;

    ros::Publisher crazyRadioCommandPublisher;
    ros::Subscriber crazyRadioStatusSubscriber;
    ros::Publisher FlyingAgentClientCommandPublisher;
    ros::Subscriber CFBatterySubscriber;
    ros::Subscriber flyingStateSubscriber;
    ros::Subscriber batteryStateSubscriber;

    ros::Publisher controllerSetpointPublisher;
    ros::Subscriber safeSetpointSubscriber;

    // SUBSCRIBERS AND PUBLISHERS:
    // > For the Demo Controller SETPOINTS
    ros::Publisher  demoSetpointPublisher;
    ros::Subscriber demoSetpointSubscriber;
    // > For the Student Controller SETPOINTS
    ros::Publisher  studentSetpointPublisher;
    ros::Subscriber studentSetpointSubscriber;
    // > For the MPC Controller SETPOINTS
    ros::Publisher  mpcSetpointPublisher;
    ros::Subscriber mpcSetpointSubscriber;

    // > For the Remote Controller subscribe action
    ros::Publisher remoteSubscribePublisher;
    // > For the Remote Controller activate action
    ros::Publisher remoteActivatePublisher;
    // > For the Remote Controller data
    ros::Subscriber remoteDataSubscriber;
    // > For the Remote Control setpoint
    ros::Subscriber remoteControlSetpointSubscriber;

    // > For the TUNING CONTROLLER "test" button publisher
    ros::Publisher tuningActivateTestPublisher;
    // > For the TUNING CONTOLLER "gain" sliders
    ros::Publisher tuningHorizontalGainPublisher;
    ros::Publisher tuningVerticalGainPublisher;
    ros::Publisher tuningHeadingGainPublisher;



	// > For the PICKER CONTROLLER
	ros::Publisher  pickerButtonPressedPublisher;
	ros::Publisher  pickerZSetpointPublisher;
	ros::Publisher  pickerYawSetpointPublisher;
	ros::Publisher  pickerMassPublisher;
	ros::Publisher  pickerXAdjustmentPublisher;
	ros::Publisher  pickerYAdjustmentPublisher;
	ros::Publisher  pickerSetpointPublisher;
	ros::Subscriber pickerSetpointSubscriber;
    ros::Subscriber pickerSetpointToGUISubscriber;

    ros::Publisher  pickerButtonPressedWithSetpointPublisher;

    bool shouldSendWithSetpoint_for_pickerButtons = true;




    ros::Publisher demoCustomButtonPublisher;
    ros::Publisher studentCustomButtonPublisher;

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
    void demoSetpointCallback(const Setpoint& newSetpoint);
    void studentSetpointCallback(const Setpoint& newSetpoint);
    void mpcSetpointCallback(const Setpoint& newSetpoint);
    void pickerSetpointCallback(const Setpoint& newSetpoint);


    void remoteDataCallback(const CrazyflieData& objectData);
    void remoteControlSetpointCallback(const CrazyflieData& setpointData);


    // > For actually sending the button message
    void send_picker_button_clicked_message(int button_index);
    void send_picker_button_clicked_message_with_setpoint(const SetpointV2& setpointV2_to_send);
    

    void DBChangedCallback(const std_msgs::Int32& msg);

    // # Load Yaml when acting as the GUI for an Agent
    void safeYamlFileTimerCallback(const ros::TimerEvent&);
    void demoYamlFileTimerCallback(const ros::TimerEvent&);
    void studentYamlFileTimerCallback(const ros::TimerEvent&);
    void mpcYamlFileTimerCallback(const ros::TimerEvent&);
    void remoteYamlFileTimerCallback(const ros::TimerEvent&);
    void tuningYamlFileTimerCallback(const ros::TimerEvent&);
    void pickerYamlFileTimerCallback(const ros::TimerEvent&);
    


    void requestLoadControllerYaml_from_my_GUI_Callback(const std_msgs::Int32& msg);
    void controllerUsedChangedCallback(const std_msgs::Int32& msg);
    void batteryStateChangedCallback(const std_msgs::Int32& msg);

    float fromVoltageToPercent(float voltage);
    void updateBatteryVoltage(float battery_voltage);
    void setCrazyRadioStatus(int radio_status);
    void loadCrazyflieContext();
    void coordinatesToLocal(CrazyflieData& cf);

    void initialize_demo_setpoint();
    void initialize_student_setpoint();
    void initialize_mpc_setpoint();
    void initialize_picker_setpoint();


    void disableGUI();
    void enableGUI();
    void highlightSafeControllerTab();
    void highlightDemoControllerTab();
    void highlightStudentControllerTab();
    void highlightMpcControllerTab();
    void highlightRemoteControllerTab();
    void highlightTuningControllerTab();
    void highlightPickerControllerTab();

    bool setpointInsideBox(Setpoint setpoint, CrazyflieContext context);
    Setpoint correctSetpointBox(Setpoint setpoint, CrazyflieContext context);

};

#endif // MAINWINDOW_H
