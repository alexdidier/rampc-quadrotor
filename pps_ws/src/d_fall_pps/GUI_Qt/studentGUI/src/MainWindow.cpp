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


#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <string>
#include <QShortcut>

#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

#include "d_fall_pps/CMQuery.h"

#include "d_fall_pps/ViconData.h"

#include "d_fall_pps/CustomButton.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_battery_level(0)
{

    ui->setupUi(this);

    m_rosNodeThread = new rosNodeThread(argc, argv, "student_GUI");
    m_rosNodeThread->init();

    setCrazyRadioStatus(DISCONNECTED);

    m_ros_namespace = ros::this_node::getNamespace();
    ROS_INFO("[Student GUI] node namespace: %s", m_ros_namespace.c_str());

    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    ros::NodeHandle nodeHandle(m_ros_namespace);


    // SUBSCRIBERS AND PUBLISHERS:
    // > For the Demo Controller SETPOINTS and CUSTOM COMMANDS
    demoSetpointPublisher     = nodeHandle.advertise<Setpoint>("DemoControllerService/Setpoint", 1);
    demoSetpointSubscriber    = nodeHandle.subscribe("DemoControllerService/Setpoint", 1, &MainWindow::demoSetpointCallback, this);
    demoCustomButtonPublisher = nodeHandle.advertise<CustomButton>("DemoControllerService/GUIButton", 1);
    // > For the Student Controller SETPOINTS and CUSTOM COMMANDS
    studentSetpointPublisher     = nodeHandle.advertise<Setpoint>("StudentControllerService/Setpoint", 1);
    studentSetpointSubscriber    = nodeHandle.subscribe("StudentControllerService/Setpoint", 1, &MainWindow::studentSetpointCallback, this);
    studentCustomButtonPublisher = nodeHandle.advertise<CustomButton>("StudentControllerService/GUIButton", 1);
    // > For the MPC Controller SETPOINTS
    mpcSetpointPublisher  = nodeHandle.advertise<Setpoint>("MpcControllerService/Setpoint", 1);
    mpcSetpointSubscriber = nodeHandle.subscribe("MpcControllerService/Setpoint", 1, &MainWindow::mpcSetpointCallback, this);


    // > For the Remote Controller subscribe action
    remoteSubscribePublisher = nodeHandle.advertise<ViconSubscribeObjectName>("RemoteControllerService/ViconSubscribeObjectName", 1);
    // > For the Remote Controller activate action
    remoteActivatePublisher = nodeHandle.advertise<std_msgs::Int32>("RemoteControllerService/Activate", 1);
    // > For the Remote Controller data
    remoteDataSubscriber = nodeHandle.subscribe("RemoteControllerService/RemoteData", 1, &MainWindow::remoteDataCallback, this);;
    // > For the Remote Controller data
    remoteControlSetpointSubscriber = nodeHandle.subscribe("RemoteControllerService/RemoteControlSetpoint", 1, &MainWindow::remoteControlSetpointCallback, this);;


    // > For the TUNING CONTROLLER "test" button publisher
    tuningActivateTestPublisher = nodeHandle.advertise<std_msgs::Int32>("TuningControllerService/ActivateTest", 1);
    // > For the TUNING CONTOLLER "gain" sliders
    tuningHorizontalGainPublisher = nodeHandle.advertise<std_msgs::Int32>("TuningControllerService/HorizontalGain", 1);
    tuningVerticalGainPublisher = nodeHandle.advertise<std_msgs::Int32>("TuningControllerService/VerticalGain", 1);
    tuningHeadingGainPublisher = nodeHandle.advertise<std_msgs::Int32>("TuningControllerService/HeadingGain", 1);



    // subscribers
    crazyRadioStatusSubscriber = nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &MainWindow::crazyRadioStatusCallback, this);

    CFBatterySubscriber = nodeHandle.subscribe("CrazyRadio/CFBattery", 1, &MainWindow::CFBatteryCallback, this);

    flyingStateSubscriber = nodeHandle.subscribe("PPSClient/flyingState", 1, &MainWindow::flyingStateChangedCallback, this);

    batteryStateSubscriber = nodeHandle.subscribe("PPSClient/batteryState", 1, &MainWindow::batteryStateChangedCallback, this);

    controllerUsedSubscriber = nodeHandle.subscribe("PPSClient/controllerUsed", 1, &MainWindow::controllerUsedChangedCallback, this);


    safeSetpointSubscriber = nodeHandle.subscribe("SafeControllerService/Setpoint", 1, &MainWindow::safeSetpointCallback, this);
    DBChangedSubscriber = nodeHandle.subscribe("/my_GUI/DBChanged", 1, &MainWindow::DBChangedCallback, this);

    ros::NodeHandle my_nodeHandle("~");
    controllerSetpointPublisher = my_nodeHandle.advertise<Setpoint>("ControllerSetpoint", 1);


    // communication with PPS Client, just to make it possible to communicate through terminal also we use PPSClient's name
    //ros::NodeHandle nh_PPSClient(m_ros_namespace + "/PPSClient");
    ros::NodeHandle nh_PPSClient("PPSClient");
    crazyRadioCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("crazyRadioCommand", 1);
    PPSClientCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("Command", 1);    


    // > For publishing a message that requests the
    //   YAML parameters to be re-loaded from file
    // > The message contents specify which controller
    //   the parameters should be re-loaded for
    requestLoadControllerYamlPublisher = nh_PPSClient.advertise<std_msgs::Int32>("requestLoadControllerYaml", 1);


    // Subscriber for locking the load the controller YAML
    // parameters when the Coordintor GUI requests a load
    requestLoadControllerYaml_from_my_GUI_Subscriber = nodeHandle.subscribe("/my_GUI/requestLoadControllerYaml", 1, &MainWindow::requestLoadControllerYaml_from_my_GUI_Callback, this);

    // First get student ID
    if(!nh_PPSClient.getParam("agentID", m_student_id))
    {
		ROS_ERROR("Failed to get agentID");
	}

    // Then, Central manager
    centralManager = nodeHandle.serviceClient<CMQuery>("/CentralManagerService/Query", false);
    loadCrazyflieContext();




    // Load default setpoint from the "SafeController" namespace of the "ParameterService"
    std::vector<float> default_setpoint(4);
    ros::NodeHandle nodeHandle_to_own_agent_parameter_service("ParameterService");
    ros::NodeHandle nodeHandle_for_safeController(nodeHandle_to_own_agent_parameter_service, "SafeController");

    if(!nodeHandle_for_safeController.getParam("defaultSetpoint", default_setpoint))
    {
        ROS_ERROR_STREAM("The StudentGUI could not find parameter 'defaultSetpoint', as called from main(...)");
    }

    // Copy the default setpoint into respective text fields of the GUI
    ui->current_setpoint_x_safe->setText(QString::number(default_setpoint[0]));
    ui->current_setpoint_y_safe->setText(QString::number(default_setpoint[1]));
    ui->current_setpoint_z_safe->setText(QString::number(default_setpoint[2]));
    ui->current_setpoint_yaw_safe->setText(QString::number(default_setpoint[3]));


    disableGUI();
    highlightSafeControllerTab();

    // INITIALISE THE BATTERY STATE AS NORMAL
    m_battery_state = BATTERY_STATE_NORMAL;

    // SET THE BATTERY VOLTAGE FIELD TO BE BLANK
    QString qstr = "-.-- V";
    ui->voltage_field->setText(qstr);
    // SET THE IMAGE FOR THE BATTERY STATUS LABEL
    QPixmap battery_unknown_pixmap(":/images/battery_unknown.png");
    ui->battery_status_label->setPixmap(battery_unknown_pixmap);
    ui->battery_status_label->setScaledContents(true);
    m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;

    // SET THE IMAGE FOR THE CRAZY RADIO STATUS LABEL
    QPixmap rf_disconnected_pixmap(":/images/rf_disconnected.png");
    ui->rf_status_label->setPixmap(rf_disconnected_pixmap);
    ui->rf_status_label->setScaledContents(true);

    // SET THE IMAGE FOR THE FLYING STATE LABEL
    QPixmap flying_state_off_pixmap(":/images/flying_state_off.png");
    ui->flying_state_label->setPixmap(flying_state_off_pixmap);
    ui->flying_state_label->setScaledContents(true);



    //QPixmap battery_80_pixmap(":/images/battery_80.png");
	//m_battery_80_pixmap = battery_80_pixmap.scaled(50,70,Qt::IgnoreAspectRatio);
	// The syntax for "scaled" is (int width, int height, ...)


    ui->error_label->setStyleSheet("QLabel { color : red; }");
    ui->error_label->clear();

    // Add keyboard shortcuts
    // > for "all motors off", press the space bar
    ui->motors_OFF_button->setShortcut(tr("Space"));
    // > for "kill GUI node", press "CTRL+C" while the GUI window is the focus
    QShortcut* close_GUI_shortcut = new QShortcut(QKeySequence(tr("CTRL+C")), this, SLOT(close()));


    initialize_demo_setpoint();
    initialize_student_setpoint();
    initialize_mpc_setpoint();
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disableGUI()
{
    ui->motors_OFF_button->setEnabled(false);
    ui->take_off_button->setEnabled(false);
    ui->land_button->setEnabled(false);
}

void MainWindow::enableGUI()
{
    ui->motors_OFF_button->setEnabled(true);
    if(m_battery_state == BATTERY_STATE_NORMAL)
    {
        ui->take_off_button->setEnabled(true);
        ui->land_button->setEnabled(true);
    }
}

void MainWindow::highlightSafeControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::green);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(2, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(3, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(4, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(5, Qt::black);
}
void MainWindow::highlightDemoControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::green);
    ui->tabWidget->tabBar()->setTabTextColor(2, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(3, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(4, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(5, Qt::black);
}
void MainWindow::highlightStudentControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(2, Qt::green);
    ui->tabWidget->tabBar()->setTabTextColor(3, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(4, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(5, Qt::black);
}
void MainWindow::highlightMpcControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(2, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(3, Qt::green);
    ui->tabWidget->tabBar()->setTabTextColor(4, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(5, Qt::black);
}
void MainWindow::highlightRemoteControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(2, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(3, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(4, Qt::green);
    ui->tabWidget->tabBar()->setTabTextColor(5, Qt::black);
}
void MainWindow::highlightTuningControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(2, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(3, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(4, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(5, Qt::green);
}

void MainWindow::DBChangedCallback(const std_msgs::Int32& msg)
{
    loadCrazyflieContext();
    ROS_INFO("context reloaded in student_GUI");
}

void MainWindow::controllerUsedChangedCallback(const std_msgs::Int32& msg)
{
    switch(msg.data)
    {
        case SAFE_CONTROLLER:
            highlightSafeControllerTab();
            break;
        case DEMO_CONTROLLER:
            highlightDemoControllerTab();
            break;
        case STUDENT_CONTROLLER:
            highlightStudentControllerTab();
            break;
        case MPC_CONTROLLER:
            highlightMpcControllerTab();
            break;
        case REMOTE_CONTROLLER:
            highlightRemoteControllerTab();
            break;
        case TUNING_CONTROLLER:
            highlightTuningControllerTab();
            break;
        default:
            break;
    }
}

void MainWindow::safeSetpointCallback(const Setpoint& newSetpoint)
{
    m_safe_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x_safe->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y_safe->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z_safe->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw_safe->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
}

void MainWindow::demoSetpointCallback(const Setpoint& newSetpoint)
{
    m_demo_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x_demo->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y_demo->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z_demo->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw_demo->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
}

void MainWindow::studentSetpointCallback(const Setpoint& newSetpoint)
{
    m_student_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x_student->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y_student->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z_student->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw_student->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
}

void MainWindow::mpcSetpointCallback(const Setpoint& newSetpoint)
{
    m_mpc_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x_mpc->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y_mpc->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z_mpc->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw_mpc->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
}

void MainWindow::flyingStateChangedCallback(const std_msgs::Int32& msg)
{
	// PUT THE CURRENT STATE INTO THE CLASS VARIABLE
	m_flying_state_mutex.lock();
	m_flying_state = msg.data;
	m_flying_state_mutex.unlock();

	// UPDATE THE LABEL TO DISPLAY THE FLYING STATE
    //QString qstr = "Flying State: ";
    switch(msg.data)
    {
        case STATE_MOTORS_OFF:
        {
            //qstr.append("Motors OFF");
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            ui->flying_state_label->clear();
            QPixmap flying_state_off_pixmap(":/images/flying_state_off.png");
            ui->flying_state_label->setPixmap(flying_state_off_pixmap);
            ui->flying_state_label->setScaledContents(true);
            ui->flying_state_label->update();
            break;
        }

        case STATE_TAKE_OFF:
        {
            //qstr.append("Take OFF");
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            ui->flying_state_label->clear();
            QPixmap flying_state_enabling_pixmap(":/images/flying_state_enabling.png");
            ui->flying_state_label->setPixmap(flying_state_enabling_pixmap);
            ui->flying_state_label->setScaledContents(true);
            ui->flying_state_label->update();
            break;
        }

        case STATE_FLYING:
        {
            //qstr.append("Flying");
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            ui->flying_state_label->clear();
            QPixmap flying_state_flying_pixmap(":/images/flying_state_flying.png");
            ui->flying_state_label->setPixmap(flying_state_flying_pixmap);
            ui->flying_state_label->setScaledContents(true);
            ui->flying_state_label->update();
            break;
        }

        case STATE_LAND:
        {
            //qstr.append("Land");
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            ui->flying_state_label->clear();
            QPixmap flying_state_disabling_pixmap(":/images/flying_state_disabling.png");
            ui->flying_state_label->setPixmap(flying_state_disabling_pixmap);
            ui->flying_state_label->setScaledContents(true);
            ui->flying_state_label->update();
            break;
        }

        default:
        {
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            ui->flying_state_label->clear();
            QPixmap flying_state_unknown_pixmap(":/images/flying_state_unknown.png");
            ui->flying_state_label->setPixmap(flying_state_unknown_pixmap);
            ui->flying_state_label->setScaledContents(true);
            ui->flying_state_label->update();
            break;
        }
    }
    //ui->flying_state_label->setText(qstr);
}

void MainWindow::batteryStateChangedCallback(const std_msgs::Int32& msg)
{
    // switch case with unabling buttons motors off, take off, etc... when battery is low
    QString qstr = "";
    switch(msg.data)
    {
        case BATTERY_STATE_LOW:
        {
            // DISABLE THE TAKE OFF AND LAND BUTTONS
            ui->take_off_button->setEnabled(false);
            ui->land_button->setEnabled(false);
            // ui->groupBox_4->setEnabled(false);

			// SET THE CLASS VARIABLE FOR TRACKING THE BATTERY STATE
            m_battery_state = BATTERY_STATE_LOW;
            break;
        }

        case BATTERY_STATE_NORMAL:
        {
            // ui->groupBox_4->setEnabled(true);
            ui->take_off_button->setEnabled(true);
            ui->land_button->setEnabled(true);

            // SET THE CLASS VARIABLE FOR TRACKING THE BATTERY STATE
            m_battery_state = BATTERY_STATE_NORMAL;
            break;
        }

        default:
            break;
    }
}


void MainWindow::setCrazyRadioStatus(int radio_status)
{
    // add more things whenever the status is changed
    switch(radio_status)
    {
        case CONNECTED:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIOSTATUS LABEL
            rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_connected_pixmap(":/images/rf_connected.png");
            ui->rf_status_label->setPixmap(rf_connected_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            rf_status_label_mutex.unlock();
            // ENABLE THE REMAINDER OF THE GUI
            enableGUI();
            break;
        }

        case CONNECTING:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIO STATUS LABEL
            rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_connecting_pixmap(":/images/rf_connecting.png");
            ui->rf_status_label->setPixmap(rf_connecting_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            rf_status_label_mutex.unlock();
            break;
        }

        case DISCONNECTED:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIO STATUS LABEL
            rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_disconnected_pixmap(":/images/rf_disconnected.png");
            ui->rf_status_label->setPixmap(rf_disconnected_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            rf_status_label_mutex.unlock();
            // SET THE BATTERY VOLTAGE FIELD TO BE BLANK
            QString qstr = "-.-- V";
            voltage_field_mutex.lock();
            ui->voltage_field->setText(qstr);
            voltage_field_mutex.unlock();
            // SET THE APPROPRIATE IMAGE FOR THE BATTERY STATUS LABEL
            if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_UNKNOWN)
			{
				battery_status_label_mutex.lock();
				ui->battery_status_label->clear();
	            QPixmap battery_unknown_pixmap(":/images/battery_unknown.png");
	            ui->battery_status_label->setPixmap(battery_unknown_pixmap);
	            ui->battery_status_label->setScaledContents(true);
	            m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
	            ui->battery_status_label->update();
	            battery_status_label_mutex.unlock();
	        }
            // DISABLE THE REMAINDER OF THE GUI
            disableGUI();
            break;
        }

        default:
        {
    		ROS_ERROR_STREAM("unexpected radio status: " << m_radio_status);
            break;
        }
    }
    this->m_radio_status = radio_status;
}




float MainWindow::fromVoltageToPercent(float voltage)
{
	// INITIALISE THE LOCAL VARIABLE FOR THE VOLTAGE WHEN FULL/EMPTY
	float voltage_when_full;
	float voltage_when_empty;

	// COMPUTE THE PERCENTAGE DIFFERENTLY DEPENDING ON
	// THE CURRENT FLYING STATE
	m_flying_state_mutex.lock();
	if (m_flying_state == STATE_MOTORS_OFF)
	{
		voltage_when_empty = battery_voltage_empty_while_motors_off;
		voltage_when_full  = battery_voltage_full_while_motors_off;
	}
	else
	{
		voltage_when_empty = battery_voltage_empty_while_flying;
		voltage_when_full  = battery_voltage_full_while_flying;
	}
	m_flying_state_mutex.unlock();
	//voltage_when_empty = battery_voltage_empty_while_motors_off;
	//voltage_when_full  = battery_voltage_full_while_motors_off;

	// COMPUTE THE PERCENTAGE
	float percentage = 100.0f * (voltage-voltage_when_empty)/(voltage_when_full-voltage_when_empty);


	// CLIP THE PERCENTAGE TO BE BETWEEN [0,100]
    // > This should not happen to often
    if(percentage > 100.0f)
    {
        percentage = 100.0f;
    }
    if(percentage < 0.0f)
    {
        percentage = 0.0f;
    }

    return percentage;
}

void MainWindow::updateBatteryVoltage(float battery_voltage)
{
	// PUT THE VOLTAGE INTO THE CLASS VARIABLES
    m_battery_voltage = battery_voltage;

    // UPDATE THE BATTERY VOLTAGE FIELD
    voltage_field_mutex.lock();
    QString qstr = "";
    qstr.append(QString::number(battery_voltage, 'f', 2));
    qstr.append(" V");
    ui->voltage_field->setText(qstr);
    voltage_field_mutex.unlock();

	// COMPUTE THE BATTERY VOLTAGE AS A PERCENTAGE
	float battery_voltage_percentage = fromVoltageToPercent(battery_voltage);

	//ROS_INFO_STREAM("Battery percentage = " << battery_voltage_percentage );

	// UPDATE THE IMAGE DISPLAYED IN THE BATTERY VOLTAGE LABEL IMAGE
	battery_status_label_mutex.lock();
	switch(m_battery_state)
	{
		// WHEN THE BATTERY IS IN A LOW STATE
		case BATTERY_STATE_LOW:
        {
			// SET THE IMAGE FOR THE BATTERY STATUS LABEL
			if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_EMPTY)
			{
				ui->battery_status_label->clear();
				QPixmap battery_empty_pixmap(":/images/battery_empty.png");
				ui->battery_status_label->setPixmap(battery_empty_pixmap);
				ui->battery_status_label->setScaledContents(true);
				m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_EMPTY;
				ui->battery_status_label->update();
			}
			break;
        }

		// WHEN THE BATTERY IS IN A NORMAL STATE
		case BATTERY_STATE_NORMAL:
        {

			if (
				((m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_EMPTY) && (battery_voltage_percentage <= 0.0f))
				||
				((m_battery_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_EMPTY) && (battery_voltage_percentage <= 2.0f))
			)
			{
				if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_EMPTY)
				{
					// SET THE IMAGE FOR THE BATTERY STATUS LABEL
					ui->battery_status_label->clear();
					QPixmap battery_empty_pixmap(":/images/battery_empty.png");
					ui->battery_status_label->setPixmap(battery_empty_pixmap);
					ui->battery_status_label->setScaledContents(true);
					m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_EMPTY;
					ui->battery_status_label->update();
				}
			}
			else if (
				((m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_20) && (battery_voltage_percentage <= 20.0f))
				||
				((m_battery_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_20) && (battery_voltage_percentage <= 22.0f))
			)
			{
				if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_20)
				{
					// SET THE IMAGE FOR THE BATTERY STATUS LABEL
					ui->battery_status_label->clear();
					QPixmap battery_20_pixmap(":/images/battery_20.png");
					ui->battery_status_label->setPixmap(battery_20_pixmap);
					ui->battery_status_label->setScaledContents(true);
					m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_20;
					ui->battery_status_label->update();
				}
			}
			else if (
				((m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_40) && (battery_voltage_percentage <= 40.0f))
				||
				((m_battery_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_40) && (battery_voltage_percentage <= 42.0f))
			)
			{
				if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_40)
				{
					// SET THE IMAGE FOR THE BATTERY STATUS LABEL
					ui->battery_status_label->clear();
					QPixmap battery_40_pixmap(":/images/battery_40.png");
					ui->battery_status_label->setPixmap(battery_40_pixmap);
					ui->battery_status_label->setScaledContents(true);
					m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_40;
					ui->battery_status_label->update();
				}
			}
			else if (
				((m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_60) && (battery_voltage_percentage <= 60.0f))
				||
				((m_battery_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_60) && (battery_voltage_percentage <= 62.0f))
			)
			{
				if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_60)
				{
					// SET THE IMAGE FOR THE BATTERY STATUS LABEL
					ui->battery_status_label->clear();
					QPixmap battery_60_pixmap(":/images/battery_60.png");
					ui->battery_status_label->setPixmap(battery_60_pixmap);
					ui->battery_status_label->setScaledContents(true);
					m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_60;
					ui->battery_status_label->update();
				}
			}
			else if (
				((m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_80) && (battery_voltage_percentage <= 80.0f))
				||
				((m_battery_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_80) && (battery_voltage_percentage <= 82.0f))
			)
			{
				if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_80)
				{
					// SET THE IMAGE FOR THE BATTERY STATUS LABEL
					ui->battery_status_label->clear();
					QPixmap battery_80_pixmap(":/images/battery_80.png");
					ui->battery_status_label->setPixmap(battery_80_pixmap);
					ui->battery_status_label->setScaledContents(true);
					m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_80;
					ui->battery_status_label->update();
				}
			}
			else
			{
				if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_FULL)
				{
					// SET THE IMAGE FOR THE BATTERY STATUS LABEL
					ui->battery_status_label->clear();
					QPixmap battery_full_pixmap(":/images/battery_full.png");
					ui->battery_status_label->setPixmap(battery_full_pixmap);
					ui->battery_status_label->setScaledContents(true);
					m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_FULL;
					ui->battery_status_label->update();
				}
			}
			break;
        }

		default:
        {
        	if (m_battery_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_UNKNOWN)
			{
	            // SET THE IMAGE FOR THE BATTERY STATUS LABEL
	            ui->battery_status_label->clear();
	            QPixmap battery_unknown_pixmap(":/images/battery_unknown.png");
	            ui->battery_status_label->setPixmap(battery_unknown_pixmap);
	            ui->battery_status_label->setScaledContents(true);
	            m_battery_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
				ui->battery_status_label->update();
	        }
			break;
        }
	}
	battery_status_label_mutex.unlock();
}

void MainWindow::CFBatteryCallback(const std_msgs::Float32& msg)
{
    updateBatteryVoltage(msg.data);
}

void MainWindow::crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("[Student GUI] Callback CrazyRadioStatus called");
    this->setCrazyRadioStatus(msg.data);
}

void MainWindow::loadCrazyflieContext()
{
	CMQuery contextCall;
	contextCall.request.studentID = m_student_id;
	ROS_INFO_STREAM("StudentID:" << m_student_id);

	centralManager.waitForExistence(ros::Duration(-1));

	if(centralManager.call(contextCall))
    {
		m_context = contextCall.response.crazyflieContext;
		ROS_INFO_STREAM("CrazyflieContext:\n" << m_context);
        // we now have the m_context variable with the current context. Put CF Name in label

        QString qstr = "StudentID ";
        qstr.append(QString::number(m_student_id));
        qstr.append(" connected to CF ");
        qstr.append(QString::fromStdString(m_context.crazyflieName));
        ui->groupBox->setTitle(qstr);
	}
    else
    {
		ROS_ERROR("Failed to load context");
	}

	ros::NodeHandle nh("CrazyRadio");
	nh.setParam("crazyFlieAddress", m_context.crazyflieAddress);
}

void MainWindow::coordinatesToLocal(CrazyflieData& cf)
{
	AreaBounds area = m_context.localArea;
	float originX = (area.xmin + area.xmax) / 2.0;
	float originY = (area.ymin + area.ymax) / 2.0;
    // change Z origin to zero, i.e., to the table height, zero of global coordinates, instead of middle of the box
    float originZ = 0.0;
	// float originZ = (area.zmin + area.zmax) / 2.0;

	cf.x -= originX;
	cf.y -= originY;
	cf.z -= originZ;
}


void MainWindow::updateNewViconData(const ptrToMessage& p_msg) //connected to newViconData, from node
{
    for(std::vector<CrazyflieData>::const_iterator it = p_msg->crazyflies.begin(); it != p_msg->crazyflies.end(); ++it)
    {
		CrazyflieData global = *it;
        if(global.crazyflieName == m_context.crazyflieName)
        {
            CrazyflieData local = global;
            coordinatesToLocal(local);

            // now we have the local coordinates, put them in the labels
            ui->current_x_safe->setText(QString::number(local.x, 'f', 3));
            ui->current_y_safe->setText(QString::number(local.y, 'f', 3));
            ui->current_z_safe->setText(QString::number(local.z, 'f', 3));
            ui->current_yaw_safe->setText(QString::number(local.yaw * RAD2DEG, 'f', 1));
            ui->current_pitch_safe->setText(QString::number(local.pitch * RAD2DEG, 'f', 1));
            ui->current_roll_safe->setText(QString::number(local.roll * RAD2DEG, 'f', 1));

            ui->current_x_demo->setText(QString::number(local.x, 'f', 3));
            ui->current_y_demo->setText(QString::number(local.y, 'f', 3));
            ui->current_z_demo->setText(QString::number(local.z, 'f', 3));
            ui->current_yaw_demo->setText(QString::number(local.yaw * RAD2DEG, 'f', 1));
            ui->current_pitch_demo->setText(QString::number(local.pitch * RAD2DEG, 'f', 1));
            ui->current_roll_demo->setText(QString::number(local.roll * RAD2DEG, 'f', 1));

            // also update diff
            ui->diff_x_safe->setText(QString::number(m_safe_setpoint.x - local.x, 'f', 3));
            ui->diff_y_safe->setText(QString::number(m_safe_setpoint.y - local.y, 'f', 3));
            ui->diff_z_safe->setText(QString::number(m_safe_setpoint.z - local.z, 'f', 3));
            ui->diff_yaw_safe->setText(QString::number((m_safe_setpoint.yaw - local.yaw) * RAD2DEG, 'f', 1));

            ui->diff_x_demo->setText(QString::number(m_demo_setpoint.x - local.x, 'f', 3));
            ui->diff_y_demo->setText(QString::number(m_demo_setpoint.y - local.y, 'f', 3));
            ui->diff_z_demo->setText(QString::number(m_demo_setpoint.z - local.z, 'f', 3));
            ui->diff_yaw_demo->setText(QString::number((m_demo_setpoint.yaw - local.yaw) * RAD2DEG, 'f', 1));
        }
    }
}



//    ----------------------------------------------------------------------------------
// # RF Crazyradio Connect Disconnect
void MainWindow::on_RF_Connect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_RECONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("command reconnect published");
}

void MainWindow::on_RF_disconnect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_DISCONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("command disconnect published");
}



//    ----------------------------------------------------------------------------------
// # Take off, lanf, motors off
void MainWindow::on_take_off_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_CRAZYFLY_TAKE_OFF;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_land_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_CRAZYFLY_LAND;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_motors_OFF_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_CRAZYFLY_MOTORS_OFF;
    this->PPSClientCommandPublisher.publish(msg);
}



//    ----------------------------------------------------------------------------------
// # Setpoint
void MainWindow::on_set_setpoint_button_safe_clicked()
{
    Setpoint msg_setpoint;

    // initialize setpoint to previous one

    msg_setpoint.x = (ui->current_setpoint_x_safe->text()).toFloat();
    msg_setpoint.y = (ui->current_setpoint_y_safe->text()).toFloat();
    msg_setpoint.z = (ui->current_setpoint_z_safe->text()).toFloat();
    msg_setpoint.yaw = (ui->current_setpoint_yaw_safe->text()).toFloat();

    if(!ui->new_setpoint_x_safe->text().isEmpty())
        msg_setpoint.x = (ui->new_setpoint_x_safe->text()).toFloat();

    if(!ui->new_setpoint_y_safe->text().isEmpty())
        msg_setpoint.y = (ui->new_setpoint_y_safe->text()).toFloat();

    if(!ui->new_setpoint_z_safe->text().isEmpty())
        msg_setpoint.z = (ui->new_setpoint_z_safe->text()).toFloat();

    if(!ui->new_setpoint_yaw_safe->text().isEmpty())
        msg_setpoint.yaw = (ui->new_setpoint_yaw_safe->text()).toFloat() * DEG2RAD;


    if(!setpointInsideBox(msg_setpoint, m_context))
    {
        ROS_INFO("Corrected setpoint, was out of bounds");

        // correct the setpoint given the box size
        msg_setpoint = correctSetpointBox(msg_setpoint, m_context);
        ui->error_label->setText("Setpoint is outside safety box");
    }
    else
    {
        ui->error_label->clear();
    }

    this->controllerSetpointPublisher.publish(msg_setpoint);

    ROS_INFO_STREAM("Setpoint change clicked with:" << msg_setpoint.x << ", "<< msg_setpoint.y << ", "<< msg_setpoint.z << ", "<< msg_setpoint.yaw);
}

void MainWindow::initialize_demo_setpoint()
{
    Setpoint msg_setpoint;
    msg_setpoint.x = 0;
    msg_setpoint.y = 0;
    msg_setpoint.z = 0.4;
    msg_setpoint.yaw = 0;

    this->demoSetpointPublisher.publish(msg_setpoint);
}

void MainWindow::initialize_student_setpoint()
{
    Setpoint msg_setpoint;
    msg_setpoint.x = 0;
    msg_setpoint.y = 0;
    msg_setpoint.z = 0.4;
    msg_setpoint.yaw = 0;

    this->studentSetpointPublisher.publish(msg_setpoint);
}

void MainWindow::initialize_mpc_setpoint()
{
    Setpoint msg_setpoint;
    msg_setpoint.x = 0;
    msg_setpoint.y = 0;
    msg_setpoint.z = 0.4;
    msg_setpoint.yaw = 0;

    this->mpcSetpointPublisher.publish(msg_setpoint);
}

void MainWindow::on_set_setpoint_button_demo_clicked()
{
    Setpoint msg_setpoint;

    msg_setpoint.x = (ui->current_setpoint_x_demo->text()).toFloat();
    msg_setpoint.y = (ui->current_setpoint_y_demo->text()).toFloat();
    msg_setpoint.z = (ui->current_setpoint_z_demo->text()).toFloat();
    msg_setpoint.yaw = (ui->current_setpoint_yaw_demo->text()).toFloat();

    if(!ui->new_setpoint_x_demo->text().isEmpty())
        msg_setpoint.x = (ui->new_setpoint_x_demo->text()).toFloat();
    if(!ui->new_setpoint_y_demo->text().isEmpty())
        msg_setpoint.y = (ui->new_setpoint_y_demo->text()).toFloat();
    if(!ui->new_setpoint_z_demo->text().isEmpty())
        msg_setpoint.z = (ui->new_setpoint_z_demo->text()).toFloat();
    if(!ui->new_setpoint_yaw_demo->text().isEmpty())
        msg_setpoint.yaw = (ui->new_setpoint_yaw_demo->text()).toFloat() * DEG2RAD;

    this->demoSetpointPublisher.publish(msg_setpoint);

    ROS_INFO_STREAM("Setpoint change clicked with:" << msg_setpoint.x << ", "<< msg_setpoint.y << ", "<< msg_setpoint.z << ", "<< msg_setpoint.yaw);
}

void MainWindow::on_set_setpoint_button_student_clicked()
{
    Setpoint msg_setpoint;

    msg_setpoint.x = (ui->current_setpoint_x_student->text()).toFloat();
    msg_setpoint.y = (ui->current_setpoint_y_student->text()).toFloat();
    msg_setpoint.z = (ui->current_setpoint_z_student->text()).toFloat();
    msg_setpoint.yaw = (ui->current_setpoint_yaw_student->text()).toFloat();

    if(!ui->new_setpoint_x_student->text().isEmpty())
        msg_setpoint.x = (ui->new_setpoint_x_student->text()).toFloat();
    if(!ui->new_setpoint_y_student->text().isEmpty())
        msg_setpoint.y = (ui->new_setpoint_y_student->text()).toFloat();
    if(!ui->new_setpoint_z_student->text().isEmpty())
        msg_setpoint.z = (ui->new_setpoint_z_student->text()).toFloat();
    if(!ui->new_setpoint_yaw_student->text().isEmpty())
        msg_setpoint.yaw = (ui->new_setpoint_yaw_student->text()).toFloat() * DEG2RAD;

    this->studentSetpointPublisher.publish(msg_setpoint);

    ROS_INFO_STREAM("Setpoint change clicked with:" << msg_setpoint.x << ", "<< msg_setpoint.y << ", "<< msg_setpoint.z << ", "<< msg_setpoint.yaw);
}

void MainWindow::on_set_setpoint_button_mpc_clicked()
{
    Setpoint msg_setpoint;

    msg_setpoint.x = (ui->current_setpoint_x_mpc->text()).toFloat();
    msg_setpoint.y = (ui->current_setpoint_y_mpc->text()).toFloat();
    msg_setpoint.z = (ui->current_setpoint_z_mpc->text()).toFloat();
    msg_setpoint.yaw = (ui->current_setpoint_yaw_mpc->text()).toFloat();

    if(!ui->new_setpoint_x_mpc->text().isEmpty())
        msg_setpoint.x = (ui->new_setpoint_x_mpc->text()).toFloat();
    if(!ui->new_setpoint_y_mpc->text().isEmpty())
        msg_setpoint.y = (ui->new_setpoint_y_mpc->text()).toFloat();
    if(!ui->new_setpoint_z_mpc->text().isEmpty())
        msg_setpoint.z = (ui->new_setpoint_z_mpc->text()).toFloat();
    if(!ui->new_setpoint_yaw_mpc->text().isEmpty())
        msg_setpoint.yaw = (ui->new_setpoint_yaw_mpc->text()).toFloat() * DEG2RAD;

    this->mpcSetpointPublisher.publish(msg_setpoint);

    ROS_INFO_STREAM("Setpoint change clicked with:" << msg_setpoint.x << ", "<< msg_setpoint.y << ", "<< msg_setpoint.z << ", "<< msg_setpoint.yaw);
}


//    ----------------------------------------------------------------------------------
// # Load Yaml when acting as the GUI for an Agent
void MainWindow::on_load_safe_yaml_button_clicked()
{
    // Set the "load safe yaml" button to be disabled
    ui->load_safe_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the safe controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_SAFE_CONTROLLER_AGENT;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("Request load of safe controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_safe_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::safeYamlFileTimerCallback, this, true);
}

void MainWindow::safeYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load safe yaml" button again
    ui->load_safe_yaml_button->setEnabled(true);
}



void MainWindow::on_load_demo_yaml_button_clicked()
{
    // Set the "load demo yaml" button to be disabled
    ui->load_demo_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the demo controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_DEMO_CONTROLLER_AGENT;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("Request load of demo controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_demo_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::demoYamlFileTimerCallback, this, true);
}

void MainWindow::demoYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load demo yaml" button again
    ui->load_demo_yaml_button->setEnabled(true);
}



void MainWindow::on_load_student_yaml_button_clicked()
{
    // Set the "load student yaml" button to be disabled
    ui->load_student_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the student controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_STUDENT_CONTROLLER_AGENT;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("Request load of student controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_student_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::studentYamlFileTimerCallback, this, true);    
}

void MainWindow::studentYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load student yaml" button again
    ui->load_student_yaml_button->setEnabled(true);
}



void MainWindow::on_load_mpc_yaml_button_clicked()
{
    // Set the "load mpc yaml" button to be disabled
    ui->load_mpc_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the mpc controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_MPC_CONTROLLER_AGENT;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("Request load of mpc controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_mpc_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::mpcYamlFileTimerCallback, this, true);
}

void MainWindow::mpcYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load mpc yaml" button again
    ui->load_mpc_yaml_button->setEnabled(true);
}




void MainWindow::on_load_remote_yaml_button_clicked()
{
    // Set the "load remote yaml" button to be disabled
    ui->load_remote_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the remote controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_REMOTE_CONTROLLER_AGENT;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("[STUDENT GUI] Request load of remote controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_remote_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::remoteYamlFileTimerCallback, this, true);
}

void MainWindow::remoteYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load remote yaml" button again
    ui->load_remote_yaml_button->setEnabled(true);
}



void MainWindow::on_load_tuning_yaml_button_clicked()
{
    // Set the "load tuning yaml" button to be disabled
    ui->load_tuning_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the tuning controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_TUNING_CONTROLLER_AGENT;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("[STUDENT GUI] Request load of tuning controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_tuning_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::tuningYamlFileTimerCallback, this, true);
}

void MainWindow::tuningYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load tuning yaml" button again
    ui->load_tuning_yaml_button->setEnabled(true);
}



void MainWindow::requestLoadControllerYaml_from_my_GUI_Callback(const std_msgs::Int32& msg)
{
    // Extract from the "msg" for which controller the YAML
    // parameters should be loaded
    int controller_to_load_yaml = msg.data;

    // Create the "nodeHandle" needed in the switch cases below
    ros::NodeHandle nodeHandle("~");

    // Switch between loading for the different controllers
    switch(controller_to_load_yaml)
    {
        case LOAD_YAML_SAFE_CONTROLLER_AGENT:
        case LOAD_YAML_SAFE_CONTROLLER_COORDINATOR:
            // Set the "load safe yaml" button to be disabled
            ui->load_safe_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            m_timer_yaml_file_for_safe_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::safeYamlFileTimerCallback, this, true);

            break;

        case LOAD_YAML_DEMO_CONTROLLER_AGENT:
        case LOAD_YAML_DEMO_CONTROLLER_COORDINATOR:
            // Set the "load demo yaml" button to be disabled
            ui->load_demo_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            m_timer_yaml_file_for_demo_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::demoYamlFileTimerCallback, this, true);    

            break;

        case LOAD_YAML_STUDENT_CONTROLLER_AGENT:
        case LOAD_YAML_STUDENT_CONTROLLER_COORDINATOR:
            // Set the "load student yaml" button to be disabled
            ui->load_student_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            m_timer_yaml_file_for_student_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::studentYamlFileTimerCallback, this, true);    

            break;

        case LOAD_YAML_MPC_CONTROLLER_AGENT:
        case LOAD_YAML_MPC_CONTROLLER_COORDINATOR:
            // Set the "load mpc yaml" button to be disabled
            ui->load_mpc_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            m_timer_yaml_file_for_mpc_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::mpcYamlFileTimerCallback, this, true);    

            break;

        case LOAD_YAML_REMOTE_CONTROLLER_AGENT:
        case LOAD_YAML_REMOTE_CONTROLLER_COORDINATOR:
            // Set the "load remote yaml" button to be disabled
            ui->load_remote_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            m_timer_yaml_file_for_remote_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::remoteYamlFileTimerCallback, this, true);    

            break;

        case LOAD_YAML_TUNING_CONTROLLER_AGENT:
        case LOAD_YAML_TUNING_CONTROLLER_COORDINATOR:
            // Set the "load tuning yaml" button to be disabled
            ui->load_tuning_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            m_timer_yaml_file_for_tuning_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::tuningYamlFileTimerCallback, this, true);    

            break;

        default:
            ROS_INFO("Unknown 'all controllers to load yaml' command, thus nothing will be disabled");
            break;
    }
}





// # Enable controllers
void MainWindow::on_enable_safe_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_SAFE_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_enable_demo_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_DEMO_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_enable_student_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_STUDENT_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_enable_mpc_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_MPC_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_enable_remote_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_REMOTE_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}

void MainWindow::on_enable_tuning_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_TUNING_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}



// # Custom command buttons - FOR DEMO CONTROLLER
void MainWindow::on_demoButton_1_clicked()
{
    CustomButton msg_custom_button;
    msg_custom_button.button_index = 1;
    msg_custom_button.command_code = 0;
    this->demoCustomButtonPublisher.publish(msg_custom_button);

    ROS_INFO("Demo button 1 pressed in GUI");
}

void MainWindow::on_demoButton_2_clicked()
{
    CustomButton msg_custom_button;
    msg_custom_button.button_index = 2;
    msg_custom_button.command_code = 0;
    this->demoCustomButtonPublisher.publish(msg_custom_button);
    ROS_INFO("Demo button 2 pressed in GUI");
}

void MainWindow::on_demoButton_3_clicked()
{
    CustomButton msg_custom_button;
    msg_custom_button.button_index = 3;
    msg_custom_button.command_code = (ui->demoField_3->text()).toFloat();
    this->demoCustomButtonPublisher.publish(msg_custom_button);
    ROS_INFO("Demo button 3 pressed in GUI");
}





// # Custom command buttons - FOR STUDENT CONTROLLER
void MainWindow::on_studentButton_1_clicked()
{
    CustomButton msg_custom_button;
    msg_custom_button.button_index = 1;
    msg_custom_button.command_code = 0;
    this->studentCustomButtonPublisher.publish(msg_custom_button);

    ROS_INFO("Student button 1 pressed in GUI");
}

void MainWindow::on_studentButton_2_clicked()
{
    CustomButton msg_custom_button;
    msg_custom_button.button_index = 2;
    msg_custom_button.command_code = 0;
    this->studentCustomButtonPublisher.publish(msg_custom_button);
    ROS_INFO("Student button 2 pressed in GUI");
}

void MainWindow::on_studentButton_3_clicked()
{
    CustomButton msg_custom_button;
    msg_custom_button.button_index = 3;
    msg_custom_button.command_code = (ui->studentField_3->text()).toFloat();
    this->studentCustomButtonPublisher.publish(msg_custom_button);
    ROS_INFO("Student button 3 pressed in GUI");
}



Setpoint MainWindow::correctSetpointBox(Setpoint setpoint, CrazyflieContext context)
{
    Setpoint corrected_setpoint;
    corrected_setpoint =  setpoint;

    float x_size = context.localArea.xmax - context.localArea.xmin;
    float y_size = context.localArea.ymax - context.localArea.ymin;
    float z_size = context.localArea.zmax - context.localArea.zmin;

    if(setpoint.x > x_size/2)
        corrected_setpoint.x = x_size/2;
    if(setpoint.y > y_size/2)
        corrected_setpoint.y = y_size/2;
    if(setpoint.z > z_size)
        corrected_setpoint.z = z_size;

    if(setpoint.x < -x_size/2)
        corrected_setpoint.x = -x_size/2;
    if(setpoint.y < -y_size/2)
        corrected_setpoint.y = -y_size/2;
    if(setpoint.z < 0)
        corrected_setpoint.z = 0;

    return corrected_setpoint;
}

bool MainWindow::setpointInsideBox(Setpoint setpoint, CrazyflieContext context)
{

    float x_size = context.localArea.xmax - context.localArea.xmin;
    float y_size = context.localArea.ymax - context.localArea.ymin;
    float z_size = context.localArea.zmax - context.localArea.zmin;
    //position check
	if((setpoint.x < -x_size/2) or (setpoint.x > x_size/2)) {
		ROS_INFO_STREAM("x outside safety box");
		return false;
	}
	if((setpoint.y < -y_size/2) or (setpoint.y > y_size/2)) {
		ROS_INFO_STREAM("y outside safety box");
		return false;
	}
	if((setpoint.z < 0) or (setpoint.z > z_size)) {
		ROS_INFO_STREAM("z outside safety box");
		return false;
	}

	return true;
}








// # Custom buttons for the REMOTE controller service
void MainWindow::on_remote_subscribe_button_clicked()
{
    // Initialise the message
    ViconSubscribeObjectName msg;
    // Set the subscribe flag
    msg.shouldSubscribe = true;
    // Set the object name
    msg.objectName = (ui->remote_object_name->text()).toUtf8().constData();
    // Publish the message
    this->remoteSubscribePublisher.publish(msg);
}

void MainWindow::on_remote_unsubscribe_button_clicked()
{
    // Initialise the message
    ViconSubscribeObjectName msg;
    // Set the subscribe flag
    msg.shouldSubscribe = false;
    // Set the object name
    msg.objectName = (ui->remote_object_name->text()).toUtf8().constData();
    // Publish the message
    this->remoteSubscribePublisher.publish(msg);
}

void MainWindow::on_remote_activate_button_clicked()
{
    // Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 1;
    // Publish the message
    this->remoteActivatePublisher.publish(msg);
}

void MainWindow::on_remote_deactivate_button_clicked()
{
    // Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 0;
    // Publish the message
    this->remoteActivatePublisher.publish(msg);
}

void MainWindow::remoteDataCallback(const CrazyflieData& objectData)
{
    // Check if the object is occluded
    if (objectData.occluded)
    {
        // Set the column heading label to have a red background
        // > IMPORTANT: Set the background auto fill property to true
        ui->remote_data_label->setAutoFillBackground(true);
        // > Get the pallette currently set for the label
        QPalette pal = ui->remote_roll_label->palette();
        // > Set the palette property that will change the background
        pal.setColor(QPalette::Window, QColor(Qt::red));
        // > Update the palette for the label
        ui->remote_data_label->setPalette(pal);
    }
    else
    {
        // Put the roll, pitch, yaw, and z data into the appropriate fields
        ui->remote_data_roll ->setText(QString::number( objectData.roll  * RAD2DEG, 'f', 1));
        ui->remote_data_pitch->setText(QString::number( objectData.pitch * RAD2DEG, 'f', 1));
        ui->remote_data_yaw  ->setText(QString::number( objectData.yaw   * RAD2DEG, 'f', 1));
        ui->remote_data_z    ->setText(QString::number( objectData.z,               'f', 2));

        // Set the column heading label to have a "normal" background
        // > IMPORTANT: Set the background auto fill property to true
        ui->remote_data_label->setAutoFillBackground(false);
        // > Get the pallette currently set for the roll label
        QPalette pal = ui->remote_roll_label->palette();
        // > Update the palette for the column heading label
        ui->remote_data_label->setPalette(pal);
    }
}

void MainWindow::remoteControlSetpointCallback(const CrazyflieData& setpointData)
{
    ui->remote_setpoint_roll ->setText(QString::number( setpointData.roll  * RAD2DEG, 'f', 1));
    ui->remote_setpoint_pitch->setText(QString::number( setpointData.pitch * RAD2DEG, 'f', 1));
    ui->remote_setpoint_yaw  ->setText(QString::number( setpointData.yaw   * RAD2DEG, 'f', 1));
    ui->remote_setpoint_z    ->setText(QString::number( setpointData.z,               'f', 2));
}






// TUNING CONTROLLER TAB
void MainWindow::on_tuning_test_horizontal_button_clicked()
{
	// Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 1;
    // Publish the message
    this->tuningActivateTestPublisher.publish(msg);
}

void MainWindow::on_tuning_test_vertical_button_clicked()
{
	// Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 2;
    // Publish the message
    this->tuningActivateTestPublisher.publish(msg);
}

void MainWindow::on_tuning_test_heading_button_clicked()
{
	// Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 3;
    // Publish the message
    this->tuningActivateTestPublisher.publish(msg);
}

void MainWindow::on_tuning_test_all_button_clicked()
{
	// Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 4;
    // Publish the message
    this->tuningActivateTestPublisher.publish(msg);
}

void MainWindow::on_tuning_test_circle_button_clicked()
{
    // Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = 5;
    // Publish the message
    this->tuningActivateTestPublisher.publish(msg);
}

void MainWindow::on_tuning_slider_horizontal_valueChanged(int value)
{
    // Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = value;
    // Publish the message
    this->tuningHorizontalGainPublisher.publish(msg);
}

void MainWindow::on_tuning_slider_vertical_valueChanged(int value)
{
    // Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = value;
    // Publish the message
    this->tuningVerticalGainPublisher.publish(msg);
}

void MainWindow::on_tuning_slider_heading_valueChanged(int value)
{
    // Initialise the message
    std_msgs::Int32 msg;
    // Set the msg data
    msg.data = value;
    // Publish the message
    this->tuningHeadingGainPublisher.publish(msg);
}


