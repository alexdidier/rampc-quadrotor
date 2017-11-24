//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero
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

#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

#include "d_fall_pps/CMQuery.h"

#include "d_fall_pps/ViconData.h"

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
    ROS_INFO("namespace: %s", m_ros_namespace.c_str());

    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    ros::NodeHandle nodeHandle(m_ros_namespace);

    customSetpointPublisher = nodeHandle.advertise<Setpoint>("CustomControllerService/Setpoint", 1);
    customSetpointSubscriber = nodeHandle.subscribe("CustomControllerService/Setpoint", 1, &MainWindow::customSetpointCallback, this);

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
    ros::NodeHandle nh_PPSClient(m_ros_namespace + "/PPSClient");
    crazyRadioCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("crazyRadioCommand", 1);
    PPSClientCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("Command", 1);

    // > For publishing a message that requests the 
    //   YAML parameters to be re-loaded from file
    // > The message contents specify which controller
    //   the parameters should be re-loaded for
    requestLoadControllerYamlPublisher;

    // Subscriber for locking the load the controller YAML
    // parameters when the Coordintor GUI requests a load
    requestLoadControllerYamlAllAgentsSubscriber = nodeHandle.subscribe("/my_GUI/requestLoadControllerYamlAllAgents", 1, &MainWindow::requestLoadControllerYamlAllAgentsCallback);
    

    // First get student ID
    if(!nh_PPSClient.getParam("studentID", m_student_id))
    {
		ROS_ERROR("Failed to get studentID");
	}

    // Then, Central manager
    centralManager = nodeHandle.serviceClient<CMQuery>("/CentralManagerService/Query", false);
    loadCrazyflieContext();



    std::vector<float> default_setpoint(4);
    ros::NodeHandle nh_safeControllerService(m_ros_namespace + "/SafeControllerService");

    ROS_INFO_STREAM(m_ros_namespace << "/SafeControllerService");

    if(!nh_safeControllerService.getParam("defaultSetpoint", default_setpoint))
    {
        ROS_ERROR_STREAM("couldn't find parameter 'defaultSetpoint'");
    }


    ui->current_setpoint_x->setText(QString::number(default_setpoint[0]));
    ui->current_setpoint_y->setText(QString::number(default_setpoint[1]));
    ui->current_setpoint_z->setText(QString::number(default_setpoint[2]));
    ui->current_setpoint_yaw->setText(QString::number(default_setpoint[3]));

    disableGUI();
    highlightSafeControllerTab();
    ui->label_battery->setStyleSheet("QLabel { color : red; }");
    m_battery_state = BATTERY_STATE_NORMAL;

    initialize_custom_setpoint();
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
}
void MainWindow::highlightCustomControllerTab()
{
    ui->tabWidget->tabBar()->setTabTextColor(0, Qt::black);
    ui->tabWidget->tabBar()->setTabTextColor(1, Qt::green);
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
        case CUSTOM_CONTROLLER:
            highlightCustomControllerTab();
            break;
        default:
            break;
    }
}

void MainWindow::safeSetpointCallback(const Setpoint& newSetpoint)
{
    m_safe_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
}

void MainWindow::customSetpointCallback(const Setpoint& newSetpoint)
{
    m_custom_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x_2->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y_2->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z_2->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw_2->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
}

void MainWindow::flyingStateChangedCallback(const std_msgs::Int32& msg)
{
    QString qstr = "Flying State: ";
    switch(msg.data)
    {
        case STATE_MOTORS_OFF:
            qstr.append("Motors OFF");
            break;
        case STATE_TAKE_OFF:
            qstr.append("Take OFF");
            break;
        case STATE_FLYING:
            qstr.append("Flying");
            break;
        case STATE_LAND:
            qstr.append("Land");
            break;
        default:
            break;
    }
    ui->flying_state_label->setText(qstr);
}

void MainWindow::batteryStateChangedCallback(const std_msgs::Int32& msg)
{
    // switch case with unabling buttons motors off, take off, etc... when battery is low
    QString qstr = "";
    switch(msg.data)
    {
        case BATTERY_STATE_LOW:
            qstr.append("Low Battery!");
            ui->take_off_button->setEnabled(false);
            ui->land_button->setEnabled(false);
            // ui->groupBox_4->setEnabled(false);

            ui->label_battery->setText(qstr);
            m_battery_state = BATTERY_STATE_LOW;
            break;
        case BATTERY_STATE_NORMAL:
            // ui->groupBox_4->setEnabled(true);
            ui->take_off_button->setEnabled(true);
            ui->land_button->setEnabled(true);

            ui->label_battery->clear();
            m_battery_state = BATTERY_STATE_NORMAL;
            break;
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
            // change icon, the rest of the GUI is available now
            ui->connected_disconnected_label->setText("Crazyradio status: Connected!");
            enableGUI();
            break;
        case CONNECTING:
            // change icon
            ui->connected_disconnected_label->setText("Crazyradio status: Connecting...");
            break;
        case DISCONNECTED:
            // change icon, the rest of the GUI is disabled
            ui->connected_disconnected_label->setText("Crazyradio status: Disconnected");
            disableGUI();
            break;
        default:
    		ROS_ERROR_STREAM("unexpected radio status: " << m_radio_status);
            break;
    }
    this->m_radio_status = radio_status;
}

float MainWindow::fromVoltageToPercent(float voltage)
{
    int num_cutoffs = m_cutoff_voltages.size();
    float hysteresis = 0.05;

    while(m_battery_level < num_cutoffs && voltage >= m_cutoff_voltages[m_battery_level])
    {
        ++m_battery_level;
    }
    while(m_battery_level > 0 && voltage < m_cutoff_voltages[m_battery_level - 1] - hysteresis)
    {
        --m_battery_level;
    }

    float percentage = 100.0 * m_battery_level/num_cutoffs;

    // should not hapen, but just in case...
    if(percentage > 100)
        percentage = 100;
    if(percentage < 0)
        percentage = 0;

    return percentage;
}

void MainWindow::updateBatteryVoltage(float battery_voltage)
{
    m_battery_voltage = battery_voltage;
    // Need to take voltage, display it and transform it to percentage
    // int percentage = (int) fromVoltageToPercent(m_battery_voltage);

    QString qstr = "";
    qstr.append(QString::number(battery_voltage, 'f', 2));
    qstr.append(" V");
    ui->voltage_field->setText(qstr);
}

void MainWindow::CFBatteryCallback(const std_msgs::Float32& msg)
{
    updateBatteryVoltage(msg.data);
}

void MainWindow::crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    ROS_INFO("Callback CrazyRadioStatus called");
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
            ui->current_x->setText(QString::number(local.x, 'f', 3));
            ui->current_y->setText(QString::number(local.y, 'f', 3));
            ui->current_z->setText(QString::number(local.z, 'f', 3));
            ui->current_yaw->setText(QString::number(local.yaw * RAD2DEG, 'f', 1));
            ui->current_pitch->setText(QString::number(local.pitch * RAD2DEG, 'f', 1));
            ui->current_roll->setText(QString::number(local.roll * RAD2DEG, 'f', 1));

            ui->current_x_2->setText(QString::number(local.x, 'f', 3));
            ui->current_y_2->setText(QString::number(local.y, 'f', 3));
            ui->current_z_2->setText(QString::number(local.z, 'f', 3));
            ui->current_yaw_2->setText(QString::number(local.yaw * RAD2DEG, 'f', 1));
            ui->current_pitch_2->setText(QString::number(local.pitch * RAD2DEG, 'f', 1));
            ui->current_roll_2->setText(QString::number(local.roll * RAD2DEG, 'f', 1));

            // also update diff
            ui->diff_x->setText(QString::number(m_safe_setpoint.x - local.x, 'f', 3));
            ui->diff_y->setText(QString::number(m_safe_setpoint.y - local.y, 'f', 3));
            ui->diff_z->setText(QString::number(m_safe_setpoint.z - local.z, 'f', 3));
            ui->diff_yaw->setText(QString::number((m_safe_setpoint.yaw - local.yaw) * RAD2DEG, 'f', 1));

            ui->diff_x_2->setText(QString::number(m_custom_setpoint.x - local.x, 'f', 3));
            ui->diff_y_2->setText(QString::number(m_custom_setpoint.y - local.y, 'f', 3));
            ui->diff_z_2->setText(QString::number(m_custom_setpoint.z - local.z, 'f', 3));
            ui->diff_yaw_2->setText(QString::number((m_custom_setpoint.yaw - local.yaw) * RAD2DEG, 'f', 1));
        }
    }
}

void MainWindow::on_RF_Connect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_RECONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("command reconnect published");
}

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

void MainWindow::on_set_setpoint_button_clicked()
{
    Setpoint msg_setpoint;
    msg_setpoint.x = (ui->new_setpoint_x->text()).toFloat();
    msg_setpoint.y = (ui->new_setpoint_y->text()).toFloat();
    msg_setpoint.z = (ui->new_setpoint_z->text()).toFloat();
    msg_setpoint.yaw = (ui->new_setpoint_yaw->text()).toFloat() * DEG2RAD;

    this->controllerSetpointPublisher.publish(msg_setpoint);
}

void MainWindow::initialize_custom_setpoint()
{
    Setpoint msg_setpoint;
    msg_setpoint.x = 0;
    msg_setpoint.y = 0;
    msg_setpoint.z = 0.4;
    msg_setpoint.yaw = 0;

    this->customSetpointPublisher.publish(msg_setpoint);
}

void MainWindow::on_set_setpoint_button_2_clicked()
{
    Setpoint msg_setpoint;
    msg_setpoint.x = (ui->new_setpoint_x_2->text()).toFloat();
    msg_setpoint.y = (ui->new_setpoint_y_2->text()).toFloat();
    msg_setpoint.z = (ui->new_setpoint_z_2->text()).toFloat();
    msg_setpoint.yaw = (ui->new_setpoint_yaw_2->text()).toFloat() * DEG2RAD;

    this->customSetpointPublisher.publish(msg_setpoint);
}

void MainWindow::on_RF_disconnect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_DISCONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("command disconnect published");
}




void MainWindow::on_load_safe_yaml_button_clicked()
{
    // Set the "load safe yaml" button to be disabled
    ui->load_safe_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the safe controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_SAFE_CONTROLLER;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("Request load of safe controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_custom_timer_yaml_file = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::safeYamlFileTimerCallback, this, true);
}

void MainWindow::safeYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load safe yaml" button again
    ui->load_safe_yaml_button->setEnabled(true);
}





void MainWindow::on_load_custom_yaml_button_clicked()
{
    // Set the "load custom yaml" button to be disabled
    ui->load_custom_yaml_button->setEnabled(false);

    // Send a message requesting the parameters from the YAML
    // file to be reloaded for the custom controller
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_CUSTOM_CONTROLLER;
    this->requestLoadControllerYamlPublisher.publish(msg);
    ROS_INFO("Request load of custom controller YAML published");

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_custom_timer_yaml_file = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::customYamlFileTimerCallback, this, true);    
}

void MainWindow::customYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load custom yaml" button again
    ui->load_custom_yaml_button->setEnabled(true);
}






void MainWindow::requestLoadControllerYamlAllAgentsCallback(const ros::TimerEvent&)
{
    // Extract from the "msg" for which controller the YAML
    // parameters should be loaded
    controller_to_load_yaml = msg.data;

    // Switch between loading for the different controllers
    switch(controller_to_load_yaml)
    {
        case LOAD_YAML_SAFE_CONTROLLER:
            // Set the "load safe yaml" button to be disabled
            ui->load_safe_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            ros::NodeHandle nodeHandle("~");
            m_custom_timer_yaml_file = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::safeYamlFileTimerCallback, this, true);

            break;

        case LOAD_YAML_CUSTOM_CONTROLLER:
            // Set the "load custom yaml" button to be disabled
            ui->load_custom_yaml_button->setEnabled(false);

            // Start a timer which will enable the button in its callback
            // > This is required because the agent node waits some time between
            //   re-loading the values from the YAML file and then assigning then
            //   to the local variable of the agent.
            // > Thus we use this timer to prevent the user from clicking the
            //   button in the GUI repeatedly.
            ros::NodeHandle nodeHandle("~");
            m_custom_timer_yaml_file = nodeHandle.createTimer(ros::Duration(1.5), &MainWindow::customYamlFileTimerCallback, this, true);    

            break;

        default:
            ROS_INFO("Unknown 'all controllers to load yaml' command, thus nothing will be disabled");
            break;
    }
}




void MainWindow::on_en_custom_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_CUSTOM_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}


void MainWindow::on_en_safe_controller_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_SAFE_CONTROLLER;
    this->PPSClientCommandPublisher.publish(msg);
}
