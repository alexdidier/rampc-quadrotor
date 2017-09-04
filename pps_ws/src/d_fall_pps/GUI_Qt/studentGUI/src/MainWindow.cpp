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

    // subscribers
    crazyRadioStatusSubscriber = nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &MainWindow::crazyRadioStatusCallback, this);

    CFBatterySubscriber = nodeHandle.subscribe("CrazyRadio/CFBattery", 1, &MainWindow::CFBatteryCallback, this);

    flyingStateSubscriber = nodeHandle.subscribe("PPSClient/flyingState", 1, &MainWindow::flyingStateChangedCallback, this);

    setpointPublisher = nodeHandle.advertise<Setpoint>("SafeControllerService/Setpoint", 1);
    setpointSubscriber = nodeHandle.subscribe("SafeControllerService/Setpoint", 1, &MainWindow::setpointCallback, this);
    DBChangedSubscriber = nodeHandle.subscribe("/my_GUI/DBChanged", 1, &MainWindow::DBChangedCallback, this);

    ros::NodeHandle my_nodeHandle("~");
    customYAMLloadedPublisher = my_nodeHandle.advertise<std_msgs::Int32>("customYAMLloaded", 1);
    safeYAMLloadedPublisher = my_nodeHandle.advertise<std_msgs::Int32>("safeYAMLloaded", 1);


    // communication with PPS Client, just to make it possible to communicate through terminal also we use PPSClient's name
    ros::NodeHandle nh_PPSClient(m_ros_namespace + "/PPSClient");
    crazyRadioCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("crazyRadioCommand", 1);
    PPSClientCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("Command", 1);



    // First get student ID
    if(!nh_PPSClient.getParam("studentID", m_student_id))
    {
		ROS_ERROR("Failed to get studentID");
	}

    // Then, Central manager
    centralManager = nodeHandle.serviceClient<CMQuery>("/CentralManagerService/Query", false);
    loadCrazyflieContext();

    // we now have the m_context variable with the current context. Put CF Name in label

    QString qstr = "StudentID ";
    qstr.append(QString::number(m_student_id));
    qstr.append(" connected to CF ");
    qstr.append(QString::fromStdString(m_context.crazyflieName));
    ui->groupBox->setTitle(qstr);

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
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disableGUI()
{
    ui->groupBox_general->setEnabled(false);
}

void MainWindow::enableGUI()
{
    ui->groupBox_general->setEnabled(true);
}

void MainWindow::DBChangedCallback(const std_msgs::Int32& msg)
{
    loadCrazyflieContext();
    ROS_INFO("context reloaded in student_GUI");
}

void MainWindow::setpointCallback(const Setpoint& newSetpoint)
{
    m_setpoint = newSetpoint;
    // here we get the new setpoint, need to update it in GUI
    ui->current_setpoint_x->setText(QString::number(newSetpoint.x, 'f', 3));
    ui->current_setpoint_y->setText(QString::number(newSetpoint.y, 'f', 3));
    ui->current_setpoint_z->setText(QString::number(newSetpoint.z, 'f', 3));
    ui->current_setpoint_yaw->setText(QString::number(newSetpoint.yaw * RAD2DEG, 'f', 1));
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

void MainWindow::setCrazyRadioStatus(int radio_status)
{
    // add more things whenever the status is changed
    switch(radio_status)
    {
        case CONNECTED:
            // change icon, the rest of the GUI is available now
            ui->connected_disconnected_label->setText("Connected!");
            enableGUI();
            break;
        case CONNECTING:
            // change icon
            ui->connected_disconnected_label->setText("Connecting...");
            break;
        case DISCONNECTED:
            // change icon, the rest of the GUI is disabled
            ui->connected_disconnected_label->setText("Disconnected");
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
    int percentage = (int) fromVoltageToPercent(m_battery_voltage);

    if(percentage != ui->battery_bar->value())
    {
        // ui->battery_bar->setValue(percentage);
    }

    QString qstr = "Raw voltage: ";
    qstr.append(QString::number(battery_voltage, 'f', 2));
    ui->raw_voltage->setText(qstr);
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

            // also update diff
            ui->diff_x->setText(QString::number(m_setpoint.x - local.x, 'f', 3));
            ui->diff_y->setText(QString::number(m_setpoint.y - local.y, 'f', 3));
            ui->diff_z->setText(QString::number(m_setpoint.z - local.z, 'f', 3));
            ui->diff_yaw->setText(QString::number((m_setpoint.yaw - local.yaw) * RAD2DEG, 'f', 1));
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

    this->setpointPublisher.publish(msg_setpoint);
}

void MainWindow::on_pushButton_3_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_DISCONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("command disconnect published");
}

void MainWindow::safeYamlFileTimerCallback(const ros::TimerEvent&)
{
    // send msg that says that parameters have changed in YAML file
    std_msgs::Int32 msg;
    msg.data = 1;
    this->safeYAMLloadedPublisher.publish(msg);
    ROS_INFO("YALMloaded published");
    ui->load_safe_yaml_button->setEnabled(true);
}

void MainWindow::on_load_safe_yaml_button_clicked()
{
    ui->load_safe_yaml_button->setEnabled(false);
    ros::NodeHandle nodeHandle("~");
    m_custom_timer_yaml_file = nodeHandle.createTimer(ros::Duration(1), &MainWindow::safeYamlFileTimerCallback, this, true);

    std::string d_fall_pps_path = ros::package::getPath("d_fall_pps");
    ROS_INFO_STREAM(d_fall_pps_path);

    // first, reload the name of the custom controller:
    std::string cmd = "rosparam load " + d_fall_pps_path + "/param/ClientConfig.yaml " + m_ros_namespace + "/PPSClient";
    system(cmd.c_str());
    ROS_INFO_STREAM(cmd);

    // then, reload the parameters of the custom controller:
    cmd = "rosparam load " + d_fall_pps_path + "/param/SafeController.yaml " + m_ros_namespace + "/SafeControllerService";
    system(cmd.c_str());
    ROS_INFO_STREAM(cmd);
}



void MainWindow::customYamlFileTimerCallback(const ros::TimerEvent&)
{
    // send msg that says that parameters have changed in YAML file
    std_msgs::Int32 msg;
    msg.data = 1;
    this->customYAMLloadedPublisher.publish(msg);
    ROS_INFO("YALMloaded published");
    ui->load_custom_yaml_button->setEnabled(true);
}

void MainWindow::on_load_custom_yaml_button_clicked()
{
    ui->load_custom_yaml_button->setEnabled(false);
    ros::NodeHandle nodeHandle("~");
    m_custom_timer_yaml_file = nodeHandle.createTimer(ros::Duration(1), &MainWindow::customYamlFileTimerCallback, this, true);

    std::string d_fall_pps_path = ros::package::getPath("d_fall_pps");
    ROS_INFO_STREAM(d_fall_pps_path);

    // first, reload the name of the custom controller:
    std::string cmd = "rosparam load " + d_fall_pps_path + "/param/ClientConfig.yaml " + m_ros_namespace + "/PPSClient";
    system(cmd.c_str());
    ROS_INFO_STREAM(cmd);

    // then, reload the parameters of the custom controller:
    cmd = "rosparam load " + d_fall_pps_path + "/param/CustomController.yaml " + m_ros_namespace + "/CustomControllerService";
    system(cmd.c_str());
    ROS_INFO_STREAM(cmd);
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
