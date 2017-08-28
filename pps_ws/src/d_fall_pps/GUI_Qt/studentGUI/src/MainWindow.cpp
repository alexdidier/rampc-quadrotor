#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <string>

#include <ros/ros.h>
#include <ros/network.h>

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

    std::string ros_namespace = ros::this_node::getNamespace();
    ROS_INFO("namespace: %s", ros_namespace.c_str());

    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    ros::NodeHandle nodeHandle(ros_namespace);
    crazyRadioStatusSubscriber = nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &MainWindow::crazyRadioStatusCallback, this);

    CFBatterySubscriber = nodeHandle.subscribe("CrazyRadio/CFBattery", 1, &MainWindow::CFBatteryCallback, this);

    flyingStateSubscriber = nodeHandle.subscribe("PPSClient/flyingState", 1, &MainWindow::flyingStateChangedCallback, this);


    // communication with PPS Client, just to make it possible to communicate through terminal also we use PPSClient's name
    ros::NodeHandle nh_PPSClient(ros_namespace + "/PPSClient");
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

    disableGUI();
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disableGUI()
{
}

void MainWindow::enableGUI()
{
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
    qstr.append(QString::number(battery_voltage));
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
            QString qstr = "x = ";
            qstr.append(QString::number(local.x));
            ui->current_x->setText(qstr);

            qstr = "y = ";
            qstr.append(QString::number(local.y));
            ui->current_y->setText(qstr);

            qstr = "z = ";
            qstr.append(QString::number(local.z));
            ui->current_z->setText(qstr);

            qstr = "yaw = ";
            qstr.append(QString::number(local.yaw));
            ui->current_yaw->setText(qstr);
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
