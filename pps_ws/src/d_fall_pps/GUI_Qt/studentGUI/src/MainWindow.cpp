#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <string>

#include <ros/ros.h>
#include <ros/network.h>

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_radio_status(DISCONNECTED),
    m_battery_level(0)
{
    m_rosNodeThread = new rosNodeThread(argc, argv, "student_GUI");
    ui->setupUi(this);
    m_rosNodeThread->init();

    std::string ros_namespace = ros::this_node::getNamespace();
    ROS_INFO("namespace: %s", ros_namespace.c_str());

    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    ros::NodeHandle nodeHandle(ros_namespace);
    crazyRadioStatusSubscriber = nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &MainWindow::crazyRadioStatusCallback, this);

    CFBatterySubscriber = nodeHandle.subscribe("CrazyRadio/CFBattery", 1, &MainWindow::CFBatteryCallback, this);


    // communication with PPS Client, just to make it possible to communicate through terminal also we use PPSClient's name
    ros::NodeHandle nh_PPSClient(ros_namespace + "/PPSClient");
    crazyRadioCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("crazyRadioCommand", 1);
    PPSClientCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("Command", 1);

    disableGUI();
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disableGUI()
{
    ui->enable_disable_CF_button->setEnabled(false);
    ui->battery_bar->setValue(0);
    ui->battery_bar->setEnabled(false);
}

void MainWindow::enableGUI()
{
    ui->enable_disable_CF_button->setEnabled(true);
    ui->battery_bar->setEnabled(true);
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
    return percentage;
}

void MainWindow::updateBatteryVoltage(float battery_voltage)
{
    m_battery_voltage = battery_voltage;
    // Need to take voltage, display it and transform it to percentage
    ui->battery_bar->setValue(fromVoltageToPercent(m_battery_voltage));
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

void MainWindow::updateNewViconData(const ptrToMessage& p_msg) //connected to newViconData, from node
{
}

void MainWindow::on_RF_Connect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_RECONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("command reconnect published");
}

void MainWindow::on_enable_disable_CF_button_clicked()
{
    if(ui->enable_disable_CF_button->text().toStdString() == "EnableCF") //enabled, disable if success
    {
        std_msgs::Int32 msg;
        msg.data = CMD_USE_CRAZYFLY_ENABLE;
        this->PPSClientCommandPublisher.publish(msg);
        ui->enable_disable_CF_button->setText("DisableCF");
    }
    else                        //disabled, enable if success
    {
        std_msgs::Int32 msg;
        msg.data = CMD_USE_CRAZYFLY_DISABLE;
        this->PPSClientCommandPublisher.publish(msg);
        ui->enable_disable_CF_button->setText("EnableCF");
    }
}
