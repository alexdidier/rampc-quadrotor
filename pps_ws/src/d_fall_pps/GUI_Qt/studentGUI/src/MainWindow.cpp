#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <string>

#include <ros/ros.h>
#include <ros/network.h>

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_radio_status(DISCONNECTED)
{
    m_rosNodeThread = new rosNodeThread(argc, argv, "student_GUI");
    ui->setupUi(this);
    m_rosNodeThread->init();

    std::string ros_namespace = ros::this_node::getNamespace();
    ROS_INFO("namespace: %s", ros_namespace.c_str());

    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    ros::NodeHandle nodeHandle("~");
    crazyRadioStatusSubscriber = nodeHandle.subscribe(ros_namespace + "/CrazyRadioStatus", 1, &MainWindow::crazyRadioStatusCallback, this);

    // communication with PPS Client, just to make it possible to communicate through terminal also we use PPSClient's name
    ros::NodeHandle nh_PPSClient(ros_namespace + "/PPSClient");
    crazyRadioCommandPublisher = nh_PPSClient.advertise<std_msgs::Int32>("crazyRadioCommand", 1);
    PPSClientCommadPublisher = nh_PPSClient.advertise<std_msgs::Int32>("Command", 1);

    disableGUI();
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disableGUI()
{
    ui->enable_disable_CF_button->setEnabled(false);
}

void MainWindow::enableGUI()
{
    ui->enable_disable_CF_button->setEnabled(true);
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
        msg.data = CMD_USE_CRAZYFLY_DISABLE;
        this->PPSClientCommadPublisher.publish(msg);
        ui->enable_disable_CF_button->setText("DisableCF");
    }
    else                        //disabled, enable if success
    {
        std_msgs::Int32 msg;
        msg.data = CMD_USE_CRAZYFLY_ENABLE;
        this->PPSClientCommadPublisher.publish(msg);
        ui->enable_disable_CF_button->setText("EnableCF");
    }
}
