#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/Int32.h>

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_rosNodeThread = new rosNodeThread(argc, argv, "student_GUI");
    ui->setupUi(this);
    m_rosNodeThread->init();
    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    ros::NodeHandle nh("~");

    crazyRadioCommandPublisher = nh.advertise<std_msgs::Int32>("crazyRadioCommand", 1);
}

MainWindow::~MainWindow()
{
    delete ui;
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
