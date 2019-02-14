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
//    Creates a thread that runs the ros node.
//
//    ----------------------------------------------------------------------------------


#include "rosNodeThread_for_systemConfigGUI.h"

#include "dfall_pkg/CMRead.h"
#include "dfall_pkg/CMUpdate.h"
#include "dfall_pkg/CMCommand.h"


rosNodeThread::rosNodeThread(int argc, char** pArgv, const char * node_name,  QObject* parent)
    :   QObject(parent),
        m_Init_argc(argc),
        m_pInit_argv(pArgv),
        m_node_name(node_name)
{
    /** Constructor for the node thread **/
}

rosNodeThread::~rosNodeThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    } // end if

    m_pThread->wait();
} // end destructor

bool rosNodeThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread); // QObject method

    connect(m_pThread, SIGNAL(started()), this, SLOT(run()));
    ros::init(m_Init_argc, m_pInit_argv, m_node_name);
    // Note that the variable "m_node_name" should be the
    // string "SystemConfigGUI" in this case

    if (!ros::master::check())
    {
        ROS_ERROR("No master found. Please make sure that there is a master roscore running");
        return false;           // do not start without ros.
    }

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh("~");

    ros::NodeHandle nodeHandle_dfall_root("/dfall");

    m_vicon_subscriber = nodeHandle_dfall_root.subscribe("ViconDataPublisher/ViconData", 100, &rosNodeThread::messageCallback, this);

    // clients for db services:
    m_read_db_client = nodeHandle_dfall_root.serviceClient<CMRead>("CentralManagerService/Read", false);
    m_update_db_client = nodeHandle_dfall_root.serviceClient<CMUpdate>("CentralManagerService/Update", false);
    m_command_db_client = nodeHandle_dfall_root.serviceClient<CMCommand>("CentralManagerService/Command", false);

    m_pThread->start();
    return true;
} // set up the thread

void rosNodeThread::messageCallback(const ptrToMessage& p_msg) // When a message arrives to the topic, this callback is executed
{
    emit newViconData(p_msg);   //pass the message to other places
}

// void rosNodeThread::messageCallback(const ViconData& data) // When a message arrives to the topic, this callback is executed
// {
//     QMutex * pMutex = new QMutex();
//     pMutex->lock();
//     ROS_INFO_STREAM("ViconData: " << data.x << ", " << data.y << ", " << data.z);
//     m_vicon_data.x = data.x;
//     m_vicon_data.y = data.y;
//     m_vicon_data.z = data.z;
//     m_vicon_data.yaw = data.yaw;
//     m_vicon_data.pitch = data.pitch;
//     m_vicon_data.roll = data.roll;
//     pMutex->unlock();
//     delete pMutex;
//     // Q_EMIT newViconData(m_vicon_data.x, m_vicon_data.y, m_vicon_data.z, m_vicon_data.yaw, m_vicon_data.pitch, m_vicon_data.roll);
//     emit newViconData(m_vicon_data.x, m_vicon_data.y);
// }

void rosNodeThread::run()
{
    ros::Rate loop_rate(100);
    QMutex * pMutex;
    while (ros::ok())
    {
        pMutex = new QMutex();

        // geometry_msgs::Twist cmd_msg;
        pMutex->lock();
        // cmd_msg.linear.x = m_speed;
        // cmd_msg.angular.z = m_angle;
        pMutex->unlock();

        // sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
    } // do ros things.
}
