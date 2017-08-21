#include "rosNodeThread.h"

#include "d_fall_pps/CMRead.h"
#include "d_fall_pps/CMUpdate.h"
#include "d_fall_pps/CMCommand.h"


rosNodeThread::rosNodeThread(int argc, char** pArgv, const char * node_name, QObject* parent)
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
    ros::init(m_Init_argc, m_pInit_argv, m_node_name); // student_GUI is the name of this node

    if (!ros::master::check())
    {
        ROS_ERROR("Master not found, please check teacher computer is running and try again");
        return false;           // do not start without ros.
    }

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh("~");

    m_vicon_subscriber = nh.subscribe("/ViconDataPublisher/ViconData", 100, &rosNodeThread::messageCallback, this);

    // clients for db services:
    m_read_db_client = nh.serviceClient<CMRead>("/CentralManagerService/Read", false);
    m_update_db_client = nh.serviceClient<CMUpdate>("/CentralManagerService/Update", false);
    m_command_db_client = nh.serviceClient<CMCommand>("/CentralManagerService/Command", false);

    m_pThread->start();
    return true;
} // set up the thread

void rosNodeThread::messageCallback(const ptrToMessage& p_msg) // When a message arrives to the topic, this callback is executed
{
    emit newViconData(p_msg);   //pass the message to other places
}

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
        ROS_INFO("RUNNING");

        // sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
    } // do ros things.
}
