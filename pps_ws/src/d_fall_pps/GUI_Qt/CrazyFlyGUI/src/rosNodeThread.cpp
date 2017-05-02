#include "rosNodeThread.h"

rosNodeThread::rosNodeThread(int argc, char** pArgv, const char * topic)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv),
        m_topic(topic)

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

    connect(m_pThread, &QThread::started, this, &rosNodeThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "my_GUI"); // GUI is the name of this node

    if (!ros::master::check())
        return false;           // do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh("~");

    // sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    test = nh.subscribe(m_topic, 10, &rosNodeThread::messageCallback, this);

    m_pThread->start();
    return true;
} // set up the thread

void rosNodeThread::messageCallback(const ViconData& data) // When a message arrives to the topic, this callback is executed
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    ROS_INFO_STREAM("ViconData: " << data.x << ", " << data.y << ", " << data.z);
    pMutex->unlock();
    delete pMutex;
    // Q_EMIT newPose(m_xPos, m_yPos, m_aPos);
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

        // sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
    } // do ros things.
}
