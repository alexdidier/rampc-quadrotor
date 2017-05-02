#ifndef ___ROSNODETHREAD_H___
#define ___ROSNODETHREAD_H___

#include <QtCore>
#include <QThread>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include "d_fall_pps/ViconData.h"

using namespace d_fall_pps;

class rosNodeThread : public QObject {
	Q_OBJECT
public:
    rosNodeThread(int argc, char **pArgv, const char * topic);
    virtual ~rosNodeThread();

    bool init();

    void messageCallback(const ViconData& data);

    Q_SLOT void run();

    // Q_SIGNAL void newPose(double,double,double);
private:
    int m_Init_argc;
    char** m_pInit_argv;
    const char * m_topic;

    QThread * m_pThread;

    ros::Subscriber test;
    // ros::Publisher  sim_velocity;
};
#endif

