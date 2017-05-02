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
    explicit rosNodeThread(int argc, char **pArgv, const char * topic, QObject *parent = 0);
    virtual ~rosNodeThread();

    bool init();

    void messageCallback(const ViconData& data);



signals:
    // void newViconData(double, double, double, double, double, double);
    void newViconData(double, double);

public slots:
    void run();

private:
    int m_Init_argc;
    char** m_pInit_argv;
    const char * m_topic;

    QThread * m_pThread;

    ViconData m_vicon_data;

    ros::Subscriber m_vicon_subscriber;
    // ros::Publisher  sim_velocity;
};
#endif

