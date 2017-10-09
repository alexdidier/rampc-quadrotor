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
#include "d_fall_pps/UnlabeledMarker.h"
#include "d_fall_pps/CrazyflieData.h"
#include "d_fall_pps/ViconData.h"

using namespace d_fall_pps;

typedef ViconData::ConstPtr ptrToMessage;

Q_DECLARE_METATYPE(ptrToMessage)


class rosNodeThread : public QObject {
	Q_OBJECT
public:
    explicit rosNodeThread(int argc, char **pArgv, const char * node_name, QObject *parent = 0);
    virtual ~rosNodeThread();

    bool init();

    // void messageCallback(const ViconData& data);
    void messageCallback(const ptrToMessage& p_msg);

    ros::ServiceClient m_read_db_client;
    ros::ServiceClient m_update_db_client;
    ros::ServiceClient m_command_db_client;

signals:

    void newViconData(const ptrToMessage& p_msg);

public slots:
    void run();

private:
    int m_Init_argc;
    char** m_pInit_argv;
    const char * m_node_name;

    QThread * m_pThread;

    ViconData m_vicon_data;

    ros::Subscriber m_vicon_subscriber;
    // ros::Publisher  sim_velocity;
};
#endif

