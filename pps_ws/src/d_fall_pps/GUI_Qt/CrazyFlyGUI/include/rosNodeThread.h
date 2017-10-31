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

