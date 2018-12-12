//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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
//    The title displayed at the top of the GUI
//
//    ----------------------------------------------------------------------------------





#ifndef TOPBANNER_H
#define TOPBANNER_H

#include <QWidget>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "d_fall_pps/IntWithHeader.h"
#include "d_fall_pps/AreaBounds.h"
#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CMQuery.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace d_fall_pps;

#endif

namespace Ui {
class TopBanner;
}

class TopBanner : public QWidget
{
    Q_OBJECT

public:
    explicit TopBanner(QWidget *parent = 0);
    ~TopBanner();

private:
	// --------------------------------------------------- //
    // PRIVATE VARIABLES
    Ui::TopBanner *ui;

    // > For the type of this node,
    //   i.e., an agent or a coordinator
	int m_type = 0;

	// > For the ID of this node
	int m_ID = 0;

	// The namespace into which node operates
	std::string m_base_namespace;



	// --------------------------------------------------- //
    // PRIVATE FUNCTIONS

    // > For loading the "context" for this agent,
    //   i.e., the {agentID,cfID,flying zone} tuple
    void loadCrazyflieContext();



#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE VARIABLES FOR ROS

    // > For the "context" of this agent
    d_fall_pps::CrazyflieContext my_context;

    // PUBLISHERS AND SUBSRIBERS

    // > For changes in the database that defines {agentID,cfID,flying zone} links
    ros::Subscriber databaseChangedSubscriber;
    ros::ServiceClient centralManagerDatabaseService;


    // --------------------------------------------------- //
    // PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

    // Get the type and ID of this node
    bool getTypeAndIDParameters();

    // > For the notification that the database was changes,
    //   received on the "DatabaseChangedSubscriber"
    void databaseChangedCallback(const std_msgs::Int32& msg);
#endif


private slots:

    // PRIVATE METHODS FOR BUTTON CALLBACKS
    // > For the emergency stop button
    void on_emergency_stop_button_clicked();

};

#endif // TOPBANNER_H
