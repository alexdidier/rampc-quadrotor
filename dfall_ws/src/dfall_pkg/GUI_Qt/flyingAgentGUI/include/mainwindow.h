//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
//    The Flying Agent GUI main window.
//
//    ----------------------------------------------------------------------------------





#ifndef MAINWINDOW_FLYINGAGENTGUI_H
#define MAINWINDOW_FLYINGAGENTGUI_H

#include <QMainWindow>
#include <QShortcut>
#include <QMutex>

#include <QTextStream>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>
#include "rosNodeThread_for_flyingAgentGUI.h"

// Include the standard message types
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/StringWithHeader.h"

#include "nodes/Constants.h"

// Namespacing the package
using namespace dfall_pkg;
//using namespace std;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

#endif


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();



private:
    Ui::MainWindow *ui;

    QShortcut* m_close_GUI_shortcut;

    // > For the type of this node,
    //   i.e., an agent or a coordinator
    int m_type = 0;

    // > For the ID of this node
    int m_ID = 0;

    // For coordinating multiple agents
    std::vector<int> m_vector_of_agentIDs_toCoordinate;
    bool m_shouldCoordinateAll = true;
    QMutex m_agentIDs_toCoordinate_mutex;
    

#ifdef CATKIN_MAKE
    rosNodeThread* m_rosNodeThread;

	// The namespace into which this parameter service loads yaml parameters
	std::string m_parameter_service_namespace;

	ros::Publisher m_requestLoadYamlFilenamePublisher;
#endif


    // --------------------------------------------------- //
    // PRIVATE FUNCTIONS
#ifdef CATKIN_MAKE
    bool getTypeAndIDParameters();
#endif




private slots:
    // PRIVATE METHODS FOR MENU ITEM CALLBACKS
    void on_actionShowHide_Coordinator_triggered();
    void on_action_LoadYAML_BatteryMonitor_triggered();
    void on_action_LoadYAML_FlyingAgentClientConfig_triggered();

    // FOR THE CONTROLLERS MENU
    void on_action_showHideController_default_changed();
    void on_action_showHideController_student_changed();
    void on_action_showHideController_picker_changed();
    void on_action_showHideController_tuning_changed();
    void on_action_showHideController_template_changed();

    void on_action_testMotors_changed();

};

#endif // MAINWINDOW_H
