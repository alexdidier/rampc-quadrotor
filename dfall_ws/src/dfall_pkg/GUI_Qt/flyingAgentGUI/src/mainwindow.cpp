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





#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
#ifdef CATKIN_MAKE
    m_rosNodeThread = new rosNodeThread(argc, argv, "FlyingAgentGUI");
#endif

#ifdef CATKIN_MAKE
    m_rosNodeThread->init();
#endif
    ui->setupUi(this);

    // ADD KEYBOARD SHORTCUTS
    // > For "kill GUI node", press "CTRL+C" while the GUI window is the focus
    m_close_GUI_shortcut = new QShortcut(QKeySequence(tr("CTRL+C")), this, SLOT(close()));

#ifdef CATKIN_MAKE
    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[FLYING AGENT GUI MAIN WINDOW] ros::this_node::getNamespace() =  " << this_namespace);


    // Get the type and ID of this parameter service
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the TYPE and ID are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[FLYING AGENT GUI MAIN WINDOW] Node NOT FUNCTIONING :-)");
        ros::spin();
    }
    
    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the namespace of this "ParameterService" node
    std::string this_node_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[FLYING AGENT GUI MAIN WINDOW] ros::this_node::getNamespace() =  " << this_node_namespace);

    // Construct the string to the namespace of this Paramater Service
    m_parameter_service_namespace = this_node_namespace + '/' + "ParameterService";
    ROS_INFO_STREAM("[FLYING AGENT GUI MAIN WINDOW] parameter service is: " << m_parameter_service_namespace);

    // Get the node handle to this parameter service
    ros::NodeHandle nodeHandle_to_parameter_service(m_parameter_service_namespace);
    m_requestLoadYamlFilenamePublisher = nodeHandle_to_parameter_service.advertise<StringWithHeader>("requestLoadYamlFilename", 1);

    // Remove the show/hide coordinator menu item if launch as an agent
    if (m_type==TYPE_AGENT)
    {
        // Hide the coordinator part of the GUI
        ui->customWidget_coordinator->hide();
        // And make the menu item unavailable
        ui->actionShowHide_Coordinator->setEnabled(false);

    }
#endif

    // CONNECT TO THE COORDINATOR SIGNAL TO BE ALWAYS UPDATED
    // WITH THE LIST OF AGENT IDs TO COORDINATE
    // This is passed from the "coordinator" to the elements
    // in the main part of the GUI, namely to the:
    // > "top banner",
    // > "connect,start,stop bar",
    // > "enable controller, load yaml bar", and
    // > "controller tabs widget".
    QObject::connect(
            ui->customWidget_coordinator , &Coordinator::agentIDsToCoordinateChanged ,
            ui->customWidget_topBanner , &TopBanner::setAgentIDsToCoordinate
            );

    QObject::connect(
            ui->customWidget_coordinator , &Coordinator::agentIDsToCoordinateChanged ,
            ui->customWidget_connectStartStopBar , &ConnectStartStopBar::setAgentIDsToCoordinate
            );

    QObject::connect(
            ui->customWidget_coordinator , &Coordinator::agentIDsToCoordinateChanged ,
            ui->customWidget_enableControllerLoadYamlBar , &EnableControllerLoadYamlBar::setAgentIDsToCoordinate
            );

    QObject::connect(
            ui->customWidget_coordinator , &Coordinator::agentIDsToCoordinateChanged ,
            ui->customWidget_controller_tabs , &ControllerTabs::setAgentIDsToCoordinate
            );

    // CONNECT SIGNAL/SLOT FOR PASSING THE OBJECT NAME FOR WHICH
    // POSE DATA SHOULD BE DISPLAYED
    // This is passed from the "top banner" to the "controller tabs"
    // because the "top banner" is in charge of fetching the object
    // name from the database, and the "controller tabs" are where
    // the pose data is displayed
    QObject::connect(
            ui->customWidget_topBanner , &TopBanner::objectNameForDisplayingPoseDataValueChanged ,
            ui->customWidget_controller_tabs , &ControllerTabs::setObjectNameForDisplayingPoseData
            );


    // TOGGLE THE CONTROLLERS THAT ARE VISIBLE
    // By default all controller buttons and tabs are visible
    // and the menu item is checked. Hence, to hide a controller
    // the menu item simply needs to be "triggered"

    // > For the picker controller
    ui->action_showHideController_picker->trigger();
    // > For the tuning controller
    ui->action_showHideController_tuning->trigger();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionShowHide_Coordinator_triggered()
{
    //ui->customWidget_enableControllerLoadYamlBar->setEnabled(false);
    if ( ui->customWidget_coordinator->isHidden() )
    {
        ui->customWidget_coordinator->show();
        ui->coordinator_to_main_panel_vertical_line->show();
        QString qstr = "Hide Coordinator";
        ui->actionShowHide_Coordinator->setText(qstr);
    }
    else
    {
        ui->customWidget_coordinator->hide();
        ui->coordinator_to_main_panel_vertical_line->hide();
        QString qstr = "Show Coordinator";
        ui->actionShowHide_Coordinator->setText(qstr);
    }
}


void MainWindow::on_action_LoadYAML_BatteryMonitor_triggered()
{
#ifdef CATKIN_MAKE
    // Inform the user that the menu item was selected
    ROS_INFO("[FLYING AGENT GUI MAIN WINDOW] Load Battery Monitor YAML was clicked.");

    // Create a local variable for the message
    StringWithHeader yaml_filename_msg;
    // Specify the data
    yaml_filename_msg.data = "BatteryMonitor";
    // Set for whom this applies to
    yaml_filename_msg.shouldCheckForAgentID = false;
    // Send the message
    m_requestLoadYamlFilenamePublisher.publish(yaml_filename_msg);
#endif
}


void MainWindow::on_action_LoadYAML_FlyingAgentClientConfig_triggered()
{
#ifdef CATKIN_MAKE
    // Inform the user that the menu item was selected
    ROS_INFO("[FLYING AGENT GUI MAIN WINDOW] Load Client Config YAML was clicked.");

    // Create a local variable for the message
    StringWithHeader yaml_filename_msg;
    // Specify the data
    yaml_filename_msg.data = "FlyingAgentClientConfig";
    // Set for whom this applies to
    yaml_filename_msg.shouldCheckForAgentID = false;
    // Send the message
    m_requestLoadYamlFilenamePublisher.publish(yaml_filename_msg);
#endif
}





void MainWindow::on_action_showHideController_default_changed()
{
    // Notify the UI elements of this change
    ui->customWidget_enableControllerLoadYamlBar->showHideController_default_changed();
    ui->customWidget_controller_tabs->showHideController_default_changed();
}


void MainWindow::on_action_showHideController_student_changed()
{
    // Notify the UI elements of this change
    ui->customWidget_enableControllerLoadYamlBar->showHideController_student_changed();
    ui->customWidget_controller_tabs->showHideController_student_changed();
}


void MainWindow::on_action_showHideController_picker_changed()
{
    // Notify the UI elements of this change
    ui->customWidget_enableControllerLoadYamlBar->showHideController_picker_changed();
    ui->customWidget_controller_tabs->showHideController_picker_changed();
}

void MainWindow::on_action_showHideController_tuning_changed()
{
    // Notify the UI elements of this change
    ui->customWidget_enableControllerLoadYamlBar->showHideController_tuning_changed();
    ui->customWidget_controller_tabs->showHideController_tuning_changed();
}

void MainWindow::on_action_showHideController_deepc_changed()
{
    // Notify the UI elements of this change
    ui->customWidget_enableControllerLoadYamlBar->showHideController_deepc_changed();
    ui->customWidget_controller_tabs->showHideController_deepc_changed();
}
void MainWindow::on_action_showHideController_rampc_changed()
{
    // Notify the UI elements of this change
    //ui->customWidget_enableControllerLoadYamlBar->showHideController_rampc_changed();
    ui->customWidget_controller_tabs->showHideController_rampc_changed();
}

void MainWindow::on_action_showHideController_logging_changed()
{
    // Notify the UI elements of this change
    //ui->customWidget_enableControllerLoadYamlBar->showHideController_logging_changed();
    ui->customWidget_controller_tabs->showHideController_logging_changed();
}

void MainWindow::on_action_testMotors_triggered()
{
    // Notify the UI elements of this change
    ui->customWidget_enableControllerLoadYamlBar->testMotors_triggered();
}


//    ----------------------------------------------------------------------------------
//    III  DDDD       &&&      TTTTT  Y   Y  PPPP   EEEEE
//     I   D   D     &           T     Y Y   P   P  E
//     I   D   D      &          T      Y    PPPP   EEE
//     I   D   D     & & &       T      Y    P      E
//    III  DDDD       &&&        T      Y    P      EEEEE
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
bool MainWindow::getTypeAndIDParameters()
{
    // Initialise the return variable as success
    bool return_was_successful = true;

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the value of the "type" parameter into a local string variable
    std::string type_string;
    if(!nodeHandle.getParam("type", type_string))
    {
        // Throw an error if the agent ID parameter could not be obtained
        ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Failed to get type");
    }

    // Set the "m_type" class variable based on this string loaded
    if ((!type_string.compare("coordinator")))
    {
        m_type = TYPE_COORDINATOR;
    }
    else if ((!type_string.compare("agent")))
    {
        m_type = TYPE_AGENT;
    }
    else
    {
        // Set "m_type" to the value indicating that it is invlid
        m_type = TYPE_INVALID;
        return_was_successful = false;
        ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] The 'type' parameter retrieved was not recognised.");
    }


    // Construct the string to the namespace of this Paramater Service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            // Get the value of the "agentID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("agentID", m_ID))
            {
                // Throw an error if the agent ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[ENABLE CONTROLLER LOAD YAML GUI BAR] Is of type AGENT with ID = " << m_ID);
            }
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            // Get the value of the "coordID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("coordID", m_ID))
            {
                // Throw an error if the coord ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[ENABLE CONTROLLER LOAD YAML GUI BAR] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
