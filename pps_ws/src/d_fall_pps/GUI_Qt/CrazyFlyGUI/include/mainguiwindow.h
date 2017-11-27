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
//    Teacher's GUI main window header.
//
//    ----------------------------------------------------------------------------------


#ifndef MAINGUIWINDOW_H
#define MAINGUIWINDOW_H

// The #define CATKIN_MAKE comes from cmake file
#include <QMainWindow>
#include <QTimer>
#include <QGridLayout>
#include <QGraphicsRectItem>


#ifdef CATKIN_MAKE
#include "rosNodeThread.h"
#include "marker.h"
#include "crazyFly.h"
#include "CFLinker.h"

#include "d_fall_pps/CrazyflieDB.h"
#include "d_fall_pps/CrazyflieEntry.h"

#include <std_msgs/Int32.h>


// The constants that are sent to the agents in order to
// "command" changes in their operation state
#define CMD_USE_SAFE_CONTROLLER   1
#define CMD_USE_CUSTOM_CONTROLLER 2
#define CMD_CRAZYFLY_TAKE_OFF     3
#define CMD_CRAZYFLY_LAND         4
#define CMD_CRAZYFLY_MOTORS_OFF   5

// The constants that are sent to the agents in order to
// adjust their radio connection
#define CMD_RECONNECT  0
#define CMD_DISCONNECT 1

// For which controller parameters to load
#define LOAD_YAML_SAFE_CONTROLLER   1
#define LOAD_YAML_CUSTOM_CONTROLLER 2


using namespace d_fall_pps;

#endif


#include "ui_mainguiwindow.h"
#include "myGraphicsScene.h"
#include "globalDefinitions.h"


namespace Ui {
class MainGUIWindow;
}


#ifdef CATKIN_MAKE
struct setpoint
{
    double x;
    double y;
    double z;
    double yaw;
};

class CSetpointQueue
{
public:
    CSetpointQueue();
    void insert(setpoint newElem);
    setpoint getNext();
    void print();

private:
    struct QueueElem
    {
        QueueElem(setpoint newElem) {elem.x=newElem.x; elem.y=newElem.y; elem.z=newElem.z; elem.yaw=newElem.yaw; next=NULL;}
        setpoint elem;
        QueueElem* next;
    };

    QueueElem* startElem;
    QueueElem* currElem;
    QueueElem* lastElem;
};
#endif

class MainGUIWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainGUIWindow(int argc, char **argv, QWidget *parent = 0);

    ~MainGUIWindow();

public slots:

private slots:
    #ifdef CATKIN_MAKE
    #endif

    void doNumCrazyFlyZonesChanged(int n);
    void transitionToMode(int mode);
    void on_removeTable_clicked();

    void on_radioButton_table_mode_toggled(bool checked);

    void on_radioButton_crazyfly_zones_mode_toggled(bool checked);
    void handleTablePiecesNumChanged(int newNum);

    void on_radioButton_lock_mode_toggled(bool checked);

    void on_checkBox_grid_toggled(bool checked);

    void on_checkBox_table_toggled(bool checked);

    void on_checkBox_crazyfly_zones_toggled(bool checked);

    void on_tabWidget_currentChanged(int index);

    void centerViewIndex(int index);

    void on_pushButton_fitAll_clicked();

    void on_checkBox_vicon_markers_toggled(bool checked);

    void on_checkBox_vicon_highlight_markers_toggled(bool checked);

    void on_save_in_DB_button_clicked();

    void on_load_from_DB_button_clicked();

    #ifdef CATKIN_MAKE
    void updateNewViconData(const ptrToMessage& p_msg);
    #endif
    void on_checkBox_vicon_crazyflies_toggled(bool checked);

    void on_scaleSpinBox_valueChanged(double arg1);
    void on_refresh_cfs_button_clicked();

    void on_refresh_student_ids_button_clicked();

    void on_link_button_clicked();

    void on_unlink_button_clicked();

    void updateComboBoxes();

    void setTabIndex(int index);
    void doTabClosed(int tab_index);

    void on_comboBoxCFs_currentTextChanged(const QString &arg1);

    
    // For the buttons that "command" all of the agent nodes
    // > For the radio connection
    void on_all_connect_button_clicked();
    void on_all_disconnect_button_clicked();
    // > For changing the operation state
    void on_all_take_off_button_clicked();
    void on_all_land_button_clicked();
    void on_all_motors_off_button_clicked();
    void on_all_enable_safe_controller_button_clicked();
    void on_all_enable_custom_controller_button_clicked();
    // > For loading the parameter
    void on_all_load_safe_controller_yaml_button_clicked();
    void on_all_load_custom_controller_yaml_button_clicked();
    // > For sending a message with updated parameters
    void on_all_send_safe_controller_yaml_button_clicked();
    void on_all_send_custom_controller_yaml_button_clicked();
    

private:

    Ui::MainGUIWindow *ui;
    myGraphicsScene* scene;

    ros::Timer m_timer_yaml_file_for_safe_controller;
    ros::Timer m_timer_yaml_file_for_custom_controlller;

    void _init();

    void safeYamlFileTimerCallback(const ros::TimerEvent&);
    void customYamlFileTimerCallback(const ros::TimerEvent&);

    void customSendYamlAsMessageTimerCallback(const ros::TimerEvent&);
    


    #ifdef CATKIN_MAKE
    rosNodeThread* _rosNodeThread;
    std::vector<Marker*> markers_vector;
    std::vector<crazyFly*> crazyflies_vector;
    CFLinker* cf_linker;

    std::string ros_namespace;

    ros::Publisher DBChangedPublisher;
    ros::Publisher emergencyStopPublisher;

    // Publsher for sending "commands" from here (the master) to all
    // of the agent nodes (where a "command" is the integer that
    // gives the directive to "take-off", "land, "motors-off", etc...)
    ros::Publisher commandAllAgentsPublisher;

    // Publisher for sending a request from here (the master) to all
    // of the agent nodes that they should re-load parameters from
    // the YAML files for their controllers
    ros::Publisher requestLoadControllerYamlAllAgentsPublisher;

    // Publisher for sending a message from here (the master) to all
    // of the agent nodes with the Custom Controller YAML parameters
    // as the contents of the message
    ros::Publisher customYAMLasMessagePublisher;

    // Publisher for sending a request from here (the master) to all
    // of the agents nodes that they should (re/dis)-connect from
    // the Crazy-Radio
    ros::Publisher crazyRadioCommandAllAgentsPublisher;
    #endif

    void updateComboBoxesCFs();

    void updateComboBoxesCFZones();

    int getTabIndexFromName(QString name);

    CrazyflieDB m_data_base;

    void clear_database_file();

    void fill_database_file();

    int read_database_from_file(CrazyflieDB &read_db);

    void save_database_file();

    void insert_or_update_entry_database(CrazyflieEntry entry);

};



#endif // MAINGUIWINDOW_H
