//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat, Angel Romero
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
#include "rosNodeThread_for_managerGUI.h"

// Include the standard message types
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/IntWithHeader.h"
#include "marker.h"
#include "crazyFly.h"
#include "CFLinker.h"

// Include the DFALL service types
#include "dfall_pkg/CrazyflieDB.h"
#include "dfall_pkg/CrazyflieEntry.h"

// Include the shared definitions
#include "nodes/Constants.h"

// Include other classes
#include "classes/GetParamtersAndNamespaces.h"


using namespace dfall_pkg;


#else
// Include the shared definitions
#include "include/constants_for_qt_compile.h"

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
    
    // For the emergency stop button
    void on_emergency_stop_button_clicked();
    

private:

    Ui::MainGUIWindow *ui;
    myGraphicsScene* scene;

    void _init();

#ifdef CATKIN_MAKE
    float getParameterFloat(ros::NodeHandle& nodeHandle, std::string name);
    void getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length);
    int getParameterInt(ros::NodeHandle& nodeHandle, std::string name);
    void getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length);
    int getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val);
    bool getParameterBool(ros::NodeHandle& nodeHandle, std::string name);
#endif


#ifdef CATKIN_MAKE
    rosNodeThread* _rosNodeThread;
    std::vector<Marker*> markers_vector;
    std::vector<crazyFly*> crazyflies_vector;
    CFLinker* cf_linker;

    std::string ros_namespace;

    //ros::Publisher DBChangedPublisher;
    ros::Publisher emergencyStopPublisher;
#endif

    void updateComboBoxesCFs();

    void updateComboBoxesCFZones();

    int getTabIndexFromName(QString name);


#ifdef CATKIN_MAKE
    CrazyflieDB m_data_base;

    void clear_database_file();

    void fill_database_file();

    int read_database_from_file(CrazyflieDB &read_db);

    void save_database_file();

    void insert_or_update_entry_database(CrazyflieEntry entry);
#endif

};



#endif // MAINGUIWINDOW_H
