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


#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include "crazyFlyZoneTab.h"
#include "myGraphicsScene.h"
#include "myGraphicsView.h"
#include "channelLUT.h"

#include <QObject>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QString>
#include <QMetaType>
#include <QDir>
#include <regex>
#include <QShortcut>

#ifdef CATKIN_MAKE
#include "d_fall_pps/UnlabeledMarker.h"
#include "d_fall_pps/CMRead.h"
#include "d_fall_pps/CrazyflieEntry.h"
#include "d_fall_pps/CMUpdate.h"
#include "d_fall_pps/CMCommand.h"
#include "nodes/CentralManagerService.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

#endif

#include <string>

#define N_MAX_CRAZYFLIES           20 // protection number

#ifdef CATKIN_MAKE
using namespace d_fall_pps;
#endif

MainGUIWindow::MainGUIWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainGUIWindow)
{
    #ifdef CATKIN_MAKE
    _rosNodeThread = new rosNodeThread(argc, argv, "my_GUI");
    #endif
    ui->setupUi(this);
    _init();
}


MainGUIWindow::~MainGUIWindow()
{
    delete ui;
}

int MainGUIWindow::getTabIndexFromName(QString name)
{
    int found_name = -1;
    for(int i = 0; i < ui->tabWidget->count(); i++)
    {
        qDebug("name: %s", name.toStdString().c_str());
        qDebug("tabText: %s", ui->tabWidget->tabText(i).toStdString().c_str());
        if(name == ui->tabWidget->tabText(i))
        {
            found_name = i;
        }
    }
    return found_name;
}

void MainGUIWindow::doNumCrazyFlyZonesChanged(int n)
{
    // tabs number management, maybe do it in a different way so we dont have to remove and add everything?
    // first check if size of tabs is greater than size of vector or viceversa. Have we removed or added a zone?

    qDebug("tabWidgetCount : %d", ui->tabWidget->count());
    if(ui->tabWidget->count() > scene->crazyfly_zones.size())
    {
        // we removed one crazyfly_zone, n means index of the one we removed. Look for that index tab and remove it
        QString qstr = "CrazyFly ";
        qstr.append(QString::number(n+1));
        if(scene->crazyfly_zones.size() == 0)
        {
            ui->tabWidget->clear();
        }
        int found_index = getTabIndexFromName(qstr);
        if(found_index != -1)
        {
            ui->tabWidget->widget(found_index)->deleteLater();
            // ui->tabWidget->removeTab(found_index);
        }

        //  now unlink it from table also:
        #ifdef CATKIN_MAKE
        if(cf_linker->isCFZoneLinked(n))
        {
            cf_linker->unlink_cf_zone(n);
        }
        #endif
    }
    else if(ui->tabWidget->count() < scene->crazyfly_zones.size())
    {
        // we added one crazyfly_zone, n means index of the new one. New tab will be labeld index + 1
        QString qstr = "CrazyFly ";
        qstr.append(QString::number(n+1));
        crazyFlyZoneTab* widget = new crazyFlyZoneTab(n);
        ui->tabWidget->insertTab(n, widget, qstr);
        connect(widget, SIGNAL(centerButtonClickedSignal(int)), this, SLOT(centerViewIndex(int)));
    }

    updateComboBoxesCFZones();
}

void MainGUIWindow::_init()
{
    // INITIALISE VARIOUS GUI ELEMENTS

	// The "CHECKBOX" for whether to display crazyflie icons
	ui->checkBox_vicon_crazyflies->setEnabled(true);
    ui->checkBox_vicon_crazyflies->setCheckable(true);
    ui->checkBox_vicon_crazyflies->setChecked(true);

    // The "SPIN BOX" for specifying the scaling of crazyflie icons
    ui->scaleSpinBox->setRange(0.1, 100);
    ui->scaleSpinBox->setSingleStep(0.1);
    ui->scaleSpinBox->setValue(5);
    ui->scaleSpinBox->setEnabled(true);

    // The "CHECKBOX" for whether to display markers
    ui->checkBox_vicon_markers->setEnabled(true);
	ui->checkBox_vicon_markers->setCheckable(true);
    ui->checkBox_vicon_markers->setChecked(true);

    // The "CHECKBOX" for whether to highlight the displayed markers
    ui->checkBox_vicon_highlight_markers->setEnabled(true);
    ui->checkBox_vicon_highlight_markers->setCheckable(true);
    ui->checkBox_vicon_highlight_markers->setChecked(true);

    // The "CHECKBOX" for whether to display the Crazyflie Zones
    ui->checkBox_crazyfly_zones->setEnabled(true);
    ui->checkBox_crazyfly_zones->setCheckable(true);
    ui->checkBox_crazyfly_zones->setChecked(true);

    // The "CHECKBOX" for whether to display the x-y grid
    ui->checkBox_grid->setEnabled(true);
    ui->checkBox_grid->setCheckable(true);
    ui->checkBox_grid->setChecked(true);

    // The "CHECKBOX" for whether to display the Table
    ui->checkBox_table->setEnabled(true);
    ui->checkBox_table->setCheckable(true);
    ui->checkBox_table->setChecked(true);


    

    
    


    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    // error messages
    // ui->err_message_cf->hide();
    // ui->err_message_cf_zone->hide();
    // ui->err_message_student_id->hide();

    ui->radioAddress_text->setReadOnly(true);

    QPalette *palette = new QPalette();
    palette->setColor(QPalette::Base,Qt::lightGray);
    palette->setColor(QPalette::Text,Qt::darkGray);
    ui->radioAddress_text->setPalette(*palette);

    ui->err_message_cf->setStyleSheet("QLabel { color : red; }");
    ui->err_message_cf_zone->setStyleSheet("QLabel { color : red; }");
    ui->err_message_student_id->setStyleSheet("QLabel { color : red; }");
    ui->err_message_radio_address->setStyleSheet("QLabel { color : red; }");

    ui->err_message_cf->clear();
    ui->err_message_cf_zone->clear();
    ui->err_message_student_id->clear();
    ui->err_message_radio_address->clear();

    // initialize table_links
    ui->table_links->setColumnCount(4);

    QFont fnt;
    fnt.setPointSize(7);
    ui->table_links->horizontalHeader()->setFont(fnt);

    ui->table_links->horizontalHeader()->setDefaultSectionSize(90);
    ui->table_links->verticalHeader()->setDefaultSectionSize(20);

    const int rowCount = ui->table_links->rowCount();
    const int columnCount = ui->table_links->columnCount();
    for(int i = 0; i < rowCount; ++i)
    {
    	for(int j = 0; j < columnCount; ++j)
        {
    		QTableWidgetItem* selectedItem = ui->table_links->item(i, j);
    		selectedItem->setFont(fnt);
    	}
    }
    ui->table_links->setSelectionBehavior(QAbstractItemView::SelectRows);
    QStringList horizontal_header;
    horizontal_header << "Student ID" << "CrazyFly" << "CrazyFly Zone" << "Radio Address";
    ui->table_links->setHorizontalHeaderLabels(horizontal_header);

    // scene
    scene = new myGraphicsScene(ui->frame_drawing);
    scene->setSceneRect(-100 * FROM_METERS_TO_UNITS, -100 * FROM_METERS_TO_UNITS, 200 * FROM_METERS_TO_UNITS, 200 * FROM_METERS_TO_UNITS);

    ui->graphicsView->setScene(scene);

    // after scene is created, create CFlinker
    #ifdef CATKIN_MAKE
    cf_linker = new CFLinker(ui, &crazyflies_vector, &scene->crazyfly_zones);
    #endif
    // connections
    QObject::connect(ui->tabWidget, SIGNAL(tabCloseRequested(int)), this, SLOT(doTabClosed(int)));
    QObject::connect(scene, SIGNAL(numCrazyFlyZonesChanged(int)), this, SLOT(doNumCrazyFlyZonesChanged(int)));
    QObject::connect(scene, SIGNAL(crazyFlyZoneSelected(int)), this, SLOT(setTabIndex(int)));
    QObject::connect(scene, SIGNAL(modeChanged(int)), this, SLOT(transitionToMode(int)));
    QObject::connect(scene, SIGNAL(numTablePiecesChanged(int)), this, SLOT(handleTablePiecesNumChanged(int)));

    // Add keyboard shortcuts
    // > for "all motors off", press the space bar
    ui->all_motors_off_button->setShortcut(tr("Space"));
    // > for "kill GUI node", press "CTRL+C" while the GUI window is the focus
    QShortcut* close_GUI_shortcut = new QShortcut(QKeySequence(tr("CTRL+C")), this, SLOT(close()));


    #ifdef CATKIN_MAKE
    _rosNodeThread->init();
    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));
    QObject::connect(cf_linker, SIGNAL(updateComboBoxes()), this, SLOT(updateComboBoxes()));


    ros_namespace = ros::this_node::getNamespace();

    ros::NodeHandle nodeHandle("~");
    DBChangedPublisher = nodeHandle.advertise<std_msgs::Int32>("DBChanged", 1);
    emergencyStopPublisher = nodeHandle.advertise<std_msgs::Int32>("emergencyStop", 1);

    // Initialise the publisher for sending "commands" from here (the master)
    // to all of the agent nodes
    commandAllAgentsPublisher = nodeHandle.advertise<std_msgs::Int32>("commandAllAgents", 1);

    // Publisher for sending a request from here (the master) to all "Parameter Service" nodes
    // that it should re-load parameters from the YAML file for the controllers.
    // > This is recieved and acted on by both Coordinate and Agent type "Parameter Services",
    // > A coordinator type "Parameter Service" will subsequently request the agents to fetch
    //   the parameters from itself.
    // > A agent type "Parameter Service" will subsequently request its own agent to fetch
    //   the parameters from itself.
    requestLoadControllerYamlPublisher = nodeHandle.advertise<std_msgs::Int32>("requestLoadControllerYaml", 1);
    
    // Initialise the publisher for sending a request from here (the master)
    // to all of the agents nodes that they should (re/dis)-connect from
    // the Crazy-Radio
    crazyRadioCommandAllAgentsPublisher = nodeHandle.advertise<std_msgs::Int32>("crazyRadioCommandAllAgents", 1);
    #endif
}

void MainGUIWindow::doTabClosed(int tab_index)
{
    QString name = ui->tabWidget->tabText(tab_index);
    #ifdef CATKIN_MAKE
    int cf_zone_index = cf_linker->getCFZoneIndexFromName(name);
    scene->removeCrazyFlyZone(cf_zone_index);
    #endif
}

void MainGUIWindow::setTabIndex(int index)
{
    QString qstr = "CrazyFly ";
    qstr.append(QString::number(index + 1));
    ui->tabWidget->setCurrentIndex(getTabIndexFromName(qstr));
}

void MainGUIWindow::updateComboBoxes()
{
    updateComboBoxesCFs();
    updateComboBoxesCFZones();
}
void MainGUIWindow::updateComboBoxesCFs()
{
    #ifdef CATKIN_MAKE
    ui->comboBoxCFs->clear();
    for(int i = 0; i < crazyflies_vector.size(); i++)
    {
        if(!cf_linker->isCFLinked(crazyflies_vector[i]->getName()))
        {
            QString qstr = QString::fromStdString(crazyflies_vector[i]->getName());
            ui->comboBoxCFs->addItem(qstr);
        }
    }
    #endif
}

void MainGUIWindow::updateComboBoxesCFZones()
{
    ui->comboBoxCFZones->clear();
    #ifdef CATKIN_MAKE
    for(int i = 0; i < scene->crazyfly_zones.size(); i++)
    {
    	// THIS IF CHECK WAS COMMENTED OUT TO ALLOW ASSIGNING MULTIPLE CRAYZFLIES TO ONE ZONE
        //if(!cf_linker->isCFZoneLinked(scene->crazyfly_zones[i]->getIndex()))
        //{
            int cf_zone_index = scene->crazyfly_zones[i]->getIndex();
            QString qstr = "CrazyFlyZone ";
            qstr.append(QString::number(cf_zone_index + 1));
            ui->comboBoxCFZones->addItem(qstr);
        //}
    }
    #endif
}


#ifdef CATKIN_MAKE
void MainGUIWindow::updateNewViconData(const ptrToMessage& p_msg) //connected to newViconData, from node
{

    // update Markers

    if(p_msg->markers.size() < markers_vector.size()) // some markers have dissapeared, received stuff is smaller than what we have
    {
        for(int i = p_msg->markers.size(); i < markers_vector.size(); i++)
        {
            scene->removeItem(markers_vector[i]); // remove objects from scene
            // ROS_INFO_STREAM("element index: " << i << " removed");
        }
        markers_vector.erase(markers_vector.begin() + p_msg->markers.size(), markers_vector.end()); //delete them
    }

    // ROS_INFO_STREAM("markers.size: " << p_msg->markers.size());

    for(int i = 0; i < p_msg->markers.size(); i++) // here, or new markers message is equal to current messages, or greater (some new markers)
    {
        if(i >= markers_vector.size()) //some new markers coming
        {
            // ROS_INFO_STREAM("element index: " << i << " added");
            Marker* tmp_p_marker = new Marker(&(p_msg->markers[i]));
            markers_vector.push_back(tmp_p_marker); // what happens with the new indexes? check if this is correct

            if(ui->checkBox_vicon_markers->checkState() == Qt::Checked) //only if markers checkbox info is checked..
            {
                scene->addItem(markers_vector[i]);
                if(ui->checkBox_vicon_highlight_markers->checkState() == Qt::Checked)
                {
                    markers_vector[i]->setHighlighted();
                }
            }
        }
        else
        {
            // ROS_INFO_STREAM("element index: " << i << " moved, already existed");
            markers_vector[i]->updateMarker(&(p_msg->markers[i]));
        }
    }

    // update Crazyflies
    // also: what happens if we dont go through one of the names? we need to remove that crazyfly
    int crazyfly_vector_size_before = crazyflies_vector.size(); //initial size of vector
    // in this loop, add new ones and update old ones
    for(int i = 0; i < p_msg->crazyflies.size(); i++)
    {
        bool name_found = false; // for each iteration, name_found starts in false
        int index_name_found;
        for(int j = 0; j < crazyfly_vector_size_before; j++)
        {
            if(crazyflies_vector[j]->getName() == p_msg->crazyflies[i].crazyflieName)
            {
                name_found = true; // name found. This can only happen once per i-iteration, names are unique
                index_name_found = j; // index in already existing vector, to update it later (really needed?)
            }
        }

        if(name_found)
        {
            crazyflies_vector[index_name_found]->updateCF(&(p_msg->crazyflies[i]));
        }
        else                    //name not found, newly arrived, add it to the vector
        {
            // now, if name follows our format, put the corresponding number. If not, put the unknown image
            std::string s = p_msg->crazyflies[i].crazyflieName;
            std::smatch m;
            std::regex e ("PPS_CF([0-9]{2})");

            QString filename(":/images/drone_fixed_");

            if(std::regex_search(s, m, e))
            {
                std::string found_string = m[1].str();
                filename.append(QString::fromStdString(found_string));
                filename.append(".svg");
            }
            else
            {
                filename.append("unk.svg");
            }

            crazyFly* tmp_p_crazyfly = new crazyFly(&(p_msg->crazyflies[i]), filename);
            tmp_p_crazyfly->setScaleCFs(ui->scaleSpinBox->value());
            crazyflies_vector.push_back(tmp_p_crazyfly);
        }

        if(ui->checkBox_vicon_crazyflies->checkState() == Qt::Checked)
        {
            for(int i = 0; i < crazyflies_vector.size(); i++) //check for occlussion
            {
                if(crazyflies_vector[i]->isOccluded())
                {
                    // ROS_INFO("===================OCCLUDED");
                    if(crazyflies_vector[i]->isAddedToScene())
                    {
                        scene->removeItem(crazyflies_vector[i]);
                        crazyflies_vector[i]->setAddedToScene(false);
                    }
                }
                else
                {
                    if(!crazyflies_vector[i]->isAddedToScene())
                    {
                        scene->addItem(crazyflies_vector[i]);
                        crazyflies_vector[i]->setAddedToScene(true);
                    }
                }
            }
        }
    }

    // in this loop, clean the ones that are not present anymore. UPDATE: this will apparently only happen when we tick and untick in Vicon
    int crazyfly_vector_size_after = crazyflies_vector.size();

    for(int j = crazyfly_vector_size_after - 1; j >= 0; j--)
    {
        bool name_found = false;
        for(int i = 0; i < p_msg->crazyflies.size(); i++)
        {
            if(crazyflies_vector[j]->getName() == p_msg->crazyflies[i].crazyflieName)
            {
                name_found = true;
            }
        }
        if(!name_found)
        {
            scene->removeItem(crazyflies_vector[j]);
            crazyflies_vector.erase(crazyflies_vector.begin() + j);
        }
    }
}
#endif


void MainGUIWindow::on_removeTable_clicked()
{
    if(scene->getMode() == myGraphicsScene::mode_table)
    {
        scene->removeTable();
    }
}

void MainGUIWindow::transitionToMode(int mode)
{
    switch(mode)
    {
        case myGraphicsScene::mode_table:
        {
            ui->removeTable->setDisabled(false);
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            ui->removeTable->setDisabled(true);
            break;
        }
    }
}

void MainGUIWindow::on_radioButton_table_mode_toggled(bool checked)
{
    switch(scene->getMode())
    {
        case myGraphicsScene::mode_table:
        {
            // already in the mode we want, do nothing
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            scene->setMode(myGraphicsScene::mode_table);
            break;
        }
        case myGraphicsScene::mode_locked:
        {
            scene->setMode(myGraphicsScene::mode_table);
            break;
        }
    }

}


void MainGUIWindow::on_radioButton_crazyfly_zones_mode_toggled(bool checked)
{
    switch(scene->getMode())
    {
        case myGraphicsScene::mode_table:
        {
            scene->setMode(myGraphicsScene::mode_crazyfly_zones);
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            // already in the mode we want, do nothing
            break;
        }
        case myGraphicsScene::mode_locked:
        {
            scene->setMode(myGraphicsScene::mode_crazyfly_zones);
            break;
        }
    }
}

void MainGUIWindow::handleTablePiecesNumChanged(int newNum)
{

}

void MainGUIWindow::on_radioButton_lock_mode_toggled(bool checked)
{
    switch(scene->getMode())
    {
        case myGraphicsScene::mode_table:
        {
            scene->setMode(myGraphicsScene::mode_locked);
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            scene->setMode(myGraphicsScene::mode_locked);
            break;
        }
        case myGraphicsScene::mode_locked:
        {
            break;
        }
    }
}

void MainGUIWindow::on_checkBox_grid_toggled(bool checked)
{
    scene->setGrid(checked);
}

void MainGUIWindow::on_checkBox_table_toggled(bool checked)
{
    if(checked)
    {
        scene->showTable();
    }
    else
    {
        scene->hideTable();
    }
}

void MainGUIWindow::on_checkBox_crazyfly_zones_toggled(bool checked)
{
    if(checked)
    {
        scene->showCrazyFlyZones();
    }
    else
    {
        scene->hideCrazyFlyZones();
    }
}

void MainGUIWindow::on_tabWidget_currentChanged(int index)
{
    // // this index is tab index. Need to go to cf index
    // QString name = ui->tabWidget->tabText(index);
    // #ifdef CATKIN_MAKE
    // int cf_index = cf_linker->getCFZoneIndexFromName(name);
    // scene->setSelectedCrazyFlyZone(cf_index);
    // #endif
}

void MainGUIWindow::centerViewIndex(int index)
{
    ui->graphicsView->fitInView(scene->getRectFCrazyFlyZone(index), Qt::KeepAspectRatio);
    ui->graphicsView->scale(0.95, 0.95); // A bit back zoom, so we can see everything better
}


void MainGUIWindow::on_pushButton_fitAll_clicked()
{
    ui->graphicsView->fitInView(scene->itemsBoundingRect(), Qt::KeepAspectRatio);
    ui->graphicsView->scale(0.95, 0.95); // A bit back zoom, so we can see everything better
}

void MainGUIWindow::on_checkBox_vicon_markers_toggled(bool checked)
{
    if(checked)
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            scene->addItem(markers_vector[i]);
        }
        #endif
        ui->checkBox_vicon_highlight_markers->setCheckable(true);
        ui->checkBox_vicon_highlight_markers->setEnabled(true);
    }
    else
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            scene->removeItem(markers_vector[i]);
        }
        #endif
        ui->checkBox_vicon_highlight_markers->setChecked(false);
        ui->checkBox_vicon_highlight_markers->setCheckable(false);
        ui->checkBox_vicon_highlight_markers->setEnabled(false);
    }
}

void MainGUIWindow::on_checkBox_vicon_highlight_markers_toggled(bool checked)
{
    if(checked)
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            markers_vector[i]->setHighlighted();
        }
        #endif
    }
    else
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            markers_vector[i]->clearHighlighted();
        }
        #endif
    }
}

void MainGUIWindow::on_checkBox_vicon_crazyflies_toggled(bool checked)
{
    if(checked)
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < crazyflies_vector.size(); i++)
        {
            if(!crazyflies_vector[i]->isAddedToScene())
            {
                scene->addItem(crazyflies_vector[i]);
                crazyflies_vector[i]->setAddedToScene(true);
            }
        }
        #endif
        ui->scaleSpinBox->setEnabled(true);
    }
    else
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < crazyflies_vector.size(); i++)
        {
            if(crazyflies_vector[i]->isAddedToScene())
            {
                scene->removeItem(crazyflies_vector[i]);
                crazyflies_vector[i]->setAddedToScene(false);
            }
        }
        #endif
        ui->scaleSpinBox->setEnabled(false);
    }
}

void MainGUIWindow::on_scaleSpinBox_valueChanged(double arg1)
{
    #ifdef CATKIN_MAKE
    for(int i = 0; i < crazyflies_vector.size(); i++)
    {
        crazyflies_vector[i]->setScaleCFs(arg1);
    }
    #endif
}

void MainGUIWindow::on_refresh_cfs_button_clicked()
{
    updateComboBoxesCFs();
}

void MainGUIWindow::on_refresh_student_ids_button_clicked()
{
    #ifdef CATKIN_MAKE
    ui->list_discovered_student_ids->clear();

    // \/(\d)\/PPSClient
    ros::V_string v_str;
    ros::master::getNodes(v_str);
    for(int i = 0; i < v_str.size(); i++)
    {
        std::string s = v_str[i];
        std::smatch m;
        //std::regex e ("\\/(\\d)\\/PPSClient");
        std::regex e ("\\/agent(\\d\\d\\d)\\/PPSClient");

        // std::regex e("\\/PPSClien(.)");

        // while(std::regex_search(s, m, e))
        // {
        //     for (int i = 0; i < m.size(); i++)
        //     {
        //         ROS_INFO("FOUND: %s", m[i].str().c_str());
        //         // std::cout << "FOUND" << m[i] << "\n";
        //     }
        //     s = m.suffix().str();
        // }

        if(std::regex_search(s, m, e))
        {
            // ROS_INFO("===============================================FOUND: %s", m[1].str().c_str()); // one because we are interested ONLY in the first match
            std::string found_string = m[1].str();
            ui->list_discovered_student_ids->addItem(found_string.c_str());
        }
    }
    #endif
}



void MainGUIWindow::on_link_button_clicked()
{
    #ifdef CATKIN_MAKE

    bool error = false;
    if(ui->comboBoxCFs->count() == 0)
    {
        // plot error message
        ui->err_message_cf->setText("CF box is empty");
        error = true;
    }
    else
    {
        ui->err_message_cf->clear();
    }

    if(ui->comboBoxCFZones->count() == 0)
    {
        // plot error message
        ui->err_message_cf_zone->setText("CFZone box is empty");
        error = true;
    }
    else
    {
        ui->err_message_cf_zone->clear();
    }

    if(cf_linker->isRadioAddressLinked(ui->radioAddress_text->text().toStdString()))
    {
        ui->err_message_radio_address->setText("Already in use");
        error = true;
    }
    else if(ui->radioAddress_text->text().toStdString() == "")
    {
        ui->err_message_radio_address->setText("Field is empty");
        error = true;
    }
    else
    {
        ui->err_message_radio_address->clear();
    }

    if(cf_linker->isStudentIDLinked(ui->spinBox_student_ids->value()))
    {
        // plot error message
        ui->err_message_student_id->setText("This StudentID has already been linked");
        error = true;
    }
    else
    {
        ui->err_message_student_id->clear();
    }

    if(!error)
    {
        cf_linker->link(ui->spinBox_student_ids->value(), cf_linker->getCFZoneIndexFromName(ui->comboBoxCFZones->currentText()), ui->comboBoxCFs->currentText().toStdString(), ui->radioAddress_text->text().toStdString());
    }
    #endif
}

void MainGUIWindow::on_unlink_button_clicked()
{
    #ifdef CATKIN_MAKE
    cf_linker->unlink_selection();
    #endif
}

void MainGUIWindow::on_save_in_DB_button_clicked()
{
    // we need to update and then save?
    CrazyflieDB tmp_db;
    for(int i = 0; i < cf_linker->links.size(); i++)
    {
        CrazyflieEntry tmp_entry;
        tmp_entry.crazyflieContext.crazyflieName = cf_linker->links[i].cf_name;
        tmp_entry.crazyflieContext.crazyflieAddress = cf_linker->links[i].radio_address;
        tmp_entry.crazyflieContext.localArea.crazyfly_zone_index = cf_linker->links[i].cf_zone_index;
        tmp_entry.studentID = cf_linker->links[i].student_id;

        for(int j = 0; j < scene->crazyfly_zones.size(); j++)
        {
            if(cf_linker->links[i].cf_zone_index == scene->crazyfly_zones[j]->getIndex())
            {
                double x_min = scene->crazyfly_zones[j]->sceneBoundingRect().bottomLeft().x();
                double y_min = - scene->crazyfly_zones[j]->sceneBoundingRect().bottomLeft().y();

                double x_max = scene->crazyfly_zones[j]->sceneBoundingRect().topRight().x();
                double y_max = -scene->crazyfly_zones[j]->sceneBoundingRect().topRight().y();

                tmp_entry.crazyflieContext.localArea.xmin = x_min * FROM_UNITS_TO_METERS;
                tmp_entry.crazyflieContext.localArea.xmax = x_max * FROM_UNITS_TO_METERS;
                tmp_entry.crazyflieContext.localArea.ymin = y_min * FROM_UNITS_TO_METERS;
                tmp_entry.crazyflieContext.localArea.ymax = y_max * FROM_UNITS_TO_METERS;

                tmp_entry.crazyflieContext.localArea.zmin = -0.2;
                tmp_entry.crazyflieContext.localArea.zmax = 1.2;
            }
        }
        tmp_db.crazyflieEntries.push_back(tmp_entry);
    }

    m_data_base = tmp_db;

    ROS_INFO_STREAM("database:\n" << m_data_base);

    // save the database in the file

    fill_database_file();

    // Now also publish a ROS message stating that we changed the DB, so the nodes can update it
    std_msgs::Int32 msg;
    msg.data = 1;
    this->DBChangedPublisher.publish(msg);
}

void MainGUIWindow::clear_database_file()
{
    CrazyflieDB tmp_db;
    if(read_database_from_file(tmp_db) == 0)
    {
        for(int i = 0; i < tmp_db.crazyflieEntries.size(); i++)
        {
            CMUpdate updateCall;
            updateCall.request.mode = ENTRY_REMOVE;
            updateCall.request.crazyflieEntry.crazyflieContext.crazyflieName = tmp_db.crazyflieEntries[i].crazyflieContext.crazyflieName;
            if(_rosNodeThread->m_update_db_client.call(updateCall))
            {
                ROS_INFO("database changed in central manager service");
            }
            else
            {
                ROS_ERROR("Failed to remove entry in DB");
            }
        }
        save_database_file();
    }
    else
    {
        ROS_INFO("Failed to read DB");
    }
}

void MainGUIWindow::fill_database_file()
{
    clear_database_file();
    ROS_INFO("cleared data base file");
    ROS_INFO_STREAM("database:\n" << m_data_base);
    for(int i = 0; i < m_data_base.crazyflieEntries.size(); i++)
    {
        ROS_INFO("inserted 1 item in DB");
        insert_or_update_entry_database(m_data_base.crazyflieEntries[i]);
    }
    save_database_file();
}

void MainGUIWindow::save_database_file()
{
    CMCommand commandCall;
    commandCall.request.command = CMD_SAVE;
    if(_rosNodeThread->m_command_db_client.call(commandCall))
    {
        ROS_INFO("successfully saved db");
    }
    else
    {
        ROS_ERROR("failed to save db");
    }
}

void MainGUIWindow::insert_or_update_entry_database(CrazyflieEntry entry)
{
    CMUpdate updateCall;
    updateCall.request.mode = ENTRY_INSERT_OR_UPDATE;
    updateCall.request.crazyflieEntry = entry;
    _rosNodeThread->m_update_db_client.call(updateCall);
}

int MainGUIWindow::read_database_from_file(CrazyflieDB &read_db)
{
    CMRead getDBCall;
    _rosNodeThread->m_read_db_client.waitForExistence(ros::Duration(-1));
    if(_rosNodeThread->m_read_db_client.call(getDBCall))
    {
        read_db = getDBCall.response.crazyflieDB;
        return 0;
    }
    else
    {
        return -1;
    }
}

void MainGUIWindow::on_load_from_DB_button_clicked()
{
    CrazyflieDB tmp_db;

    if(read_database_from_file(tmp_db) == 0)
    {
		ROS_INFO_STREAM("database:\n" << tmp_db);
        m_data_base = tmp_db;

        cf_linker->clear_all_links();
        // remove all cf_zones existing
        scene->removeAllCFZones();

        int size = scene->crazyfly_zones.size();
        ROS_INFO("vector_cf_zones_size %d", size);

        for(int i = 0; i < m_data_base.crazyflieEntries.size(); i++)
        {
        	// Get the CF name from the database
            std::string cf_name = m_data_base.crazyflieEntries[i].crazyflieContext.crazyflieName;
            // Get the radio address from the database
            std::string radio_address = m_data_base.crazyflieEntries[i].crazyflieContext.crazyflieAddress;
            // Get the CF Zone Index from the database
            int cf_zone_index = m_data_base.crazyflieEntries[i].crazyflieContext.localArea.crazyfly_zone_index;


            // THIS WAS ADDED TO ALLOW ASSIGNING MULTIPLE CRAYZFLIES TO ONE ZONE
            // Check if a Zone with this index already exists.
            bool cf_zone_exists;
            scene->crazyfly_zones.size();
            for(int i = 0; i < scene->crazyfly_zones.size(); i++)
		    {
	            int existing_cf_zone_index = scene->crazyfly_zones[i]->getIndex();
	            if (cf_zone_index==existing_cf_zone_index)
	            {
	            	cf_zone_exists = true;
	            }
		    }

            // we should first create the cf zones that are in the database?
            if (!cf_zone_exists)
            {
            	// Get the size of this Zone from the database
	            qreal width = m_data_base.crazyflieEntries[i].crazyflieContext.localArea.xmax - m_data_base.crazyflieEntries[i].crazyflieContext.localArea.xmin;
	            qreal height = m_data_base.crazyflieEntries[i].crazyflieContext.localArea.ymax - m_data_base.crazyflieEntries[i].crazyflieContext.localArea.ymin;
	            // Create a rectangle for disaplying the zone
	            QRectF tmp_rect(m_data_base.crazyflieEntries[i].crazyflieContext.localArea.xmin * FROM_METERS_TO_UNITS,
	                            - m_data_base.crazyflieEntries[i].crazyflieContext.localArea.ymax * FROM_METERS_TO_UNITS, // minus sign because qt has y-axis inverted
	                            width * FROM_METERS_TO_UNITS,
	                            height * FROM_METERS_TO_UNITS);
	            // Add the zone to the scene
	            scene->addCFZone(tmp_rect, cf_zone_index);
        	}

        	// Get the student ID for this entry in the database
        	int student_id = m_data_base.crazyflieEntries[i].studentID;

        	// Create the link
            cf_linker->link(student_id, cf_zone_index, cf_name, radio_address);

        }


        // THIS WAS ADDED TO ALLOW ASSIGNING MULTIPLE CRAYZFLIES TO ONE ZONE
        // Update the CF Zone combo box with the zones just added
        updateComboBoxesCFZones();
    }
    else
    {
        ROS_ERROR("Failed to read DB");
    }
}

void MainGUIWindow::on_comboBoxCFs_currentTextChanged(const QString &arg1)
{
    std::string key = arg1.toStdString();
    auto it = channel_LUT.find(key);
    if(it != channel_LUT.end())
    {
        std::string found = it->second;
        QString found_qstr = QString::fromStdString(found);
        ui->radioAddress_text->setText(found_qstr);
    }
    else
    {
        ROS_INFO("name not found in LUT");
    }
}





//    ----------------------------------------------------------------------------------
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD          A    L      L
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D        A A   L      L
//    C      O   O  M M M  M M M  A   A  N N N  D   D       A   A  L      L
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D       AAAAA  L      L
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD        A   A  LLLLL  LLLLL
//
//    BBBB   U   U  TTTTT  TTTTT   OOO   N   N   SSSS
//    B   B  U   U    T      T    O   O  NN  N  S
//    BBBB   U   U    T      T    O   O  N N N   SSS
//    B   B  U   U    T      T    O   O  N  NN      S
//    BBBB    UUU     T      T     OOO   N   N  SSSS
//    ----------------------------------------------------------------------------------


// For the buttons that commands all of the agent nodes to:
// > (RE)CONNECT THE RADIO
void MainGUIWindow::on_all_connect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_RECONNECT;
    crazyRadioCommandAllAgentsPublisher.publish(msg);
}
// > DISCONNECT THE RADIO
void MainGUIWindow::on_all_disconnect_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_DISCONNECT;
    crazyRadioCommandAllAgentsPublisher.publish(msg);
}
// > TAKE-OFF
void MainGUIWindow::on_all_take_off_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_CRAZYFLY_TAKE_OFF;
    commandAllAgentsPublisher.publish(msg);
}
// > LAND
void MainGUIWindow::on_all_land_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_CRAZYFLY_LAND;
    commandAllAgentsPublisher.publish(msg);
}
// > MOTORS OFF
void MainGUIWindow::on_all_motors_off_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_CRAZYFLY_MOTORS_OFF;
    commandAllAgentsPublisher.publish(msg);
    //emergencyStopPublisher.publish(msg);
}
// > ENABLE SAFE CONTROLLER
void MainGUIWindow::on_all_enable_safe_controller_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_SAFE_CONTROLLER;
    commandAllAgentsPublisher.publish(msg);
}
// > ENABLE SAFE CONTROLLER
void MainGUIWindow::on_all_enable_custom_controller_button_clicked()
{
    std_msgs::Int32 msg;
    msg.data = CMD_USE_CUSTOM_CONTROLLER;
    commandAllAgentsPublisher.publish(msg);
}
// > LOAD THE YAML PARAMETERS FOR THE SAFE CONTROLLER
void MainGUIWindow::on_all_load_safe_controller_yaml_own_agent_button_clicked()
{
	// Disable the button
	ui->all_load_safe_controller_yaml_own_agent_button->setEnabled(false);
	ui->all_load_safe_controller_yaml_coordinator_button->setEnabled(false);

	// Send the message that the YAML paremters should be loaded
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_SAFE_CONTROLLER_AGENT;
    requestLoadControllerYamlPublisher.publish(msg);

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_safe_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainGUIWindow::safeYamlFileTimerCallback, this, true);
}
// > LOAD THE YAML PARAMETERS FOR THE CUSTOM CONTROLLER
void MainGUIWindow::on_all_load_custom_controller_yaml_own_agent_button_clicked()
{
	// Disable the button
	ui->all_load_custom_controller_yaml_own_agent_button->setEnabled(false);
	ui->all_load_custom_controller_yaml_coordinator_button->setEnabled(false);

	// Send the message that the YAML paremters should be loaded
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_CUSTOM_CONTROLLER_AGENT;
    requestLoadControllerYamlPublisher.publish(msg);

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_custom_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainGUIWindow::customYamlFileTimerCallback, this, true);

}
// > SEND THE YAML PARAMETERS FOR THE SAFE CONTROLLER
void MainGUIWindow::on_all_load_safe_controller_yaml_coordinator_button_clicked()
{
	// Disable the button
	ui->all_load_safe_controller_yaml_own_agent_button->setEnabled(false);
	ui->all_load_safe_controller_yaml_coordinator_button->setEnabled(false);

	// Send the message that the YAML paremters should be loaded
    // by the coordinator (and then the agent informed)
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_SAFE_CONTROLLER_COORDINATOR;
    requestLoadControllerYamlPublisher.publish(msg);

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_safe_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainGUIWindow::safeYamlFileTimerCallback, this, true);
}
// > SEND THE YAML PARAMETERS FOR THE CUSTOM CONTROLLER
void MainGUIWindow::on_all_load_custom_controller_yaml_coordinator_button_clicked()
{
	// Disable the button
	ui->all_load_custom_controller_yaml_own_agent_button->setEnabled(false);
	ui->all_load_custom_controller_yaml_coordinator_button->setEnabled(false);

    // Send the message that the YAML paremters should be loaded
    // by the coordinator (and then the agent informed)
    std_msgs::Int32 msg;
    msg.data = LOAD_YAML_CUSTOM_CONTROLLER_COORDINATOR;
    requestLoadControllerYamlPublisher.publish(msg);

    // Start a timer which will enable the button in its callback
    // > This is required because the agent node waits some time between
    //   re-loading the values from the YAML file and then assigning then
    //   to the local variable of the agent.
    // > Thus we use this timer to prevent the user from clicking the
    //   button in the GUI repeatedly.
    ros::NodeHandle nodeHandle("~");
    m_timer_yaml_file_for_custom_controller = nodeHandle.createTimer(ros::Duration(1.5), &MainGUIWindow::customYamlFileTimerCallback, this, true);

}
// > CALLBACK TO RE-ENABLE THE SAFE CONTROLLER YAML BUTTONS
void MainGUIWindow::safeYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load" and the "send" safe controller YAML button again
    ui->all_load_safe_controller_yaml_own_agent_button->setEnabled(true);
	ui->all_load_safe_controller_yaml_coordinator_button->setEnabled(true);
}
// > CALLBACK TO RE-ENABLE THE CUSTOM CONTROLLER YAML BUTTONS
void MainGUIWindow::customYamlFileTimerCallback(const ros::TimerEvent&)
{
    // Enble the "load" and the "send" custom controller YAML button again
    ui->all_load_custom_controller_yaml_own_agent_button->setEnabled(true);
	ui->all_load_custom_controller_yaml_coordinator_button->setEnabled(true);
}

/*
// > CALLBACK TO SEND THE CUSTOM CONTROLLER YAML PARAMETERS AS A MESSAGE
void MainGUIWindow::customSendYamlAsMessageTimerCallback(const ros::TimerEvent&)
{
	// Load the CUSTOM controller YAML parameters from file into a message for
	// sending directly to the "CustonControllerService" node of every agent

	ros::NodeHandle nodeHandle("~");

	// Instaniate a local variable of the message type
	CustomControllerYAML customControllerYamlMessage;

	// Load the data directly from the YAML file into the message

	// > For the mass
	customControllerYamlMessage.mass = MainGUIWindow::getParameterFloat(nodeHandle, "mass");

    // Debugging this this works
    ROS_INFO_STREAM("DEBUGGING: mass loaded from the custom controller YAML = " << customControllerYamlMessage.mass );

	// > For the control_frequency
	customControllerYamlMessage.control_frequency = MainGUIWindow::getParameterFloat(nodeHandle, "control_frequency");

	// > For the motorPoly coefficients
    std::vector<float> temp_motorPoly(3);
	MainGUIWindow::getParameterFloatVector(nodeHandle, "motorPoly", temp_motorPoly, 3);
    // Copy the loaded data into the message
    for ( int i=0 ; i<3 ; i++ )
    {
        customControllerYamlMessage.motorPoly.push_back( temp_motorPoly[i] );
    }

    // > For the boolean about whether to execute the convert into body frame function
    customControllerYamlMessage.shouldPerformConvertIntoBodyFrame = MainGUIWindow::getParameterBool(nodeHandle, "shouldPerformConvertIntoBodyFrame");

    // > For the boolean about publishing the agents current position
	customControllerYamlMessage.shouldPublishCurrent_xyz_yaw = MainGUIWindow::getParameterBool(nodeHandle, "shouldPublishCurrent_xyz_yaw");

	// > For the boolean about following another agent
	customControllerYamlMessage.shouldFollowAnotherAgent = MainGUIWindow::getParameterBool(nodeHandle, "shouldFollowAnotherAgent");

	// > For the ordered agent ID's for following eachother in a line
    std::vector<int> temp_follow_in_a_line_agentIDs(100);
	int temp_number_of_agents_in_a_line = MainGUIWindow::getParameterIntVectorWithUnknownLength(nodeHandle, "follow_in_a_line_agentIDs", temp_follow_in_a_line_agentIDs);
	// > Double check that the sizes agree
    if ( temp_number_of_agents_in_a_line != temp_follow_in_a_line_agentIDs.size() )
    {
    	// Update the user if the sizes don't agree
    	ROS_ERROR_STREAM("parameter 'follow_in_a_line_agentIDs' was loaded with two different lengths, " << temp_number_of_agents_in_a_line << " versus " << temp_follow_in_a_line_agentIDs.size() );
    }
    // Copy the loaded data into the message
    for ( int i=0 ; i<temp_number_of_agents_in_a_line ; i++ )
    {
        customControllerYamlMessage.follow_in_a_line_agentIDs.push_back( temp_follow_in_a_line_agentIDs[i] );
    }    

    // Publish the message containing the loaded YAML parameters
    customYAMLasMessagePublisher.publish(customControllerYamlMessage);

	// Start a timer which will enable the button in its callback
    m_timer_yaml_file_for_custom_controller = nodeHandle.createTimer(ros::Duration(0.5), &MainGUIWindow::customYamlFileTimerCallback, this, true);
}
*/




//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD        Y   Y    A    M   M  L
//    L      O   O   A A   D   D        Y Y    A A   MM MM  L
//    L      O   O  A   A  D   D         Y    A   A  M M M  L
//    L      O   O  AAAAA  D   D         Y    AAAAA  M   M  L
//    LLLLL   OOO   A   A  DDDD          Y    A   A  M   M  LLLLL
//
//    PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
//    P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
//    PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
//    P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
//    P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
//    ----------------------------------------------------------------------------------

// load parameters from corresponding YAML file
//
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
float MainGUIWindow::getParameterFloat(ros::NodeHandle& nodeHandle, std::string name)
{
    float val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void MainGUIWindow::getParameterFloatVector(ros::NodeHandle& nodeHandle, std::string name, std::vector<float>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
int MainGUIWindow::getParameterInt(ros::NodeHandle& nodeHandle, std::string name)
{
    int val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
void MainGUIWindow::getParameterIntVectorWithKnownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val, int length)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    if(val.size() != length) {
        ROS_ERROR_STREAM("parameter '" << name << "' has wrong array length, " << length << " needed");
    }
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
int MainGUIWindow::getParameterIntVectorWithUnknownLength(ros::NodeHandle& nodeHandle, std::string name, std::vector<int>& val)
{
    if(!nodeHandle.getParam(name, val)){
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val.size();
}
// This function DOES NOT NEED TO BE edited for successful completion of the PPS exercise
bool MainGUIWindow::getParameterBool(ros::NodeHandle& nodeHandle, std::string name)
{
    bool val;
    if(!nodeHandle.getParam(name, val))
    {
        ROS_ERROR_STREAM("missing parameter '" << name << "'");
    }
    return val;
}
