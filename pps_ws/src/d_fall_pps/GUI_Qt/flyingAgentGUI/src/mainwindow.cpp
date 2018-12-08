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
//    Coordinator GUI main window.
//
//    ----------------------------------------------------------------------------------

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
#ifdef CATKIN_MAKE
    //ROS_INFO("[flyingAgentGUI] Debug Point 5");
    m_rosNodeThread = new rosNodeThread(argc, argv, "flyingAgentGUI");
#endif

#ifdef CATKIN_MAKE
    m_rosNodeThread->init();
#endif
    ui->setupUi(this);

    // ADD KEYBOARD SHORTCUTS
    // > For "kill GUI node", press "CTRL+C" while the GUI window is the focus
    m_close_GUI_shortcut = new QShortcut(QKeySequence(tr("CTRL+C")), this, SLOT(close()));



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
        QString qstr = "Hide Coordinator";
        ui->actionShowHide_Coordinator->setText(qstr);
    }
    else
    {
        ui->customWidget_coordinator->hide();
        QString qstr = "Show Coordinator";
        ui->actionShowHide_Coordinator->setText(qstr);
    }
}


void MainWindow::on_action_LoadYAML_BatteryMonitor_triggered()
{
#ifdef CATKIN_MAKE
    // Send a message that the "BatteryMonitor" Yaml should be loaded
    // by the appropriate Parameter Service
#endif
}


void MainWindow::on_action_LoadYAML_ClientConfig_triggered()
{
#ifdef CATKIN_MAKE
    // Send a message that the "ClientConfig" Yaml should be loaded
    // by the appropriate Parameter Service
#endif
}
