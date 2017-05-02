#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include "crazyFlyZoneTab.h"
#include "myGraphicsScene.h"
#include "myGraphicsView.h"

#include <QObject>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QString>

#include <string>

#define N_MAX_CRAZYFLIES           20 // protection number

using namespace d_fall_pps;

MainGUIWindow::MainGUIWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainGUIWindow),
    _rosNodeThread(argc, argv, "/ViconDataPublisher/ViconData")
{

    ui->setupUi(this);
    _init();
}


MainGUIWindow::~MainGUIWindow()
{
    delete ui;
}

void MainGUIWindow::set_tabs(int n)
{
    ui->tabWidget->clear();
    for (int i = 0; i < n; i++)
    {
        QString qstr = "CrazyFly ";
        qstr.append(QString::number(i+1));
        crazyFlyZoneTab* widget = new crazyFlyZoneTab(i);
        ui->tabWidget->addTab(widget, qstr);
        connect(widget, SIGNAL(centerButtonClickedSignal(int)), this, SLOT(centerViewIndex(int)));
    }
}

void MainGUIWindow::_init()
{
    // #ifdef CATKIN_MAKE
    // ViconSubscriber = new ros::Subscriber(_pNodeHandle->subscribe("/ViconDataPublisher/ViconData", 1, viconCallback));
    // ros::spin();
    // ROS_INFO("successfully subscribed to ViconData from GUI");
    // qDebug("successfully subscribed to ViconData from GUI");
    // #endif

    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    scene = new myGraphicsScene(ui->frame_drawing);
    scene->setSceneRect(-100 * TO_METERS, -100 * TO_METERS, 200 * TO_METERS, 200 * TO_METERS);

    // Marker* marker = new Marker(0, 0);
    // scene->addItem(marker);

    ui->graphicsView->setScene(scene);

    QObject::connect(ui->tabWidget, SIGNAL(tabCloseRequested(int)), scene, SLOT(removeCrazyFlyZone(int)));
    QObject::connect(scene, SIGNAL(numCrazyFlyZonesChanged(int)), this, SLOT(set_tabs(int)));
    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), scene, SLOT(setSelectedCrazyFlyZone(int)));
    QObject::connect(scene, SIGNAL(crazyFlyZoneSelected(int)), ui->tabWidget, SLOT(setCurrentIndex(int)));
    QObject::connect(scene, SIGNAL(modeChanged(int)), this, SLOT(transitionToMode(int)));
    QObject::connect(scene, SIGNAL(numTablePiecesChanged(int)), this, SLOT(handleTablePiecesNumChanged(int)));

    ui->checkBox_vicon_highlight_markers->setEnabled(false);

    _rosNodeThread.init();
}



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
    if(index >= 0)
    {
        scene->setSelectedCrazyFlyZone(index);
    }
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
    // This is temporal, just to see effect. In the end the marker will be created with data from vicon
    if(checked)
    {
        marker = new Marker(0, 0);
        scene->addItem(marker);
        ui->checkBox_vicon_highlight_markers->setCheckable(true);
        ui->checkBox_vicon_highlight_markers->setEnabled(true);
    }
    else
    {
        marker->setParentItem(NULL);
        delete marker;
        ui->checkBox_vicon_highlight_markers->setChecked(false);
        ui->checkBox_vicon_highlight_markers->setCheckable(false);
        ui->checkBox_vicon_highlight_markers->setEnabled(false);
    }
}

void MainGUIWindow::on_checkBox_vicon_highlight_markers_toggled(bool checked)
{
    if(checked)
    {
        marker->setHighlighted();
    }
    else
    {
        marker->clearHighlighted();
    }
}
