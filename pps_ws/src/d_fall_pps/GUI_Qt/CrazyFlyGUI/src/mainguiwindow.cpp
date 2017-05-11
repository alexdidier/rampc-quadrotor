#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include "crazyFlyZoneTab.h"
#include "myGraphicsScene.h"
#include "myGraphicsView.h"

#include <QObject>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QString>
#include <QMetaType>

#include <string>

#define N_MAX_CRAZYFLIES           20 // protection number

#ifdef CATKIN_MAKE
using namespace d_fall_pps;
#endif

#ifdef CATKIN_MAKE
MainGUIWindow::MainGUIWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainGUIWindow)//,
    // _rosNodeThread(argc, argv, "/ViconDataPublisher/ViconData")
{
    _rosNodeThread = new rosNodeThread(argc, argv, "/ViconDataPublisher/UnlabeledMarkersArray");

    ui->setupUi(this);
    _init();
}
#else
MainGUIWindow::MainGUIWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainGUIWindow)
{

    ui->setupUi(this);
    _init();
}
#endif


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

    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    scene = new myGraphicsScene(ui->frame_drawing);
    scene->setSceneRect(-100 * FROM_METERS_TO_UNITS, -100 * FROM_METERS_TO_UNITS, 200 * FROM_METERS_TO_UNITS, 200 * FROM_METERS_TO_UNITS);

    marker = new Marker(1 * FROM_METERS_TO_UNITS, - 1 * FROM_METERS_TO_UNITS);
    marker->setPos(100, -200);

    ui->graphicsView->setScene(scene);

    QObject::connect(ui->tabWidget, SIGNAL(tabCloseRequested(int)), scene, SLOT(removeCrazyFlyZone(int)));
    QObject::connect(scene, SIGNAL(numCrazyFlyZonesChanged(int)), this, SLOT(set_tabs(int)));
    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), scene, SLOT(setSelectedCrazyFlyZone(int)));
    QObject::connect(scene, SIGNAL(crazyFlyZoneSelected(int)), ui->tabWidget, SLOT(setCurrentIndex(int)));
    QObject::connect(scene, SIGNAL(modeChanged(int)), this, SLOT(transitionToMode(int)));
    QObject::connect(scene, SIGNAL(numTablePiecesChanged(int)), this, SLOT(handleTablePiecesNumChanged(int)));

    ui->checkBox_vicon_highlight_markers->setEnabled(false);

    #ifdef CATKIN_MAKE
    _rosNodeThread->init();
    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(setPosMarkers(const ptrToMessage&)));
    #endif
}

#ifdef CATKIN_MAKE
void MainGUIWindow::setPosMarkers(const ptrToMessage& p_msg) //connected to newViconData, from node
{

    if(p_msg->markers.size() < markers_vector.size()) // some markers have dissapeared, received stuff is smaller than what we have
    {
        for(int i = p_msg->markers.size(); i < markers_vector.size(); i++)
        {
            scene->removeItem(markers_vector[i]); // remove objects from scene
            ROS_INFO_STREAM("element index: " << i << " removed");
        }
        markers_vector.erase(markers_vector.begin() + p_msg->markers.size(), markers_vector.end()); //delete them
    }

    ROS_INFO_STREAM("markers.size: " << p_msg->markers.size());

    for(int i = 0; i < p_msg->markers.size(); i++) // here, or new markers message is equal to current messages, or greater (some new markers)
    {
        if(i >= markers_vector.size()) //some new markers coming
        {
            ROS_INFO_STREAM("element index: " << i << " added");
            Marker* tmp_p_marker = new Marker(p_msg->markers[i].x * FROM_METERS_TO_UNITS, p_msg->markers[i].y * FROM_METERS_TO_UNITS);
            markers_vector.push_back(tmp_p_marker); // what happens with the new indexes? check if this is correct

            if(ui->checkBox_vicon_markers->checkState() == Qt::Checked)                                    //only if markers checkbox info is checked..
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
            ROS_INFO_STREAM("element index: " << i << " moved, already existed");
            markers_vector[i]->setPosMarker(QPointF(p_msg->markers[i].x * FROM_METERS_TO_UNITS, p_msg->markers[i].y * FROM_METERS_TO_UNITS));
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
        for(int i = 0; i < markers_vector.size(); i++)
        {
            scene->addItem(markers_vector[i]);
        }
        ui->checkBox_vicon_highlight_markers->setCheckable(true);
        ui->checkBox_vicon_highlight_markers->setEnabled(true);
    }
    else
    {
        for(int i = 0; i < markers_vector.size(); i++)
        {
            scene->removeItem(markers_vector[i]);
        }
        ui->checkBox_vicon_highlight_markers->setChecked(false);
        ui->checkBox_vicon_highlight_markers->setCheckable(false);
        ui->checkBox_vicon_highlight_markers->setEnabled(false);
    }
}

void MainGUIWindow::on_checkBox_vicon_highlight_markers_toggled(bool checked)
{
    if(checked)
    {
        for(int i = 0; i < markers_vector.size(); i++)
        {
            markers_vector[i]->setHighlighted();
        }
    }
    else
    {
        for(int i = 0; i < markers_vector.size(); i++)
        {
            markers_vector[i]->clearHighlighted();
        }
    }
}
