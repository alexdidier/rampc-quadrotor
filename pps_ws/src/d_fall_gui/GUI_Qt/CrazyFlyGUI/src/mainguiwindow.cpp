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

#ifndef DEBUG_GUI
MainGUIWindow::MainGUIWindow(ros::NodeHandle* nodeHandle, /*ros::CallbackQueue *callbackQueue,
                             ros::Publisher* publisherMotorCommandsGUI,*/
                             QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainGUIWindow),
    m_pNodeHandle(nodeHandle)
{
    ui->setupUi(this);
    m_isStopButtonActive=false;
    m_isCalActive=false;
    m_trajectoryType=eTrajCustom;
    _init();
}
#else
MainGUIWindow::MainGUIWindow(QWidget *parent) :
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

    scene = new myGraphicsScene(ui->frame_drawing);
    scene->setSceneRect(QRectF(QPointF(-1400, 1400), QSizeF(1400, 1400)));

    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->graphicsView->setScene(scene);

    QObject::connect(ui->tabWidget, SIGNAL(tabCloseRequested(int)), scene, SLOT(removeCrazyFlyZone(int)));
    QObject::connect(scene, SIGNAL(numCrazyFlyZonesChanged(int)), this, SLOT(set_tabs(int)));
    QObject::connect(ui->tabWidget, SIGNAL(currentChanged(int)), scene, SLOT(setSelectedCrazyFlyZone(int)));
    QObject::connect(scene, SIGNAL(crazyFlyZoneSelected(int)), ui->tabWidget, SLOT(setCurrentIndex(int)));
    QObject::connect(scene, SIGNAL(modeChanged(int)), this, SLOT(transitionToMode(int)));
    QObject::connect(scene, SIGNAL(numTablePiecesChanged(int)), this, SLOT(handleTablePiecesNumChanged(int)));
}

#ifndef DEBUG_GUI
void MainGUIWindow::init()
{
    m_pNodeHandle->setCallbackQueue(&m_CallbackQueue);

//    m_pPublisherMotorCommandsGUI=new ros::Publisher(m_pNodeHandle->advertise
//            <crazypkg::MotorCommands>("topicDummyControllerCmd", 1));

    m_pPublisherControllerParam=new ros::Publisher(m_pNodeHandle->advertise
            <crazypkg::ControllerParam>("topicControllerParam", 100));

    m_pPublisherPositionSetpoint=new ros::Publisher(m_pNodeHandle->advertise
            <crazypkg::PositionSetpoint>("topicPositionSetpoint", 1));

    m_pPublisherSampleTime=new ros::Publisher(m_pNodeHandle->advertise
            <crazypkg::SampleTimeParam>("topicSampleTimeParam", 20));

    m_pPublisherControllerType=new ros::Publisher(m_pNodeHandle->advertise
            <std_msgs::Int32>("topicControllerType", 1));

    m_pPublisherDoSomething=new ros::Publisher(m_pNodeHandle->advertise
            <std_msgs::Int32>("topicDoSomething", 20));

    m_pPublisherFeedforwardCmd=new ros::Publisher(m_pNodeHandle->advertise
            <crazypkg::MotorCommands>("topicFeedforwardCmd",1));


    // m_pSubscriberControllerOutput=new ros::Subscriber(m_pNodeHandle->subscribe
    //         ("/FlightControl/topicControllerOutput",1,&MainGUIWindow::callbackControllerOutput,this));

    // m_pSubscriberViconData=new ros::Subscriber(m_pNodeHandle->subscribe
    //         ("/ViconDataStreamSDK/topicViconData",1,&MainGUIWindow::callbackViconData,this));

    // m_pSubscriberCntViconDataMissed=new ros::Subscriber(m_pNodeHandle->subscribe
    //         ("/FlightControl/topicCntViconDataMissed",1,&MainGUIWindow::callbackCntViconDataMissed,this));
    // initPIDParamsTable();
    // initRateParamsTable();

    readDefaultParameters();


    ros::Time::init();
    ros::Duration(3).sleep();

    m_CallbackQueue.callAvailable(ros::WallDuration(0));

    ros::Duration(1).sleep();

}


void MainGUIWindow::runCallbacks()
{
    m_CallbackQueue.callAvailable(ros::WallDuration(0));

    // updateSetpoint();
}

void MainGUIWindow::readDefaultParameters()
{
    m_pNodeHandle->param<double>("KpX",m_DefaultPIDParams[ePIDX].Kp,0);
    m_pNodeHandle->param<double>("KiX",m_DefaultPIDParams[ePIDX].Ki,0);
    m_pNodeHandle->param<double>("KdX",m_DefaultPIDParams[ePIDX].Kd,0);
    m_pNodeHandle->param<double>("NX",m_DefaultPIDParams[ePIDX].N,60);
    m_pNodeHandle->param<double>("MinSatPIDX",m_DefaultPIDParams[ePIDX].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatPIDX",m_DefaultPIDParams[ePIDX].MaxSat,98765);

    m_pNodeHandle->param<double>("KpY",m_DefaultPIDParams[ePIDY].Kp,0);
    m_pNodeHandle->param<double>("KiY",m_DefaultPIDParams[ePIDY].Ki,0);
    m_pNodeHandle->param<double>("KdY",m_DefaultPIDParams[ePIDY].Kd,0);
    m_pNodeHandle->param<double>("NY",m_DefaultPIDParams[ePIDY].N,60);
    m_pNodeHandle->param<double>("MinSatPIDY",m_DefaultPIDParams[ePIDY].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatPIDY",m_DefaultPIDParams[ePIDY].MaxSat,98765);

    m_pNodeHandle->param<double>("KpZ",m_DefaultPIDParams[ePIDZ].Kp,0);
    m_pNodeHandle->param<double>("KiZ",m_DefaultPIDParams[ePIDZ].Ki,0);
    m_pNodeHandle->param<double>("KdZ",m_DefaultPIDParams[ePIDZ].Kd,0);
    m_pNodeHandle->param<double>("NZ",m_DefaultPIDParams[ePIDZ].N,60);
    m_pNodeHandle->param<double>("MinSatPIDZ",m_DefaultPIDParams[ePIDZ].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatPIDZ",m_DefaultPIDParams[ePIDZ].MaxSat,98765);

    m_pNodeHandle->param<double>("KpYaw",m_DefaultPIDParams[ePIDYaw].Kp,0);
    m_pNodeHandle->param<double>("KiYaw",m_DefaultPIDParams[ePIDYaw].Ki,0);
    m_pNodeHandle->param<double>("KdYaw",m_DefaultPIDParams[ePIDYaw].Kd,0);
    m_pNodeHandle->param<double>("NYaw",m_DefaultPIDParams[ePIDYaw].N,60);
    m_pNodeHandle->param<double>("MinSatPIDYaw",m_DefaultPIDParams[ePIDYaw].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatPIDYaw",m_DefaultPIDParams[ePIDYaw].MaxSat,98765);

    m_pNodeHandle->param<double>("KpPitch",m_DefaultPIDParams[ePIDPitch].Kp,0);
    m_pNodeHandle->param<double>("KiPitch",m_DefaultPIDParams[ePIDPitch].Ki,0);
    m_pNodeHandle->param<double>("KdPitch",m_DefaultPIDParams[ePIDPitch].Kd,0);
    m_pNodeHandle->param<double>("NPitch",m_DefaultPIDParams[ePIDPitch].N,60);
    m_pNodeHandle->param<double>("MinSatPIDPitch",m_DefaultPIDParams[ePIDPitch].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatPIDPitch",m_DefaultPIDParams[ePIDPitch].MaxSat,98765);

    m_pNodeHandle->param<double>("KpRoll",m_DefaultPIDParams[ePIDRoll].Kp,0);
    m_pNodeHandle->param<double>("KiRoll",m_DefaultPIDParams[ePIDRoll].Ki,0);
    m_pNodeHandle->param<double>("KdRoll",m_DefaultPIDParams[ePIDRoll].Kd,0);
    m_pNodeHandle->param<double>("NRoll",m_DefaultPIDParams[ePIDRoll].N,60);
    m_pNodeHandle->param<double>("MinSatPIDRoll",m_DefaultPIDParams[ePIDRoll].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatPIDRoll",m_DefaultPIDParams[ePIDRoll].MaxSat,98765);



    m_pNodeHandle->param<double>("KpRateYaw",m_DefaultRateParams[ePIDYawRate].Kp,0);
    m_pNodeHandle->param<double>("KiRateYaw",m_DefaultRateParams[ePIDYawRate].Ki,0);
    m_pNodeHandle->param<double>("KdRateYaw",m_DefaultRateParams[ePIDYawRate].Kd,0);
    m_pNodeHandle->param<double>("NRateYaw",m_DefaultRateParams[ePIDYawRate].N,60);
    m_pNodeHandle->param<double>("MinSatRateYaw",m_DefaultRateParams[ePIDYawRate].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatRateYaw",m_DefaultRateParams[ePIDYawRate].MaxSat,98765);

    m_pNodeHandle->param<double>("KpRatePitch",m_DefaultRateParams[ePIDPitchRate].Kp,0);
    m_pNodeHandle->param<double>("KiRatePitch",m_DefaultRateParams[ePIDPitchRate].Ki,0);
    m_pNodeHandle->param<double>("KdRatePitch",m_DefaultRateParams[ePIDPitchRate].Kd,0);
    m_pNodeHandle->param<double>("NRatePitch",m_DefaultRateParams[ePIDPitchRate].N,60);
    m_pNodeHandle->param<double>("MinSatRatePitch",m_DefaultRateParams[ePIDPitchRate].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatRatePitch",m_DefaultRateParams[ePIDPitchRate].MaxSat,98765);

    m_pNodeHandle->param<double>("KpRateRoll",m_DefaultRateParams[ePIDRollRate].Kp,0);
    m_pNodeHandle->param<double>("KiRateRoll",m_DefaultRateParams[ePIDRollRate].Ki,0);
    m_pNodeHandle->param<double>("KdRateRoll",m_DefaultRateParams[ePIDRollRate].Kd,0);
    m_pNodeHandle->param<double>("NRateRoll",m_DefaultRateParams[ePIDRollRate].N,60);
    m_pNodeHandle->param<double>("MinSatRateRoll",m_DefaultRateParams[ePIDRollRate].MinSat,-9876);
    m_pNodeHandle->param<double>("MaxSatRateRoll",m_DefaultRateParams[ePIDRollRate].MaxSat,98765);



    m_pNodeHandle->param<double>("SampleTimePID",m_DefaultSampleTime[ePIDTs],0.020);
    m_pNodeHandle->param<double>("SampleTimeLQRFull",m_DefaultSampleTime[eLQRFullTs],0.020);
    m_pNodeHandle->param<double>("SampleTimeLQRNested",m_DefaultSampleTime[eLQRNestedTs],0.020);
    m_pNodeHandle->param<double>("SampleTimeRate",m_DefaultSampleTime[eRateTs],0.020);


    m_pNodeHandle->param<float>("FeedforwardMotor1",m_DefaultFeedforwardCmd.cmd1,0.0);
    m_pNodeHandle->param<float>("FeedforwardMotor2",m_DefaultFeedforwardCmd.cmd2,0.0);
    m_pNodeHandle->param<float>("FeedforwardMotor3",m_DefaultFeedforwardCmd.cmd3,0.0);
    m_pNodeHandle->param<float>("FeedforwardMotor4",m_DefaultFeedforwardCmd.cmd4,0.0);
}


CSetpointQueue::CSetpointQueue()
{
    startElem=NULL;
    currElem=NULL;
    lastElem=NULL;
}

void CSetpointQueue::insert(setpoint newElem)
{
    if (startElem==NULL)
    {
        startElem=new QueueElem(newElem);
        lastElem=startElem;
        currElem=startElem;
    }
    else
    {
    lastElem->next=new QueueElem(newElem);
    lastElem=lastElem->next;
    }
}
setpoint CSetpointQueue::getNext()
{
    setpoint ret;
    ret.x=currElem->elem.x;
    ret.y=currElem->elem.y;
    ret.z=currElem->elem.z;
    ret.yaw=currElem->elem.yaw;

    if(currElem->next!=NULL)
        currElem=currElem->next;
    else currElem=startElem;

    return ret;
}

void CSetpointQueue::print()
{
    QueueElem* p=startElem;

    ROS_INFO_STREAM("queue elements: ");
    int cnt=0;
    while (p!=NULL)
    {
        cnt++;
        ROS_INFO_STREAM("element "<<cnt<<": "<<"x="<<p->elem.x<<" y="<<p->elem.y<<" z="<<p->elem.z<<" yaw="<<p->elem.yaw);
        p=p->next;
    }
}

#endif  // DEBUG_GUI



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
    // if(newNum == 0)
    // {
    //     ui->radioButton_crazyfly_zones_mode->setCheckable(false);
    //     ui->radioButton_crazyfly_zones_mode->setEnabled(false);
    // }
    // else
    // {
    //     ui->radioButton_crazyfly_zones_mode->setCheckable(true);
    //     ui->radioButton_crazyfly_zones_mode->setEnabled(true);
    // }
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
