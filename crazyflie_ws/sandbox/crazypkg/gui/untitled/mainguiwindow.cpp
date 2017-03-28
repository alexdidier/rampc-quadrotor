#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"

#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QString>



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

void MainGUIWindow::_refresh_tabs()
{
    ui->tabWidget->clear();
    std::string str;
    for (int i = 0; i < ui->spinBoxNumCrazyflies->value(); i++)
    {
        str = "CrazyFly ";
        str += std::to_string(i+1);
        QString qstr(str.c_str());
        ui->tabWidget->addTab(new QWidget(), qstr);
    }
}

void MainGUIWindow::_init()
{

    scene = new myGraphicsScene(ui->frame_drawing);
    //scene->setSceneRect(QRectF(QPointF(-100, 100), QSizeF(200, 200)));

    ui->graphicsView->setScene(scene);
    item1 = new QGraphicsRectItem(0,0,100,100);
    item1->setBrush(QBrush(Qt::blue));
    scene->addText("Hello world!");
    // scene->addRect(rect, QPen(Qt::black), QBrush(Qt::blue));
    scene->addItem(item1);

    ui->spinBoxNumCrazyflies->setMaximum(N_MAX_CRAZYFLIES);
    _refresh_tabs();
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

    // setDefaultPIDParameters();
    // setDefaultRateParameters();

    // initPositionSetpoint();
    // initSetpointType();
    // initSampleTime();
    // initFeedforwardCmd();

    // initSetpointQueues();

    ros::Duration(1).sleep();

    // initControllerType();

    //refreshScreen();
}

// void MainGUIWindow::refreshScreen()
// {
// //    for(int i=0;i<countPIDControllers;i++)
// //    {
// //        ((QDoubleSpinBox*)ui->tableWidget->cellWidget(i,eKp))->setValue(m_PIDParams[i].Kp);
// //        ((QDoubleSpinBox*)ui->tableWidget->cellWidget(i,eKi))->setValue(m_PIDParams[i].Ki);
// //        ((QDoubleSpinBox*)ui->tableWidget->cellWidget(i,eKd))->setValue(m_PIDParams[i].Kd);
// //        ((QDoubleSpinBox*)ui->tableWidget->cellWidget(i,eN))->setValue(m_PIDParams[i].N);
// //    }

// }




// void MainGUIWindow::callbackCntViconDataMissed(const std_msgs::Int32& msg)
// {
//     ui->LCDMissedMes->display(msg.data);
// }

// void MainGUIWindow::callbackControllerOutput(const crazypkg::ControllerOutputPackage& msg)
// {
//     ui->LCDMotor1Cmd->display(msg.motorCmd1);
//     ui->LCDMotor2Cmd->display(msg.motorCmd2);
//     ui->LCDMotor3Cmd->display(msg.motorCmd3);
//     ui->LCDMotor4Cmd->display(msg.motorCmd4);
//     ui->LCDRollCmd->display(msg.roll);
//     ui->LCDPitchCmd->display(msg.pitch);
//     ui->LCDYawCmd->display(msg.yaw);
//     ui->LCDThrustCmd->display(msg.thrust);

//     switch (msg.onboardControllerType)
//     {
//     case eOnboardAngleController: {ui->labelControllerOutputMode->setText("Angle"); break;}
//     case eOnboardRateController: {ui->labelControllerOutputMode->setText("Rate"); break;}
//     case eOnboardMotorCmdController: {ui->labelControllerOutputMode->setText("MotorCmd"); break;}
//     default:{ROS_ERROR("unknown onboard controller type in MainGUIWindow::callbackControllerOutput"); break;}
//     }


// }

// void MainGUIWindow::callbackViconData(const crazypkg::ViconData& msg)
// {
//     ui->LCDViconDataX->display(msg.x);
//     ui->LCDViconDataY->display(msg.y);
//     ui->LCDViconDataZ->display(msg.z);
//     ui->LCDViconDataYaw->display(msg.yaw*RAD2DEG);
//     ui->LCDViconDataPitch->display(msg.pitch*RAD2DEG);
//     ui->LCDViconDataRoll->display(msg.roll*RAD2DEG);
// }

void MainGUIWindow::runCallbacks()
{
    m_CallbackQueue.callAvailable(ros::WallDuration(0));

    // updateSetpoint();
}

// void MainGUIWindow::setDefaultPIDParameters()
// {
//     //memcpy(&m_PIDParams,&m_DefaultPIDParams,sizeof(m_DefaultPIDParams));

//     for(int i=0;i<countPIDControllers;i++)
//     {
//         ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,eKp))->setValue(m_DefaultPIDParams[i].Kp);
//         ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,eKi))->setValue(m_DefaultPIDParams[i].Ki);
//         ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,eKd))->setValue(m_DefaultPIDParams[i].Kd);
//         ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,eN))->setValue(m_DefaultPIDParams[i].N);
//         ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,eMinPIDSat))->setValue(m_DefaultPIDParams[i].MinSat);
//         ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,eMaxPIDSat))->setValue(m_DefaultPIDParams[i].MaxSat);
//     }
// }

// void MainGUIWindow::setDefaultRateParameters()
// {
//     //memcpy(&m_PIDParams,&m_DefaultPIDParams,sizeof(m_DefaultPIDParams));

//     for(int i=0;i<countRateControllers;i++)
//     {
//         ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,eKp))->setValue(m_DefaultRateParams[i].Kp);
//         ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,eKi))->setValue(m_DefaultRateParams[i].Ki);
//         ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,eKd))->setValue(m_DefaultRateParams[i].Kd);
//         ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,eN))->setValue(m_DefaultRateParams[i].N);
//         ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,eMinPIDSat))->setValue(m_DefaultRateParams[i].MinSat);
//         ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,eMaxPIDSat))->setValue(m_DefaultRateParams[i].MaxSat);
//     }
// }

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

// void MainGUIWindow::initPIDParamsTable()
// {
//     for (int i = 0; i < countPIDControllers; ++i)
//     {
//         for (int j = 0; j < countPIDParams ; ++j)
//         {
//             ui->PIDParamTable->setCellWidget(i,j,new QDoubleSpinBox(ui->PIDParamTable));
//             ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setDecimals(3);
//             ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setMaximum(999999.0);
//             ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setProperty("row",i);
//             ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setProperty("column",j);
//             ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setValue(1234.321);
//             if(j==eMinPIDSat)
//                 ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setMinimum(-99999.0);
//             else
//                 ((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))->setMinimum(0);
//             connect(((QDoubleSpinBox*)ui->PIDParamTable->cellWidget(i,j))
//                     , SIGNAL(valueChanged(double)), this, SLOT(PIDParamTableChanged(double)));
//         }


//     }
// }

// void MainGUIWindow::initRateParamsTable()
// {
//         for (int i = 0; i < countRateControllers; ++i) {
//             for (int j = 0; j < countPIDParams ; ++j) {
//                 ui->RateParamTable->setCellWidget(i,j,new QDoubleSpinBox(ui->RateParamTable));
//                 ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setDecimals(3);
//                 ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setMaximum(999999.0);
//                 ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setProperty("row",i);
//                 ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setProperty("column",j);
//                 ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setValue(1234.321);
//                 if(j==eMinPIDSat)
//                     ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setMinimum(-99999.0);
//                 else
//                      ((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))->setMinimum(0);
//                 connect(((QDoubleSpinBox*)ui->RateParamTable->cellWidget(i,j))
//                         , SIGNAL(valueChanged(double)), this, SLOT(RateParamTableChanged(double)));
//             }


//         }

// }

// void MainGUIWindow::PIDParamTableChanged(double param)
// {

//     QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
//         if (spinBox)
//         {
//             m_controllerParam.crazyControllerType=ePID;
//             m_controllerParam.basicControllerType=spinBox->property("row").toInt();
//             m_controllerParam.paramType=spinBox->property("column").toInt();
//             m_controllerParam.value=param;

//             m_pPublisherControllerParam->publish(m_controllerParam);
//         }
// }

// void MainGUIWindow::RateParamTableChanged(double param)
// {

//     QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
//         if (spinBox)
//         {
//             m_controllerParam.crazyControllerType=eRate;
//             m_controllerParam.basicControllerType=spinBox->property("row").toInt();
//             m_controllerParam.paramType=spinBox->property("column").toInt();
//             m_controllerParam.value=param;

//             m_pPublisherControllerParam->publish(m_controllerParam);
//         }
// }

// void MainGUIWindow::initPositionSetpoint()
// {
//     ui->SetpointX->setValue(ui->LCDViconDataX->value());
//     ui->SetpointY->setValue(ui->LCDViconDataY->value());
//     ui->SetpointZ->setValue(ui->LCDViconDataZ->value());
//     ui->SetpointYaw->setValue(ui->LCDViconDataYaw->value());

//     publishSetpoint();

//     connect(ui->SetpointX, SIGNAL(valueChanged(double)), this, SLOT(positionSetpointChanged(double)));
//     ui->SetpointX->setProperty("param","X");
//     connect(ui->SetpointY, SIGNAL(valueChanged(double)), this, SLOT(positionSetpointChanged(double)));
//     ui->SetpointY->setProperty("param","Y");
//     connect(ui->SetpointZ, SIGNAL(valueChanged(double)), this, SLOT(positionSetpointChanged(double)));
//     ui->SetpointZ->setProperty("param","Z");
//     connect(ui->SetpointYaw, SIGNAL(valueChanged(double)), this, SLOT(positionSetpointChanged(double)));
//     ui->SetpointYaw->setProperty("param","Yaw");

// }

// void MainGUIWindow::positionSetpointChanged(double param)
// {
//     //publishSetpoint();
// }

// void MainGUIWindow::publishSetpoint()
// {
//     m_positionSetpoint.x=ui->SetpointX->value()/1000;
//     m_positionSetpoint.y=ui->SetpointY->value()/1000;
//     m_positionSetpoint.z=ui->SetpointZ->value()/1000;
//     m_positionSetpoint.yaw=ui->SetpointYaw->value()*DEG2RAD;
//     m_pPublisherPositionSetpoint->publish(m_positionSetpoint);
// }

// void MainGUIWindow::initSampleTime()
// {



// //    publishSampleTime(ePIDTs);
// //    publishSampleTime(eLQRTs);
// //    publishSampleTime(eLQRInnerTs);
// //    publishSampleTime(eLQROuterTs);

//     connect(ui->PIDTs, SIGNAL(valueChanged(double)), this, SLOT(sampleTimeChanged(double)));
//     ui->PIDTs->setProperty("sampleTimeType",ePIDTs);
//     connect(ui->RateTs, SIGNAL(valueChanged(double)), this, SLOT(sampleTimeChanged(double)));
//     ui->RateTs->setProperty("sampleTimeType",eRateTs);
//     connect(ui->LQRFullTs, SIGNAL(valueChanged(double)), this, SLOT(sampleTimeChanged(double)));
//     ui->LQRFullTs->setProperty("sampleTimeType",eLQRFullTs);
//     connect(ui->LQRNestedTs, SIGNAL(valueChanged(double)), this, SLOT(sampleTimeChanged(double)));
//     ui->LQRNestedTs->setProperty("sampleTimeType",eLQRNestedTs);

//     setDefaultSampleTime();

// }

// void MainGUIWindow::setDefaultSampleTime()
// {
//     ui->PIDTs->setValue(m_DefaultSampleTime[ePIDTs]*1000);
//     ui->RateTs->setValue(m_DefaultSampleTime[eRateTs]*1000);
//     ui->LQRFullTs->setValue(m_DefaultSampleTime[eLQRFullTs]*1000);
//     ui->LQRNestedTs->setValue(m_DefaultSampleTime[eLQRNestedTs]*1000);
// }

//void MainGUIWindow::publishSampleTime(ESampleTimeType sampleTimeType)
//{
//    m_sampleTimeParam.crazyControllerType=controller;
//    switch (controller)
//    {
//    case ePIDTs:
//    {
//        m_sampleTimeParam.value=ui->PIDTs->value();
//        break;
//    }
//    case eLQRTs:
//    {
//        m_sampleTimeParam.value=ui->LQRTs->value();
//        break;
//    }
//    case eLQRInnerTs:
//    {
//        m_sampleTimeParam.value=ui->LQRInnerTs->value();
//        break;
//    }
//    case eLQROuterTs:
//    {
//        m_sampleTimeParam.value=ui->LQROuterTs->value();
//        break;
//    }
//    default: ROS_ERROR("invalid sampleTime type in publish sample time")
//    }

//    m_pPublisherSampleTime->publish(m_sampleTimeParam);
//}

// void MainGUIWindow::sampleTimeChanged(double param)
// {
//     QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
//         if (spinBox)
//         {
//             m_sampleTimeParam.value=param/1000;
//             m_sampleTimeParam.sampleTimeType=spinBox->property("sampleTimeType").toInt();
//             m_pPublisherSampleTime->publish(m_sampleTimeParam);
//         }
// }

// void MainGUIWindow::initControllerType()
// {
//     connect(ui->controllerPIDPosition, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerPIDPosition->setProperty("controllerType",ePIDPosition);
//     connect(ui->controllerPIDAngle, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerPIDAngle->setProperty("controllerType",ePIDAngle);
//     connect(ui->controllerPIDFull, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerPIDFull->setProperty("controllerType",ePIDFull);

//     connect(ui->controllerLQRFull, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerLQRFull->setProperty("controllerType",eLQRFull);

//     connect(ui->controllerLQRNestedOnboard, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerLQRNestedOnboard->setProperty("controllerType",eLQRNestedOnboardRate);
//     connect(ui->controllerLQRNestedOffboard, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerLQRNestedOffboard->setProperty("controllerType",eLQRNestedOffboardRate);

//     connect(ui->controllerAngleCmdTest, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerAngleCmdTest->setProperty("controllerType",eAngleCmdTest);
//     connect(ui->controllerRateCmdTest, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerRateCmdTest->setProperty("controllerType",eRateCmdTest);
//     connect(ui->controllerMotorCmdTest, SIGNAL(toggled(bool)), this, SLOT(controllerTypeChanged(bool)));
//     ui->controllerMotorCmdTest->setProperty("controllerType",eMotorCmdTest);

//     //set default controller:
//     ui->controllerMotorCmdTest->setChecked(true);

// }

// void MainGUIWindow::initSetpointType()
// {
//     connect(ui->trajCustom, SIGNAL(toggled(bool)), this, SLOT(trajectoryTypeChanged(bool)));
//     ui->trajCustom->setProperty("trajectoryType",eTrajCustom);
//     connect(ui->trajCircle, SIGNAL(toggled(bool)), this, SLOT(trajectoryTypeChanged(bool)));
//     ui->trajCircle->setProperty("trajectoryType",eTrajCircle);
//     connect(ui->trajSquare, SIGNAL(toggled(bool)), this, SLOT(trajectoryTypeChanged(bool)));
//     ui->trajSquare->setProperty("trajectoryType",eTrajSquare);

//     ui->trajCustom->setChecked(true);
// }

// void MainGUIWindow::trajectoryTypeChanged(bool checked)
// {
//     QRadioButton* radioButton = qobject_cast<QRadioButton*>(sender());
//         if (radioButton && checked)
//         {
//             m_trajectoryType=(ETrajectoryType)radioButton->property("trajectoryType").toInt();
//         }
// }

// void MainGUIWindow::controllerTypeChanged(bool checked)
// {
//     QRadioButton* radioButton = qobject_cast<QRadioButton*>(sender());
//         if (radioButton && checked)
//         {
//             m_controllerType.data=radioButton->property("controllerType").toInt();

//             if(m_controllerType.data==ePIDPosition || m_controllerType.data==ePIDAngle ||
//                     m_controllerType.data==eLQRNestedOnboardRate
//                     || m_controllerType.data==eAngleCmdTest || m_controllerType.data==eRateCmdTest
//                     || m_controllerType.data==eMotorCmdTest)
//             {
//                 ui->labelRateMode->setText("Onboard");
//             }
//             else if(m_controllerType.data==ePIDFull || m_controllerType.data==eLQRFull ||
//                     m_controllerType.data==eLQRNestedOffboardRate )
//             {
//                ui->labelRateMode->setText("Offboard");
//             }
//             else
//             {
//                 ROS_ERROR("unknown controller type in MainGUIWindow::controllerTypeChanged");
//             }

//             m_pPublisherControllerType->publish(m_controllerType);
//         }
// }




// void MainGUIWindow::on_buttonStop_clicked()
// {
//     if(!m_isStopButtonActive)
//     {
//         m_isStopButtonActive=true;
//         m_DoSomething.data=eStopQuad;
//         m_pPublisherDoSomething->publish(m_DoSomething);
//         ui->buttonStop->setText("START");
//     }
//     else
//     {
//         m_isStopButtonActive=false;
//         m_DoSomething.data=eStartQuad;
//         m_pPublisherDoSomething->publish(m_DoSomething);
//         ui->buttonStop->setText("STOP");
//     }

// }

// void MainGUIWindow::on_buttonPrint_clicked()
// {
//     m_DoSomething.data=ePrintInfo;
//     m_pPublisherDoSomething->publish(m_DoSomething);
// }




// void MainGUIWindow::initFeedforwardCmd()
// {
//     connect(ui->FeedforwardCmd1, SIGNAL(valueChanged(double)), this, SLOT(feedforwardCmdChanged(double)));
//     connect(ui->FeedforwardCmd2, SIGNAL(valueChanged(double)), this, SLOT(feedforwardCmdChanged(double)));
//     connect(ui->FeedforwardCmd3, SIGNAL(valueChanged(double)), this, SLOT(feedforwardCmdChanged(double)));
//     connect(ui->FeedforwardCmd4, SIGNAL(valueChanged(double)), this, SLOT(feedforwardCmdChanged(double)));


//     setDefaultFeedforwardCmd();
// }

// void MainGUIWindow::feedforwardCmdChanged(double cmd)
// {
//     m_feedforwardCmd.cmd1=ui->FeedforwardCmd1->value();
//     m_feedforwardCmd.cmd2=ui->FeedforwardCmd2->value();
//     m_feedforwardCmd.cmd3=ui->FeedforwardCmd3->value();
//     m_feedforwardCmd.cmd4=ui->FeedforwardCmd4->value();
//     m_pPublisherFeedforwardCmd->publish(m_feedforwardCmd);
// }

// void MainGUIWindow::setDefaultFeedforwardCmd()
// {
//     ui->FeedforwardCmd1->setValue(m_DefaultFeedforwardCmd.cmd1);
//     ui->FeedforwardCmd2->setValue(m_DefaultFeedforwardCmd.cmd2);
//     ui->FeedforwardCmd3->setValue(m_DefaultFeedforwardCmd.cmd3);
//     ui->FeedforwardCmd4->setValue(m_DefaultFeedforwardCmd.cmd4);
// }

// void MainGUIWindow::on_buttonSetpointChange_clicked()
// {
//     publishSetpoint();
// }

// void MainGUIWindow::on_buttonResetControllers_clicked()
// {
//     m_DoSomething.data=eResetControllers;
//     m_pPublisherDoSomething->publish(m_DoSomething);
// }

// void MainGUIWindow::on_buttonSetDefaultTs_clicked()
// {
//         setDefaultSampleTime();
// }

// void MainGUIWindow::on_buttonPIDDefaultParams_clicked()
// {
//     setDefaultPIDParameters();
// }

// void MainGUIWindow::on_buttonSetpointCurrPos_clicked()
// {
//     ui->SetpointX->setValue(ui->LCDViconDataX->value());
//     ui->SetpointY->setValue(ui->LCDViconDataY->value());
//     ui->SetpointZ->setValue(ui->LCDViconDataZ->value());
//     ui->SetpointYaw->setValue(ui->LCDViconDataYaw->value());

//     publishSetpoint();
// }

// void MainGUIWindow::on_buttonDefaultFeedforward_clicked()
// {
//     setDefaultFeedforwardCmd();
// }


// void MainGUIWindow::on_buttonResetMissed_clicked()
// {
//     ui->LCDMissedMes->display(0);
//     m_DoSomething.data=eResetCntMissedViconData;
//     m_pPublisherDoSomething->publish(m_DoSomething);
// }

// void MainGUIWindow::on_buttonSetDefaultRateParams_clicked()
// {
//     setDefaultRateParameters();
// }

// void MainGUIWindow::on_SetpointHome_clicked()
// {
//     ui->SetpointX->setValue(0);
//     ui->SetpointY->setValue(-700);
//     ui->SetpointZ->setValue(1200);

//     publishSetpoint();
// }

// void MainGUIWindow::on_setpointZ200_clicked()
// {
//     ui->SetpointZ->setValue(200);

//     publishSetpoint();
// }




// void MainGUIWindow::on_slideMotorCmdTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eMotorCmdTest;
//     m_controllerParam.basicControllerType=eTestMotorCmd;
//     m_controllerParam.value=value*600;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDMotorCmdTest->display(value*600);
// }



// void MainGUIWindow::on_slideRollAngleTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eAngleCmdTest;
//     m_controllerParam.basicControllerType=eTestRoll;
//     m_controllerParam.value=value*0.008;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDRollAngleTest->display(value*0.008*RAD2DEG);
// }

// void MainGUIWindow::on_slidePitchAngleTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eAngleCmdTest;
//     m_controllerParam.basicControllerType=eTestPitch;
//     m_controllerParam.value=value*0.008;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDPitchAngleTest->display(value*0.008*RAD2DEG);
// }

// void MainGUIWindow::on_slideYawAngleTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eAngleCmdTest;
//     m_controllerParam.basicControllerType=eTestYaw;
//     m_controllerParam.value=value*0.03;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDYawAngleTest->display(value*0.03*RAD2DEG);
// }

// void MainGUIWindow::on_slideRollRateTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eRateCmdTest;
//     m_controllerParam.basicControllerType=eTestRoll;
//     m_controllerParam.value=value*0.005;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDRollRateTest->display(value*0.005*RAD2DEG);
// }

// void MainGUIWindow::on_slidePitchRateTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eRateCmdTest;
//     m_controllerParam.basicControllerType=eTestPitch;
//     m_controllerParam.value=value*0.005;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDPitchRateTest->display(value*0.005*RAD2DEG);
// }

// void MainGUIWindow::on_slideYawRateTest_valueChanged(int value)
// {
//     m_controllerParam.crazyControllerType=eRateCmdTest;
//     m_controllerParam.basicControllerType=eTestYaw;
//     m_controllerParam.value=value*0.005;
//     m_pPublisherControllerParam->publish(m_controllerParam);
//     ui->LCDYawRateTest->display(value*0.005*RAD2DEG);
// }


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

// void MainGUIWindow::initSetpointQueues()
// {
//     setpoint sp;

//     int pointsCnt=500;
//     for(int i=0; i<pointsCnt; i++)
//     {
//         double angle=i*2*3.141592/pointsCnt;
//         sp.x=cos(angle)*0.7;
//         sp.y=sin(angle)*0.7;
//         sp.z=1;
//         double yaw=PI/2+angle;
//         while(yaw>PI) yaw-=2*PI;
//         while(yaw<-PI) yaw+=2*PI;
//         //sp.yaw=yaw;
//         sp.yaw=0;
//         m_trajCircle.insert(sp);
//     }

// //    for(int i=0; i<pointsCnt; i++)
// //    {

// //        sp.x=0;
// //        sp.y=0.1;
// //        sp.z=1.123;
// //        sp.yaw=0;

// //        m_trajSquare.insert(sp);
// //    }

//     sp.y=0;
//     sp.yaw=0;


//     sp.x=-0.250; sp.z=0.500; m_trajSquare.insert(sp);

//     //sp.x=-0.550; sp.z=0.500; m_trajSquare.insert(sp);
//     sp.x=-0.750; sp.z=0.500; m_trajSquare.insert(sp);

//     //sp.x=-0.750; sp.z=0.750; m_trajSquare.insert(sp);
//     sp.x=-0.750; sp.z=1; m_trajSquare.insert(sp);

//     //sp.x=-0.500; sp.z=1; m_trajSquare.insert(sp);
//     sp.x=-0.250; sp.z=1; m_trajSquare.insert(sp);

//     //sp.x=-0.500; sp.z=1; m_trajSquare.insert(sp);
//     sp.x=-0.750; sp.z=1; m_trajSquare.insert(sp);

//     sp.x=-0.750; sp.z=1.5; m_trajSquare.insert(sp);

//     sp.x=0; sp.z=1.5; m_trajSquare.insert(sp);

//     sp.x=0; sp.z=0.500; m_trajSquare.insert(sp);

//     sp.x=0; sp.z=1.5; m_trajSquare.insert(sp);

//     sp.x=0.250; sp.z=1.5; m_trajSquare.insert(sp);

//     sp.x=0.250; sp.z=0.500; m_trajSquare.insert(sp);

//     sp.x=0.250; sp.z=1; m_trajSquare.insert(sp);

//     sp.x=0.750; sp.z=1; m_trajSquare.insert(sp);

//     sp.x=0.750; sp.z=1.5; m_trajSquare.insert(sp);

//     sp.x=0.750; sp.z=0.500; m_trajSquare.insert(sp);

//     m_trajSquare.print();

// }

// void MainGUIWindow::updateSetpoint()
// {
//   if(m_trajectoryType==eTrajCustom)
//       return;

//   double currX,currY,spX,spY,currZ,spZ;
//   currX=ui->LCDViconDataX->value();
//   currY=ui->LCDViconDataY->value();
//   currZ=ui->LCDViconDataZ->value();

//   spX=ui->SetpointX->value();
//   spY=ui->SetpointY->value();
//   spZ=ui->SetpointZ->value();

//   while(sqrt((spX-currX)*(spX-currX)+(spY-currY)*(spY-currY)+(spZ-currZ)*(spZ-currZ))<40)
// {

//   setpoint sp;

//     if(m_trajectoryType==eTrajCircle)
//         sp=m_trajCircle.getNext();
//     if(m_trajectoryType==eTrajSquare)
//         sp=m_trajSquare.getNext();

//     ui->SetpointX->setValue(sp.x*1000);
//     ui->SetpointY->setValue(sp.y*1000);
//     ui->SetpointZ->setValue(sp.z*1000);
//     ui->SetpointYaw->setValue(sp.yaw*RAD2DEG);

//     publishSetpoint();

//     spX=ui->SetpointX->value();
//     spY=ui->SetpointY->value();
//     spZ=ui->SetpointZ->value();
// }

// }




// void MainGUIWindow::on_buttonStop_2_clicked()
// {

//         if(!m_isCalActive)
//         {
//             m_isCalActive=true;
//             m_DoSomething.data=eStartCal;
//             m_pPublisherDoSomething->publish(m_DoSomething);
//             ui->buttonStop_2->setText("STOP cal");
//         }
//         else
//         {
//             m_isCalActive=false;
//             m_DoSomething.data=eStopCal;
//             m_pPublisherDoSomething->publish(m_DoSomething);
//             ui->buttonStop_2->setText("START cal");
//         }


// }
#endif  // DEBUG_GUI
void MainGUIWindow::on_spinBoxNumCrazyflies_valueChanged(int arg1)
{
    _refresh_tabs();
}

void MainGUIWindow::on_spinBoxNumCrazyflies_editingFinished()
{

}

void MainGUIWindow::on_graphicsView_rubberBandChanged(const QRect &viewportRect, const QPointF &fromScenePoint, const QPointF &toScenePoint)
{

}
