
#include "CrazyFlieInclude.h"

uint16_t SaturateToUINT16(float input)
{
  if(input>UINT16_MAX)
    input=UINT16_MAX-1;
  else if(input<0)
    input=0;
  return input;
}

FullStateEstimation discreteDerivativeEstimator::estimateState()
{
    m_estimation.x=m_measurement.x;
    m_estimation.y=m_measurement.y;
    m_estimation.z=m_measurement.z;
    m_estimation.roll=m_measurement.roll;
    m_estimation.pitch=m_measurement.pitch;
    m_estimation.yaw=m_measurement.yaw;

    m_estimation.pitch_dot=(m_measurement.pitch-m_prevMeasurement.pitch)*m_frequency;
    m_estimation.roll_dot=(m_measurement.roll-m_prevMeasurement.roll)*m_frequency;
    m_estimation.yaw_dot=(m_measurement.yaw-m_prevMeasurement.yaw)*m_frequency;

    m_estimation.x_dot=(m_measurement.x-m_prevMeasurement.x)*m_frequency;
    m_estimation.y_dot=(m_measurement.y-m_prevMeasurement.y)*m_frequency;
    m_estimation.z_dot=(m_measurement.z-m_prevMeasurement.z)*m_frequency;

    m_prevEstimation=m_estimation;
    return m_estimation;
}

SSKalmanFilter::SSKalmanFilter(double frequency):CStateEstimator(frequency)
{
    K[0]=1; K[1]=1; K[2]=1; K[3]=22.3384; K[4]=22.3384; K[5]=22.3384;
    Ahat[0]=-22.3384; Ahat[1]=0.9106;
}

FullStateEstimation SSKalmanFilter::estimateState()
{


    m_estimation.roll=m_measurement.roll;
    m_estimation.pitch=m_measurement.pitch;
    m_estimation.yaw=m_measurement.yaw;

    Ahatx1[0]=0; Ahatx1[1]=0; Ahatx1[2]=0;
    Ahatx1[3]=Ahat[0]*m_prevEstimation.x+Ahat[1]*m_prevEstimation.x_dot;
    Ahatx1[4]=Ahat[0]*m_prevEstimation.y+Ahat[1]*m_prevEstimation.y_dot;
    Ahatx1[5]=Ahat[0]*m_prevEstimation.z+Ahat[1]*m_prevEstimation.z_dot;

    Kx[0]=m_measurement.x*K[0];
    Kx[1]=m_measurement.y*K[1];
    Kx[2]=m_measurement.z*K[2];
    Kx[3]=m_measurement.x*K[3];
    Kx[4]=m_measurement.y*K[4];
    Kx[5]=m_measurement.z*K[5];

    m_estimation.x=Ahatx1[0]+Kx[0];
    m_estimation.y=Ahatx1[1]+Kx[1];
    m_estimation.z=Ahatx1[2]+Kx[2];
    m_estimation.x_dot=Ahatx1[3]+Kx[3];
    m_estimation.y_dot=Ahatx1[4]+Kx[4];
    m_estimation.z_dot=Ahatx1[5]+Kx[5];

    m_prevEstimation=m_estimation;

    m_estimation.x=m_measurement.x;
    m_estimation.y=m_measurement.y;
    m_estimation.z=m_measurement.z;

    return m_estimation;
}

static CControlMgr gControlMgr;

CControlMgr* pGetControlMgr()
{
    return &gControlMgr;
}


void CControlMgr::init(ros::NodeHandle* nodeHandle)
{

    ROS_INFO("initializing Control Manager");

    m_pMotorCmdTestController=new CMotorCmdTestController();
    m_pAngleCmdTestController=new CAngleCmdTestController();
    m_pRateCmdTestController=new CRateCmdTestController();
    m_pCrazyPIDController=new CCrazyPIDController();
    m_pCrazyLQRNestedController=new CCrazyLQRNestedController;
    m_pCrazyLQRFullController=NULL;

    m_pRateController=new CRateController();

    m_pActiveCrazyController=NULL;

    m_derivativeEstimator=new discreteDerivativeEstimator(250.0);
    m_SSKalmanEstimator=new SSKalmanFilter(250.0);


    m_isStopped=false;
    m_isRateOffboard=false;

    m_isCalibrating=false;
    m_calCnt=0;
    m_calRollSum=0;
    m_calPitchSum=0;
    m_RollOffset=0;
    m_PitchOffset=0;

    streamStateEstimation.open("/home/crazyfly/crazyflie_ws/sandbox/crazypkg/log/fullStateEstimation.txt",std::ios_base::trunc);
    streamDerEstimation.open("/home/crazyfly/crazyflie_ws/sandbox/crazypkg/log/derivativeEstimation.txt",std::ios_base::trunc);
    streamSetpoint.open("/home/crazyfly/crazyflie_ws/sandbox/crazypkg/log/setpoint.txt",std::ios_base::trunc);
    streamPackageSentToCF.open("/home/crazyfly/crazyflie_ws/sandbox/crazypkg/log/packageSentToCF.txt",std::ios_base::trunc);
    streamTime.open("/home/crazyfly/crazyflie_ws/sandbox/crazypkg/log/time.txt",std::ios_base::trunc);

    if(!streamStateEstimation.is_open())
        ROS_ERROR("could not open log fullStateEstimation");
    if(!streamSetpoint.is_open())
        ROS_ERROR("could not open log setpoint");
    if(!streamPackageSentToCF.is_open())
        ROS_ERROR("could not open log packageSentToCF");
    if(!streamTime.is_open())
        ROS_ERROR("could not open log time");

    logTimeOffset=ros::Time::now().toSec();
    ROS_INFO("log opened");



    m_pNodeHandle=nodeHandle;

    m_pCallbackQueueControlMgr=new ros::CallbackQueue();
    m_pNodeHandle->setCallbackQueue(m_pCallbackQueueControlMgr);

    m_pTimerCrazyController=new ros::Timer(m_pNodeHandle->createTimer
            (ros::Duration(0.01), &CControlMgr::callbackRunCrazyController,this));
    m_pTimerCrazyController->stop();
    m_pTimerRateController=new ros::Timer(m_pNodeHandle->createTimer
            (ros::Duration(0.01), &CControlMgr::callbackRunRateController,this));
    m_pTimerRateController->stop();

    ROS_INFO("creating subscribers and publishers for the control manager");
    m_pSubscriberViconData=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/ViconDataStreamSDK/topicViconData",1,&CControlMgr::callbackViconData,this));

    m_pSubscriberPositionSetpoint=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/GUI/topicPositionSetpoint",1,&CControlMgr::callbackPositionSetpointChanged,this));

    m_pSubscriberSampleTimeParam=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/GUI/topicSampleTimeParam",20,&CControlMgr::callbackSampleTimeChanged,this));

    m_pSubscriberControllerParam=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/GUI/topicControllerParam",100,&CControlMgr::callbackControllerParametersChanged,this));

//    m_pSubscriberDummyControllerCmd=new ros::Subscriber(m_pNodeHandle->subscribe
//            ("/GUI/topicDummyControllerCmd",1,&CControlMgr::callbackDummyControllerCommandChanged,this));

    m_pSubscriberControllerType=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/GUI/topicControllerType",1,&CControlMgr::callbackControllerTypeChanged,this));

    m_pSubscriberDoSomething=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/GUI/topicDoSomething",20,&CControlMgr::callbackDoSomething,this));

    m_pSubscriberFeedforwardCmd=new ros::Subscriber(m_pNodeHandle->subscribe
            ("/GUI/topicFeedforwardCmd",1,&CControlMgr::callbackFeedforwardCmdChanged,this));


    m_pPublisherControllerOutput=new ros::Publisher(m_pNodeHandle->advertise
                                                 <crazypkg::ControllerOutputPackage>("topicControllerOutput", 1));

    m_pPublisherCntViconDataMissed=new ros::Publisher(m_pNodeHandle->advertise
                                                 <std_msgs::Int32>("topicCntViconDataMissed", 1));




    ROS_INFO("starting spinner");
    m_pAsyncSpinner=new ros::AsyncSpinner(0,m_pCallbackQueueControlMgr);
    m_pAsyncSpinner->start();
}


void CControlMgr::StopQuadrotor()
{
    m_pTimerCrazyController->stop();
    m_pTimerRateController->stop();
    m_isStopped=true;

    m_packageToSend.motorCmd1=0;
    m_packageToSend.motorCmd2=0;
    m_packageToSend.motorCmd3=0;
    m_packageToSend.motorCmd4=0;
    m_packageToSend.onboardControllerType=eOnboardMotorCmdController;
    m_pPublisherControllerOutput->publish(m_packageToSend);

}

void CControlMgr::StartQuadrotor()
{
    if(m_isRateOffboard)
        m_pTimerRateController->start();
    m_pTimerCrazyController->start();

    m_SSKalmanEstimator->pushMeasurement(FullStateEstimation
                     (m_ViconData.acquiringTime, m_ViconData.x,m_ViconData.y,m_ViconData.z,
                              m_ViconData.yaw,m_ViconData.pitch,m_ViconData.roll));
    m_SSKalmanEstimator->reset();


    m_isStopped=false;
}

void CControlMgr::callbackDoSomething(const std_msgs::Int32& msg)
{
    ROS_INFO("callback do something");
    switch (msg.data)
    {
    case ePrintInfo: { printInfo(); break; }
    case eStopQuad: { StopQuadrotor(); break; }
    case eStartQuad: { StartQuadrotor(); break; }
    case eResetControllers: {resetControllers(); break; }
    case eResetCntMissedViconData: {m_cntViconDataMissed=0; break;}
    case eStartCal: { m_calCnt=0; m_calRollSum=0;
        m_calPitchSum=0; m_isCalibrating=true; break;}
    case eStopCal: {m_isCalibrating=false; m_RollOffset=m_calRollSum/m_calCnt;
        m_PitchOffset=m_calPitchSum/m_calCnt; break;}
    default: ROS_ERROR("invalid do something code in doSomething callback");
    }
}

void CControlMgr::resetControllers()
{
    ROS_INFO("reset controllers");
    m_pMotorCmdTestController->reset();
    m_pAngleCmdTestController->reset();
    m_pRateCmdTestController->reset();
    m_pCrazyPIDController->reset();
    m_pRateController->reset();
    m_pCrazyLQRNestedController->reset();
}


void CControlMgr::callbackPositionSetpointChanged(const crazypkg::PositionSetpoint &msgPositionSetpoint)
{
    //ROS_INFO("changing position setpoint");
    m_positionSetpoint = msgPositionSetpoint;
    if(m_pActiveCrazyController!=NULL)
        m_pActiveCrazyController->setSetpoint(m_positionSetpoint);
   // ROS_INFO_STREAM("Setpoint changed: "<<m_positionSetpoint.x<<", "<<m_positionSetpoint.y<<
   //                 ", "<<m_positionSetpoint.z<<", "<<m_positionSetpoint.yaw);

}

void CControlMgr::callbackFeedforwardCmdChanged(const crazypkg::MotorCommands &feedforwardCmd)
{
    //TODO: send it to all controllers.. no need for this if ff is managed in control mgr
    ROS_INFO("Setting FeedForward Cmd");
    m_feedforwardCommands=feedforwardCmd;
    m_pCrazyPIDController->setFeedforwardCommands(feedforwardCmd);
    m_pCrazyLQRNestedController->setFeedforwardCommands(feedforwardCmd);
    m_pRateController->setFeedforwardCommands(feedforwardCmd);
}

void CControlMgr::callbackControllerParametersChanged(const crazypkg::ControllerParam& msgParameter)
{
    //ROS_INFO("Changing controller param");
    //TODO; send the parameter to the controller
    switch (msgParameter.crazyControllerType)
    {
    case ePID: {m_pCrazyPIDController->setControllerParameter(msgParameter); break;}
    case eLQRFull: {/*m_pLQRController->setParameter(msgParameter);*/ break;}
    case eLQRNested: { m_pCrazyLQRNestedController->setControllerParameter(msgParameter); break;}
    case eMotorCmdTest: {m_pMotorCmdTestController->setControllerParameter(msgParameter); break;}
    case eAngleCmdTest: {m_pAngleCmdTestController->setControllerParameter(msgParameter); break;}
    case eRateCmdTest: {m_pRateCmdTestController->setControllerParameter(msgParameter); break;}
    case eRate: {m_pRateController->setControllerParameter(msgParameter); break;}
    default: ROS_ERROR("wrong crazyControllerType in callbackControllerParametersChanged");
    }
    //ROS_INFO_STREAM("param value: "<<msgParameter.value);
}

void CControlMgr::callbackControllerTypeChanged(const std_msgs::Int32& msg)
{
    //if(m_isStopped==true) return;
//ROS_INFO("changing Controleer Type");
    if(m_pActiveCrazyController!=NULL)
        if(msg.data==m_pActiveCrazyController->getControllerType())
            return;

    switch (msg.data)
    {
    case ePIDAngle:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_pCrazyPIDController->setControllerType(ePIDAngle);
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pCrazyPIDController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        ROS_INFO("controller changed to PIDAngle");
        break;
    }
    case ePIDPosition:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_pCrazyPIDController->setControllerType(ePIDPosition);
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pCrazyPIDController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        ROS_INFO("controller changed to PIDPosition");
        break;
    }
    case ePIDFull:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_pCrazyPIDController->setControllerType(ePIDFull);
        m_isRateOffboard=true;
        m_pActiveCrazyController=m_pCrazyPIDController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerRateController->setPeriod(ros::Duration(m_pRateController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        m_pTimerRateController->start();
        ROS_INFO("controller changed to PIDFull");
        break;
    }
    case eLQRFull:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pCrazyLQRFullController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerRateController->setPeriod(ros::Duration(m_pRateController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        m_pTimerRateController->start();
        ROS_INFO("controller changed to LQRFull");
        break;
    }
    case eLQRNestedOnboardRate:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_pCrazyLQRNestedController->setControllerType(eLQRNestedOnboardRate);
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pCrazyLQRNestedController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        ROS_INFO("controller changed to LQRNestedOnboardRate");
        break;
    }
    case eLQRNestedOffboardRate:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_pCrazyLQRNestedController->setControllerType(eLQRNestedOffboardRate);
        m_isRateOffboard=true;
        m_pActiveCrazyController=m_pCrazyLQRNestedController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerRateController->setPeriod(ros::Duration(m_pRateController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        m_pTimerRateController->start();
        ROS_INFO("controller changed to LQRNestedOffboardRate");
        break;
    }
    case eMotorCmdTest:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pMotorCmdTestController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        ROS_INFO("controller changed to MotorCmdTest");
        break;
    }
    case eAngleCmdTest:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pAngleCmdTestController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        ROS_INFO("controller changed to AngleCmdTest");
        break;
    }
    case eRateCmdTest:
    {
        m_pTimerCrazyController->stop();
        m_pTimerRateController->stop();
        m_isRateOffboard=false;
        m_pActiveCrazyController=m_pRateCmdTestController;
        m_pTimerCrazyController->setPeriod(ros::Duration(m_pActiveCrazyController->getSamplePeriod()));
        m_pTimerCrazyController->start();
        ROS_INFO("controller changed to RateCmdTest");
        break;
    }
    default: ROS_ERROR("wrong eControllerType in change controller callback");
    }

    if(m_pActiveCrazyController!=NULL) //TODO: if condition should be removed once all controllers
        //are implemented
    {
        m_pActiveCrazyController->setSetpoint(m_positionSetpoint);
        m_pActiveCrazyController->setStateEstimation(m_fullStateEstimation);
    }
}

void CControlMgr::callbackSampleTimeChanged(const crazypkg::SampleTimeParam& msgTs)
{
//ROS_INFO("changing sampleTime");

    switch(msgTs.sampleTimeType)
    {
    case ePIDTs:
    {
        m_pCrazyPIDController->setSamplePeriod(msgTs.value);
        if(m_pActiveCrazyController!=NULL)
            if(m_pActiveCrazyController->getControllerType()==ePIDAngle ||
                    m_pActiveCrazyController->getControllerType()==ePIDPosition ||
                    m_pActiveCrazyController->getControllerType()==ePIDFull)
            {
                m_pTimerCrazyController->setPeriod(ros::Duration(msgTs.value));
            }
        ROS_INFO_STREAM("PID Ts changed to: "<<msgTs.value);
        break;
    }
    case eLQRNestedTs:
    {
        m_pCrazyLQRNestedController->setSamplePeriod(msgTs.value);
        if(m_pActiveCrazyController!=NULL)
            if(m_pActiveCrazyController->getControllerType()==eLQRNestedOffboardRate ||
                    m_pActiveCrazyController->getControllerType()==eLQRNestedOnboardRate )
            {
                m_pTimerCrazyController->setPeriod(ros::Duration(msgTs.value));
            }
        ROS_INFO_STREAM("LQRNested Ts changed to: "<<msgTs.value);
        break;
    }
    case eLQRFullTs:
    {
        //m_pCrazyLQRFullController->setSamplePeriod(msgTs.value);
        if(m_pActiveCrazyController!=NULL)
            if(m_pActiveCrazyController->getControllerType()==eLQRFull)
            {
                m_pTimerCrazyController->setPeriod(ros::Duration(msgTs.value));
            }
        ROS_INFO_STREAM("LQRNested Ts changed to: "<<msgTs.value);
        break;
    }
    case eRateTs:
    {
        m_pRateController->setSamplePeriod(msgTs.value);
        if(m_pRateController!=NULL)
            if(m_isRateOffboard)
            {
                m_pTimerRateController->setPeriod(ros::Duration(msgTs.value));
            }
        ROS_INFO_STREAM("Rate Ts changed to: "<<msgTs.value);
        break;
    }
    default: ROS_ERROR("wrong eSampleTimeType in callback");
    }
}


void CControlMgr::callbackViconData(const crazypkg::ViconData& msgViconData)
{
    if(fabs(msgViconData.x)>0.00001 && fabs(msgViconData.y)>0.00001 && fabs(msgViconData.z)>0.00001)
    {
        m_ViconDataPrev=m_ViconData;
        m_ViconData=msgViconData;
        m_ViconData.x/=1000;
        m_ViconData.y/=1000;
        m_ViconData.z/=1000;

        if(m_isCalibrating)
        {
            m_calCnt++;
            m_calRollSum+=m_ViconData.roll;
            m_calPitchSum+=m_ViconData.pitch;
        }

        m_ViconData.roll-=m_RollOffset;
        m_ViconData.pitch-=m_PitchOffset;

        m_derivativeEstimator->pushMeasurement(FullStateEstimation
                         (m_ViconData.acquiringTime, m_ViconData.x,m_ViconData.y,m_ViconData.z,
                                  m_ViconData.yaw,m_ViconData.pitch,m_ViconData.roll));

        m_SSKalmanEstimator->pushMeasurement(FullStateEstimation
                         (m_ViconData.acquiringTime, m_ViconData.x,m_ViconData.y,m_ViconData.z,
                                  m_ViconData.yaw,m_ViconData.pitch,m_ViconData.roll));

        m_fullStateEstimation=m_SSKalmanEstimator->estimateState();
        m_derivativeEstimation=m_derivativeEstimator->estimateState();




        if(m_pActiveCrazyController!=NULL)
        {
            m_pActiveCrazyController->setStateEstimation(m_fullStateEstimation);
            m_pRateController->setStateEstimation(m_fullStateEstimation);
        }

    }
    else
    {
        m_cntViconDataMissed++;
        m_pPublisherCntViconDataMissed->publish(m_cntViconDataMissed);
    }
}

void CControlMgr::AddFeedforwardToMotorCommands(ControllerOutput* output)
{
    output->motorCmd1+=m_feedforwardCommands.cmd1;
    output->motorCmd2+=m_feedforwardCommands.cmd2;
    output->motorCmd3+=m_feedforwardCommands.cmd3;
    output->motorCmd4+=m_feedforwardCommands.cmd4;
}

void CControlMgr::DistributePowerAndSendToCrazyflie(ControllerOutput rateControllerOutput)
{
    assert(rateControllerOutput.onboardControllerType=eOnboardMotorCmdController);
    assert(m_isRateOffboard==true);


    rateControllerOutput.motorCmd1=m_CrazyControllerOutput.motorCmd1-rateControllerOutput.roll/2.0-rateControllerOutput.pitch/2.0-rateControllerOutput.yaw;
    rateControllerOutput.motorCmd2=m_CrazyControllerOutput.motorCmd2-rateControllerOutput.roll/2.0+rateControllerOutput.pitch/2.0+rateControllerOutput.yaw;
    rateControllerOutput.motorCmd3=m_CrazyControllerOutput.motorCmd3+rateControllerOutput.roll/2.0+rateControllerOutput.pitch/2.0-rateControllerOutput.yaw;
    rateControllerOutput.motorCmd4=m_CrazyControllerOutput.motorCmd4+rateControllerOutput.roll/2.0-rateControllerOutput.pitch/2.0+rateControllerOutput.yaw;

    SendToCrazyflie(rateControllerOutput);
}

void CControlMgr::SendToCrazyflie(ControllerOutput package)
{
    if(m_isStopped)
    {
        m_packageToSend.motorCmd1=0;
        m_packageToSend.motorCmd2=0;
        m_packageToSend.motorCmd3=0;
        m_packageToSend.motorCmd4=0;
        m_packageToSend.onboardControllerType=eOnboardMotorCmdController;
        m_pPublisherControllerOutput->publish(m_packageToSend);
        return;
    }
    else
    {
        m_packageToSend.roll=package.roll*RAD2DEG;
        m_packageToSend.pitch=package.pitch*RAD2DEG;
        m_packageToSend.yaw=package.yaw*RAD2DEG;

        m_packageToSend.thrust=SaturateToUINT16(package.thrust);
        m_packageToSend.motorCmd1=SaturateToUINT16(package.motorCmd1);
        m_packageToSend.motorCmd2=SaturateToUINT16(package.motorCmd2);
        m_packageToSend.motorCmd3=SaturateToUINT16(package.motorCmd3);
        m_packageToSend.motorCmd4=SaturateToUINT16(package.motorCmd4);

        m_packageToSend.onboardControllerType=package.onboardControllerType;

        m_pPublisherControllerOutput->publish(m_packageToSend);
    }
}



ros::Time lasttime;
ros::Time newtime;

void CControlMgr::callbackRunCrazyController(const ros::TimerEvent&)
{
    if(m_isStopped) return;
//    newtime=ros::Time::now();
//    ROS_INFO_STREAM_THROTTLE(0.01,"loop period: "<<(newtime-lasttime).toSec());
//    lasttime=newtime;

    if(m_pActiveCrazyController!=NULL)
    {
        if(m_pActiveCrazyController->fIsInit())
        {
            m_CrazyControllerOutput=m_pActiveCrazyController->computeOutput();

            if(m_isRateOffboard)
                m_pRateController->setSetpoint(m_CrazyControllerOutput.roll,m_CrazyControllerOutput.pitch,m_CrazyControllerOutput.yaw);
            else
                SendToCrazyflie(m_CrazyControllerOutput);
        }
        logData();
    }
//DistributePowerAndSendToCrazyflie(m_pRateController->computeOutput());





  //  ROS_INFO_STREAM_THROTTLE(0.01,"inner control loop execution time: "<<(ros::Time::now()-newtime).toSec());
}

void CControlMgr::callbackRunRateController(const ros::TimerEvent&)
{
    if(m_pRateController!=NULL)
        if(m_pRateController->fIsInit())
        {
            DistributePowerAndSendToCrazyflie(m_pRateController->computeOutput());
        }
}

void CControlMgr::printInfo()
{
    ROS_INFO("**************** PRINTING INFORMATION ****************");

    ROS_INFO_STREAM("*** quadrotor stopped: "<<m_isStopped);
    ROS_INFO_STREAM("*** rate controller offboard : "<<m_isRateOffboard);


    switch (m_pActiveCrazyController->getControllerType())
    {
    case ePIDFull:{ ROS_INFO("*** active controller: PIDFull"); break; }
    case ePIDAngle:{ ROS_INFO("*** active controller: PIDAngle"); break; }
    case ePIDPosition:{ ROS_INFO("*** active controller: PIDPosition"); break; }
    case eLQRFull:{ ROS_INFO("*** active controller: LQRFull"); break; }
    case eLQRNestedOffboardRate:{ ROS_INFO("*** active controller: LQR Nested Offboard rate controller"); break; }
    case eLQRNestedOnboardRate:{ ROS_INFO("*** active controller: LQR Nested Onboard rate controller"); break; }
    case eMotorCmdTest:{ ROS_INFO("*** active controller: MotorCmdTest"); break; }
    case eAngleCmdTest:{ ROS_INFO("*** active controller: AngleCmdTest"); break; }
    case eRateCmdTest:{ ROS_INFO("*** active controller: RateCmdTest"); break; }
    default: ROS_ERROR("wrong eControllerType in change controller callback");
    }

    ROS_INFO_STREAM("*** last package sent: ");
    ROS_INFO_STREAM(m_packageToSend);
    ROS_INFO_STREAM("*** position setpoint: ");
    ROS_INFO_STREAM(m_positionSetpoint);

    ROS_INFO_STREAM("*** roll offset: ");
    ROS_INFO_STREAM(m_RollOffset);
    ROS_INFO_STREAM("*** pitch offset: ");
    ROS_INFO_STREAM(m_PitchOffset);

    ROS_INFO("************ CONTROLLER INFORMATION ************");

    m_pCrazyPIDController->printInfo();
    m_pCrazyLQRNestedController->printInfo();
    m_pRateController->printInfo();
    m_pMotorCmdTestController->printInfo();
    m_pAngleCmdTestController->printInfo();
    m_pRateCmdTestController->printInfo();


}

void CControlMgr::logData()
{
    streamTime<<std::fixed<<ros::Time::now().toSec()-logTimeOffset<<std::endl;

    streamStateEstimation<<m_fullStateEstimation.x<<" "<<
                           m_fullStateEstimation.y<<" "<<
                           m_fullStateEstimation.z<<" "<<
                           m_fullStateEstimation.x_dot<<" "<<
                           m_fullStateEstimation.y_dot<<" "<<
                           m_fullStateEstimation.z_dot<<" "<<
                           m_fullStateEstimation.roll<<" "<<
                           m_fullStateEstimation.pitch<<" "<<
                           m_fullStateEstimation.yaw<<" "<<
                           m_fullStateEstimation.roll_dot<<" "<<
                           m_fullStateEstimation.pitch_dot<<" "<<
                           m_fullStateEstimation.yaw_dot<<std::endl;

    streamDerEstimation<<m_derivativeEstimation.x<<" "<<
                           m_derivativeEstimation.y<<" "<<
                           m_derivativeEstimation.z<<" "<<
                           m_derivativeEstimation.x_dot<<" "<<
                           m_derivativeEstimation.y_dot<<" "<<
                           m_derivativeEstimation.z_dot<<" "<<
                           m_derivativeEstimation.roll<<" "<<
                           m_derivativeEstimation.pitch<<" "<<
                           m_derivativeEstimation.yaw<<" "<<
                           m_derivativeEstimation.roll_dot<<" "<<
                           m_derivativeEstimation.pitch_dot<<" "<<
                           m_derivativeEstimation.yaw_dot<<std::endl;

    streamSetpoint<<m_positionSetpoint.x<<" "<<
                    m_positionSetpoint.y<<" "<<
                    m_positionSetpoint.z<<" "<<
                    m_positionSetpoint.yaw<<std::endl;

    streamPackageSentToCF<<m_packageToSend.roll<<" "<<
                           m_packageToSend.pitch<<" "<<
                           m_packageToSend.yaw<<" "<<
                           m_packageToSend.motorCmd1<<" "<<
                           m_packageToSend.motorCmd2<<" "<<
                           m_packageToSend.motorCmd3<<" "<<
                           m_packageToSend.motorCmd4<<std::endl;

    /*  ROS_INFO_NAMED("time","%d",ros::Time::now().toSec());

    ROS_INFO_STREAM_NAMED("fullStateEstimation",
                          m_fullStateEstimation.x<<" "<<
                          m_fullStateEstimation.y<<" "<<
                          m_fullStateEstimation.z<<" "<<
                          m_fullStateEstimation.x_dot<<" "<<
                          m_fullStateEstimation.y_dot<<" "<<
                          m_fullStateEstimation.z_dot<<" "<<
                          m_fullStateEstimation.roll<<" "<<
                          m_fullStateEstimation.pitch<<" "<<
                          m_fullStateEstimation.yaw<<" "<<
                          m_fullStateEstimation.roll_dot<<" "<<
                          m_fullStateEstimation.pitch_dot<<" "<<
                          m_fullStateEstimation.yaw_dot);

    ROS_INFO_STREAM_NAMED("setpoint",
                          m_positionSetpoint.x<<" "<<
                          m_positionSetpoint.y<<" "<<
                          m_positionSetpoint.z<<" "<<
                          m_positionSetpoint.yaw);

    ROS_INFO_STREAM_NAMED("packageSentToCF",
                          m_packageToSend.roll<<" "<<
                          m_packageToSend.pitch<<" "<<
                          m_packageToSend.yaw<<" "<<
                          m_packageToSend.motorCmd1<<" "<<
                          m_packageToSend.motorCmd2<<" "<<
                          m_packageToSend.motorCmd3<<" "<<
                          m_packageToSend.motorCmd4);
                          */
}

void CControlMgr::closeLog()
{
    streamPackageSentToCF.close();
    streamSetpoint.close();
    streamStateEstimation.close();
    streamTime.close();
}


