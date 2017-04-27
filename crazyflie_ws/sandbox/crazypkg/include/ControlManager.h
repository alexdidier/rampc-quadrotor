#pragma once

#include <iostream>
#include <fstream>
#include <sstream>

class CStateEstimator
{
public:
  CStateEstimator(double frequency):m_frequency(frequency){}
  virtual FullStateEstimation estimateState()=0;
  void pushMeasurement(FullStateEstimation measurement){m_prevMeasurement=m_measurement; m_measurement=measurement;}
  virtual void reset()=0;
protected:
  FullStateEstimation m_measurement;
  FullStateEstimation m_prevMeasurement;
  FullStateEstimation m_prevEstimation;
  FullStateEstimation m_estimation;
  double m_frequency;
};
class discreteDerivativeEstimator:public CStateEstimator
{
public:
  discreteDerivativeEstimator(double frequency):CStateEstimator(frequency){}
  FullStateEstimation estimateState();
  void reset() {m_prevMeasurement=m_measurement; m_prevEstimation=m_measurement;}
};
class SSKalmanFilter:public CStateEstimator
{
public:
  SSKalmanFilter(double frequency);
  FullStateEstimation estimateState();
  void reset() {m_prevMeasurement=m_measurement; m_prevEstimation=m_measurement;}
private:
  double K[6];
  double Kx[6];
  double Ahat[2];
  double Ahatx1[6];

};


class CControlMgr
{
public:

    void init(ros::NodeHandle *nodeHandle);
    void closeLog();

private:

    void callbackRunCrazyController(const ros::TimerEvent&);
    void callbackRunRateController(const ros::TimerEvent&);

    void callbackViconData(const crazypkg::ViconData& msgViconData);

    void callbackPositionSetpointChanged(const crazypkg::PositionSetpoint& msgPositionSetpoint);

    void callbackControllerParametersChanged(const crazypkg::ControllerParam& msgParameter);

    void callbackSampleTimeChanged(const crazypkg::SampleTimeParam& msgTs);

    void callbackControllerTypeChanged(const std_msgs::Int32& msg);

    void callbackDoSomething(const std_msgs::Int32& msg);

    void callbackFeedforwardCmdChanged(const crazypkg::MotorCommands& feedforwardCmd);


    void StopQuadrotor();
    void StartQuadrotor();
    void printInfo();
    void logData();
    void resetControllers();

    void DistributePowerAndSendToCrazyflie(ControllerOutput rateControllerOutput);
    void SendToCrazyflie(ControllerOutput package);
    void AddFeedforwardToMotorCommands(ControllerOutput* output);

    bool m_isStopped;
    bool m_isRateOffboard;

    bool m_isCalibrating;
    long int m_calCnt;
    double m_calRollSum;
    double m_calPitchSum;
    double m_RollOffset;
    double m_PitchOffset;


    ros::NodeHandle* m_pNodeHandle;
    ros::CallbackQueue* m_pCallbackQueueControlMgr;

    ros::Subscriber* m_pSubscriberViconData;
    ros::Subscriber* m_pSubscriberPositionSetpoint;
    ros::Subscriber* m_pSubscriberSampleTimeParam;
    ros::Subscriber* m_pSubscriberControllerParam;
    ros::Subscriber* m_pSubscriberControllerType;
    ros::Subscriber* m_pSubscriberDoSomething;
    ros::Subscriber* m_pSubscriberFeedforwardCmd;

    ros::Publisher* m_pPublisherControllerOutput;
    ros::Publisher* m_pPublisherCntViconDataMissed;

    std::ofstream streamStateEstimation;
    std::ofstream streamDerEstimation;
    std::ofstream streamSetpoint;
    std::ofstream streamPackageSentToCF;
    std::ofstream streamTime;
    double logTimeOffset;


    ros::AsyncSpinner* m_pAsyncSpinner;
    ros::Timer* m_pTimerCrazyController;
    ros::Timer* m_pTimerRateController;

    crazypkg::ViconData m_ViconData;
    crazypkg::ViconData m_ViconDataPrev;
    FullStateEstimation m_fullStateEstimation;
    FullStateEstimation m_derivativeEstimation;
    CStateEstimator* m_derivativeEstimator;
    CStateEstimator* m_SSKalmanEstimator;
    int m_cntViconDataMissed;

    CCrazyController* m_pMotorCmdTestController;
    CCrazyController* m_pAngleCmdTestController;
    CCrazyController* m_pRateCmdTestController;
    CCrazyController* m_pCrazyPIDController;
    CCrazyController* m_pCrazyLQRNestedController;
    CCrazyController* m_pCrazyLQRFullController;

    CRateController* m_pRateController;

    CCrazyController* m_pActiveCrazyController;


    ControllerOutput m_CrazyControllerOutput;
    crazypkg::ControllerOutputPackage m_packageToSend;

    crazypkg::PositionSetpoint m_positionSetpoint;
    crazypkg::MotorCommands m_feedforwardCommands;
};

CControlMgr* pGetControlMgr();




