#pragma once

#include "crazypkg/ViconData.h"
#include "crazypkg/PositionSetpoint.h"
#include "crazypkg/ControllerParam.h"
#include "crazypkg/SampleTimeParam.h"
#include "crazypkg/ControllerOutputPackage.h"
#include "crazypkg/MotorCommands.h"
#include "std_msgs/Int32.h"

#define RAD2DEG 57.2957795
#define DEG2RAD 0.017453293
#define PI 3.14159265359



enum EOnboardControllerType
{
  eOnboardRateController=0,
  eOnboardAngleController=1,
  eOnboardMotorCmdController=2
};

enum EDoSomething
{
  ePrintInfo,
  eStartQuad,
  eStopQuad,
  eResetControllers,
  eResetCntMissedViconData,
  eStartCal,
  eStopCal
};

enum EControllerType
{
  ePID,
  eLQRFull,
  eLQRNested,
  eLQRNestedOnboardRate,
  eLQRNestedOffboardRate,
  ePIDPosition,
  ePIDAngle,
  ePIDFull,
  eMotorCmdTest,
  eAngleCmdTest,
  eRateCmdTest,
  eRate
};

//this has to be in exact same order as in the GUI table
enum EPIDControllers
{
  ePIDX,
  ePIDY,
  ePIDZ,
  ePIDYaw,
  ePIDPitch,
  ePIDRoll,

  countPIDControllers //last enum field
};

//this has to be in exact same order as in the GUI table
enum ERateControllers
{
  ePIDYawRate,
  ePIDPitchRate,
  ePIDRollRate,

  countRateControllers //last enum field
};


enum ESampleTimeType
{
  ePIDTs,
  eLQRFullTs,
  eLQRNestedTs,
  eRateTs,

  countSampleTimeTypes  //last enum field
};

//this has to be in exact same order as in the GUI table
enum EPIDParams
{
  eKp,
  eKi,
  eKd,
  eN,
  eMinPIDSat,
  eMaxPIDSat,

  countPIDParams //last enum field
};

enum ETestControllerParams
{
  eTestMotorCmd,
  eTestRoll,
  eTestPitch,
  eTestYaw
};

struct ControllerOutput
{
  ControllerOutput():roll(0),pitch(0),yaw(0),thrust(0),motorCmd1(0),motorCmd2(0),motorCmd3(0),motorCmd4(0) {}
  float roll;
  float pitch;
  float yaw;
  float thrust;
  float motorCmd1;
  float motorCmd2;
  float motorCmd3;
  float motorCmd4;
  uint8_t onboardControllerType;
};

struct FullStateEstimation
{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
  double x_dot;
  double y_dot;
  double z_dot;
  double yaw_dot;
  double pitch_dot;
  double roll_dot;

  double acquiringTime;
  FullStateEstimation(double t=-1, double ax=0,double ay=0,double az=0, double ayaw=0, double apitch=0,
                      double aroll=0, double ax_dot=0, double ay_dot=0, double az_dot=0,
                      double ayaw_dot=0, double apitch_dot=0, double aroll_dot=0):
    acquiringTime(t), x(ax),y(ay),z(az),yaw(ayaw),pitch(apitch),roll(aroll),x_dot(ax_dot),y_dot(ay_dot),
    z_dot(az_dot),yaw_dot(ayaw_dot),pitch_dot(apitch_dot),roll_dot(aroll_dot) {}

};

struct PIDParams
{
  double Kp;
  double Ki;
  double Kd;
  double N;
  double MinSat;
  double MaxSat;
  PIDParams(double aKp=0,double aKi=0,double aKd=0,double aN=0, double aMin=0,double aMax=0):
    Kp(aKp),Ki(aKi),Kd(aKd),N(aN),MinSat(aMin),MaxSat(aMax) {}
};


/*class CMotorCommands
{
public:
    CMotorCommands(double cmd1=0,double cmd2=0,double cmd3=0,double cmd4=0);
    double m_cmd1;
    double m_cmd2;
    double m_cmd3;
    double m_cmd4;
};*/

uint16_t SaturateToUINT16(float input);

