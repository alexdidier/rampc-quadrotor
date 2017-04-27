#ifndef CRAZYFLIECONTROLLERS_H
#define CRAZYFLIECONTROLLERS_H

//crazyflie control loop interface
class CCrazyController
{
public:
    virtual ControllerOutput computeOutput()=0;
    void setSetpoint(crazypkg::PositionSetpoint positionSetpoint);
    EControllerType getControllerType();
    void setControllerType(EControllerType type);
    void setStateEstimation(FullStateEstimation fullStateEstimation);
    virtual void setControllerParameter(const crazypkg::ControllerParam& parameter)=0;
    virtual void setSamplePeriod(double Ts);
    virtual double getSamplePeriod();
    virtual void printInfo();
    virtual void reset()=0;
    virtual bool fIsInit()=0;
    virtual void setFeedforwardCommands(crazypkg::MotorCommands feedforwardCmd);
protected:
    crazypkg::PositionSetpoint m_positionSetpoint;
    FullStateEstimation m_fullStateEstimation;
    ControllerOutput m_ControllerOutput;
    double m_Ts;
    EControllerType m_controllerType;
    crazypkg::MotorCommands m_feedforwardMotorCommands;
};



class CMotorCmdTestController: public CCrazyController
{
public:
    CMotorCmdTestController();
    virtual ControllerOutput computeOutput() {return m_ControllerOutput;}
    void setControllerParameter(const crazypkg::ControllerParam &parameter);
    void reset() {}
    void printInfo();
    bool fIsInit() {return true;}
    EControllerType getControllerType() {return eMotorCmdTest; }
    void setControllerType(EControllerType type) {assert(type==eMotorCmdTest);}
    double getSamplePeriod() {return 0.123;}
};

class CAngleCmdTestController: public CCrazyController
{
public:
  CAngleCmdTestController();
  virtual ControllerOutput computeOutput() {return m_ControllerOutput;}
  void setControllerParameter(const crazypkg::ControllerParam &parameter);
  void reset() {}
  void printInfo();
  bool fIsInit() {return true;}
  EControllerType getControllerType() {return eAngleCmdTest;}
  void setControllerType(EControllerType type) {assert(type==eAngleCmdTest);}
  double getSamplePeriod() {return 0.123;}
};

class CRateCmdTestController: public CCrazyController
{
public:
  CRateCmdTestController();
  virtual ControllerOutput computeOutput() {return m_ControllerOutput;}
  void setControllerParameter(const crazypkg::ControllerParam &parameter);
  void reset() {}
  void printInfo();
  bool fIsInit() {return true;}
  EControllerType getControllerType() {return eRateCmdTest;}
  void setControllerType(EControllerType type) {assert(type==eRateCmdTest);}
  double getSamplePeriod() {return 0.123;}
};

class CPIDController
{
public:
  CPIDController();
  virtual double computeOutput(double error);
  virtual void reset();
  bool fIsInit();
  void setControllerParameter(const crazypkg::ControllerParam& parameter);
  void setSamplePeriod(double Ts);
  virtual void printInfo();
protected:
  void computeInternalParams();
  PIDParams m_params;
  PIDParams m_ParamInit;
  double m_output_1;
  double m_output_2;
  double m_error_1;
  double m_error_2;
  double m_output;
  double m_Ts;
  bool m_isInit;

  //internal params
  double m_a0,m_NT2,m_NKd,m_KiT2,m_a2;
  double m_pder; // 1/(1+NT)
  double m_p1,m_p2,m_p3,m_p4,m_p5;

};

class CClassicPIDController: public CPIDController
{
public:
  double computeOutput(double error);
  CClassicPIDController();
  void reset();
  void printInfo();
private:
  double m_integrator;
};

class CFilteredPIDController: public CPIDController
{
public:
  double computeOutput(double error);
  CFilteredPIDController();
  void reset();
  void printInfo();
private:
  double m_integrator;
  double m_derivative_1;
  double m_derivative;
};

class CRateController: public CCrazyController
{
public:
  CRateController();
  ControllerOutput computeOutput();
  EControllerType getControllerType();
  void setSetpoint(double p, double q, double r); //omega setpoint (roll,pitch,yaw)
  void setControllerParameter(const crazypkg::ControllerParam& parameter);
  void setSamplePeriod(double Ts);
  void printInfo();
  void reset();
  bool fIsInit();
private:
  void updateOmega();

  CFilteredPIDController* m_pPIDRollRate;
  CFilteredPIDController* m_pPIDPitchRate;
  CFilteredPIDController* m_pPIDYawRate;

  double m_sinYaw, m_cosYaw;
  double m_sinRoll, m_cosRoll;
  double m_sinPitch, m_cosPitch;

  double m_omega[3]; // p,q,r (roll, pitch, yaw)
  double m_omegaSP[3]; // p,q,r setpoint

  bool m_isInit;


};

class CCrazyPIDController: public CCrazyController
{
public:
  CCrazyPIDController();
  ControllerOutput computeOutput();
  void setControllerParameter(const crazypkg::ControllerParam& parameter);
  void setSamplePeriod(double Ts);
  void printInfo();
  void reset();
  bool fIsInit();

private:
  CPIDController* m_pPIDX;
  CPIDController* m_pPIDY;
  CPIDController* m_pPIDZ;
  CPIDController* m_pPIDYaw;
  CPIDController* m_pPIDPitch;
  CPIDController* m_pPIDRoll;
  bool m_isInit;

  double m_sinYaw, m_cosYaw;
  double m_Xerr, m_Yerr, m_Zerr;
  double m_inputPIDX, m_inputPIDY;
  double m_outputPIDZ,m_outputPIDY,m_outputPIDX;

};

class CCrazyLQRNestedController: public CCrazyController
{
public:
  CCrazyLQRNestedController();
  ControllerOutput computeOutput();
  void setControllerParameter(const crazypkg::ControllerParam& parameter);
  void setFeedforwardCommands(crazypkg::MotorCommands feedforwardCmd);
  void setSamplePeriod(double Ts);
  void printInfo();
  void reset();
  bool fIsInit();

private:
  bool m_isInit;
  double m_K[4][9];
  double m_KisInit[4][9];
  double m_controllerInput[9];
  double m_a2,m_a1,m_a0;
  double m_sinYaw, m_cosYaw;
  double m_Xerr, m_Yerr, m_Xdot_err,m_Ydot_err;
  double m_ffCmd1Thrust, m_ffCmd2Thrust, m_ffCmd3Thrust, m_ffCmd4Thrust;
  double m_saturationThrust;

};




#endif // CRAZYFLIECONTROLLERS_H
