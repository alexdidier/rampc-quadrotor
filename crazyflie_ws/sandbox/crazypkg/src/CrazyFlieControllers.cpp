#include "CrazyFlieInclude.h"


void CCrazyController::setSetpoint(crazypkg::PositionSetpoint positionSetpoint)
{
    m_positionSetpoint=positionSetpoint;
}

EControllerType CCrazyController::getControllerType()
{
    return m_controllerType;
}

void CCrazyController::setControllerType(EControllerType type)
{
    m_controllerType=type;
}

void CCrazyController::setStateEstimation(FullStateEstimation fullStateEstimation)
{
    m_fullStateEstimation=fullStateEstimation;
}

void CCrazyController::setSamplePeriod(double Ts)
{
    m_Ts=Ts;
}

double CCrazyController::getSamplePeriod()
{
    return m_Ts;
}

void CCrazyController::setFeedforwardCommands(crazypkg::MotorCommands feedforwardCmd)
{
    m_feedforwardMotorCommands=feedforwardCmd;
}

void CCrazyController::printInfo()
{
    ROS_INFO_STREAM("*** Sample rate: "<<m_Ts);
    ROS_INFO_STREAM("*** Current Position Setpoint: ");
    ROS_INFO_STREAM(m_positionSetpoint);
    ROS_INFO_STREAM("*** Current State Estimation: ");
    ROS_INFO_STREAM("acquiring time: "<<m_fullStateEstimation.acquiringTime);
    ROS_INFO_STREAM("x: "<<m_fullStateEstimation.x);
    ROS_INFO_STREAM("y: "<<m_fullStateEstimation.y);
    ROS_INFO_STREAM("z: "<<m_fullStateEstimation.z);
    ROS_INFO_STREAM("yaw: "<<m_fullStateEstimation.yaw);
    ROS_INFO_STREAM("pitch: "<<m_fullStateEstimation.pitch);
    ROS_INFO_STREAM("roll: "<<m_fullStateEstimation.roll);
    ROS_INFO_STREAM("x_dot: "<<m_fullStateEstimation.x_dot);
    ROS_INFO_STREAM("y_dot: "<<m_fullStateEstimation.y_dot);
    ROS_INFO_STREAM("z_dot: "<<m_fullStateEstimation.z_dot);
    ROS_INFO_STREAM("yaw_dot: "<<m_fullStateEstimation.yaw_dot);
    ROS_INFO_STREAM("pitch_dot: "<<m_fullStateEstimation.pitch_dot);
    ROS_INFO_STREAM("roll_dot: "<<m_fullStateEstimation.roll_dot);
}

CMotorCmdTestController::CMotorCmdTestController()
{
    m_ControllerOutput.onboardControllerType=eOnboardMotorCmdController;
}

void CMotorCmdTestController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType==eMotorCmdTest);
    assert(parameter.basicControllerType==eTestMotorCmd);
    m_ControllerOutput.motorCmd1=parameter.value;
    m_ControllerOutput.motorCmd2=parameter.value;
    m_ControllerOutput.motorCmd3=parameter.value;
    m_ControllerOutput.motorCmd4=parameter.value;
}

void CMotorCmdTestController::printInfo()
{
    ROS_INFO("*****************************************************************");
    ROS_INFO("        MotorCmdTest CONTROLLER INFO:");
    CCrazyController::printInfo();
    ROS_INFO_STREAM("motor command to output: "<<m_ControllerOutput.motorCmd1);
}



CAngleCmdTestController::CAngleCmdTestController()
{
    m_ControllerOutput.onboardControllerType=eOnboardAngleController;
    m_ControllerOutput.motorCmd1=1;
    m_ControllerOutput.motorCmd2=1;
    m_ControllerOutput.motorCmd3=1;
    m_ControllerOutput.motorCmd4=1;
}

void CAngleCmdTestController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType==eAngleCmdTest);

    switch (parameter.basicControllerType)
    {
    case eTestRoll: { m_ControllerOutput.roll=parameter.value; break;}
    case eTestPitch: {m_ControllerOutput.pitch=parameter.value; break;}
    case eTestYaw: {m_ControllerOutput.yaw=parameter.value; break;}
    default: {ROS_ERROR("unknown parameter type in CAngleCmdTestController::setControllerParameter");
        break;}
    }
}

void CAngleCmdTestController::printInfo()
{
    ROS_INFO("*****************************************************************");
    ROS_INFO("        AngleCmdTest CONTROLLER INFO:");
    CCrazyController::printInfo();
    ROS_INFO_STREAM("angle setpoint: ");
    ROS_INFO_STREAM("roll [deg]: "<<m_ControllerOutput.roll*RAD2DEG);
    ROS_INFO_STREAM("pitch [deg]: "<<m_ControllerOutput.pitch*RAD2DEG);
    ROS_INFO_STREAM("yaw [deg]: "<<m_ControllerOutput.yaw*RAD2DEG);
}




CRateCmdTestController::CRateCmdTestController()
{
    m_ControllerOutput.onboardControllerType=eOnboardRateController;
    m_ControllerOutput.motorCmd1=1;
    m_ControllerOutput.motorCmd2=1;
    m_ControllerOutput.motorCmd3=1;
    m_ControllerOutput.motorCmd4=1;
}

void CRateCmdTestController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType==eRateCmdTest);

    switch (parameter.basicControllerType)
    {
    case eTestRoll: { m_ControllerOutput.roll=parameter.value; break;}
    case eTestPitch: {m_ControllerOutput.pitch=parameter.value; break;}
    case eTestYaw: {m_ControllerOutput.yaw=parameter.value; break;}
    default: {ROS_ERROR("unknown parameter type in CRateCmdTestController::setControllerParameter");
        break;}
    }
}

void CRateCmdTestController::printInfo()
{
    ROS_INFO("*****************************************************************");
    ROS_INFO("        RateCmdTest CONTROLLER INFO:");
    CCrazyController::printInfo();
    ROS_INFO_STREAM("rate setpoint: ");
    ROS_INFO_STREAM("roll [deg/s]: "<<m_ControllerOutput.roll*RAD2DEG);
    ROS_INFO_STREAM("pitch [deg/s]: "<<m_ControllerOutput.pitch*RAD2DEG);
    ROS_INFO_STREAM("yaw [deg/s]: "<<m_ControllerOutput.yaw*RAD2DEG);
}



CPIDController::CPIDController(): m_output(0),m_output_1(0),m_output_2(0),m_error_1(0),
    m_error_2(0),m_Ts(-1),m_isInit(false)
{

}

double CPIDController::computeOutput(double error)
{
    m_output=m_p1*m_output_1+m_p2*m_output_2+m_p3*error+m_p4*m_error_1+m_p5*m_error_2;

    if (m_output>m_params.MaxSat)
        m_output=m_params.MaxSat;
    else if(m_output<m_params.MinSat)
        m_output=m_params.MinSat;

    m_error_2=m_error_1;
    m_error_1=error;
    m_output_2=m_output_1;
    m_output_1=m_output;

    return m_output;
}

void CPIDController::reset()
{
    m_error_1=0;
    m_error_2=0;
    m_output_1=0;
    m_output_2=0;
    m_output=0;
}

bool CPIDController::fIsInit()
{
    return m_isInit;
}

void CPIDController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType==ePID);

    switch (parameter.paramType) {
    case eKp: { m_params.Kp=parameter.value; m_ParamInit.Kp=1; break; }
    case eKi: { m_params.Ki=parameter.value; m_ParamInit.Ki=1; break; }
    case eKd: { m_params.Kd=parameter.value; m_ParamInit.Kd=1; break; }
    case eN: { m_params.N=parameter.value; m_ParamInit.N=1; break; }
    case eMinPIDSat: { m_params.MinSat=parameter.value; m_ParamInit.MinSat=1; break; }
    case eMaxPIDSat: { m_params.MaxSat=parameter.value; m_ParamInit.MaxSat=1; break; }
    default: ROS_ERROR("wrong parameterType in PID Controller set parameter");
    }
    m_isInit=(m_ParamInit.Kp && m_ParamInit.Ki && m_ParamInit.Kd && m_ParamInit.N && m_ParamInit.MinSat && m_ParamInit.MaxSat);
    computeInternalParams();
}

void CPIDController::setSamplePeriod(double Ts)
{
    m_Ts=Ts;
    computeInternalParams();
}

void CPIDController::printInfo()
{
    ROS_INFO_STREAM("*** Parameters: ");
    ROS_INFO_STREAM("Kp: "<<m_params.Kp);
    ROS_INFO_STREAM("Ki: "<<m_params.Ki);
    ROS_INFO_STREAM("Kd: "<<m_params.Kd);
    ROS_INFO_STREAM("N: "<<m_params.N);
    ROS_INFO_STREAM("Min Saturation: "<<m_params.MinSat);
    ROS_INFO_STREAM("Max Saturation: "<<m_params.MaxSat);
    ROS_INFO_STREAM("Sample Period: "<<m_Ts);
    ROS_INFO_STREAM("*** PID Initialized: "<<fIsInit());
    ROS_INFO_STREAM("error1: "<<m_error_1);
    ROS_INFO_STREAM("error2: "<<m_error_2);
    ROS_INFO_STREAM("output1: "<<m_output_1);
    ROS_INFO_STREAM("output2: "<<m_output_2);
    ROS_INFO_STREAM("output: "<<m_output);
    ROS_INFO_STREAM("1/(1+NT)= "<<m_pder);
    ROS_INFO_STREAM("p1: "<<m_p1);
    ROS_INFO_STREAM("p2: "<<m_p2);
    ROS_INFO_STREAM("p3: "<<m_p3);
    ROS_INFO_STREAM("p4: "<<m_p4);
    ROS_INFO_STREAM("p5: "<<m_p5);
}

void CPIDController::computeInternalParams()
{
    if(fIsInit())
    {
        m_NT2=m_params.N*m_Ts/2;
        m_NKd=m_params.N*m_params.Kd;
        m_KiT2=m_params.Ki*m_Ts/2;

        m_pder=1/(m_params.N*m_Ts+1);

        m_a0=1+m_NT2;
        m_a2=1-m_NT2;

        m_p1=2/m_a0;
        m_p2=-m_a2/m_a0;
        m_p3=m_params.Kp+m_KiT2+m_NKd/m_a0;
        m_p4=(-2*m_params.Kp+m_KiT2*m_params.N*m_Ts-2*m_NKd)/m_a0;
        m_p5=(m_a2*(m_params.Kp-m_KiT2)+m_NKd)/m_a0;
    }
}

CClassicPIDController::CClassicPIDController():CPIDController(),m_integrator(0) {}

void CClassicPIDController::reset()
{
    CPIDController::reset();
    m_integrator=0;
}

void CClassicPIDController::printInfo()
{
    CPIDController::printInfo();
    ROS_INFO_STREAM("integrated value: "<<m_integrator);
}

double CClassicPIDController::computeOutput(double error)
{
    m_output=m_params.Kp*error+m_params.Ki*(m_integrator+m_Ts*error)+m_params.Kd*(error-m_error_1)/m_Ts;

    if((m_output<m_params.MaxSat)&&(m_output>m_params.MinSat))
        m_integrator+=m_Ts*error;
    else if (m_output>m_params.MaxSat)
        m_output=m_params.MaxSat;
    else if(m_output<m_params.MinSat)
        m_output=m_params.MinSat;

    m_error_2=m_error_1;
    m_error_1=error;
    m_output_2=m_output_1;
    m_output_1=m_output;

    return m_output;

}



CFilteredPIDController::CFilteredPIDController():CPIDController(),m_integrator(0),m_derivative_1(0) {}

void CFilteredPIDController::reset()
{
    CPIDController::reset();
    m_integrator=0;
    m_derivative_1=0;
}

void CFilteredPIDController::printInfo()
{
    CPIDController::printInfo();
    ROS_INFO_STREAM("integrated value: "<<m_integrator);
    ROS_INFO_STREAM("previous derivative output: "<<m_derivative_1);
}

double CFilteredPIDController::computeOutput(double error)
{
    m_derivative=m_pder*(m_derivative_1+m_params.Kd*m_params.N*(error-m_error_1));

    m_output=m_params.Kp*error+m_params.Ki*(m_integrator+m_Ts*error)+m_derivative;

    if((m_output<m_params.MaxSat)&&(m_output>m_params.MinSat))
        m_integrator+=m_Ts*error;
    else if (m_output>m_params.MaxSat)
        m_output=m_params.MaxSat;
    else if(m_output<m_params.MinSat)
        m_output=m_params.MinSat;

    m_derivative_1=m_derivative;
    m_error_2=m_error_1;
    m_error_1=error;
    m_output_2=m_output_1;
    m_output_1=m_output;

    return m_output;

}

CRateController::CRateController(): m_isInit(false)
{
    m_pPIDYawRate=new CFilteredPIDController();
    m_pPIDPitchRate=new CFilteredPIDController();
    m_pPIDRollRate=new CFilteredPIDController();
}

void CRateController::updateOmega()
{
    m_sinYaw=sin(m_fullStateEstimation.yaw);
    m_cosYaw=cos(m_fullStateEstimation.yaw);

    m_sinRoll=sin(m_fullStateEstimation.roll);
    m_cosRoll=cos(m_fullStateEstimation.roll);

    m_sinPitch=sin(m_fullStateEstimation.pitch);
    m_cosPitch=cos(m_fullStateEstimation.pitch);

    m_omega[0]=m_fullStateEstimation.roll_dot-m_sinPitch*m_fullStateEstimation.yaw_dot;
    m_omega[1]=m_cosRoll*m_fullStateEstimation.pitch_dot+m_sinRoll*m_cosPitch*m_fullStateEstimation.yaw_dot;
    m_omega[2]=-m_sinRoll*m_fullStateEstimation.pitch_dot+m_cosRoll*m_cosPitch*m_fullStateEstimation.yaw_dot;
}

void CRateController::setSetpoint(double p, double q, double r)
{
    m_omegaSP[0]=p;
    m_omegaSP[1]=q;
    m_omegaSP[2]=r;
}

ControllerOutput CRateController::computeOutput()
{
    updateOmega();

    m_ControllerOutput.roll= m_pPIDRollRate->computeOutput(m_omegaSP[0]-m_omega[0]);
    m_ControllerOutput.pitch= m_pPIDPitchRate->computeOutput(m_omegaSP[1]-m_omega[1]);
    m_ControllerOutput.yaw= m_pPIDYawRate->computeOutput(m_omegaSP[2]-m_omega[2]);
    m_ControllerOutput.onboardControllerType=eOnboardMotorCmdController;
    return m_ControllerOutput;
}

EControllerType CRateController::getControllerType()
{
    return eRate;
}

void CRateController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType=eRate);
    switch (parameter.basicControllerType)
    {
    case ePIDYawRate: { m_pPIDYawRate->setControllerParameter(parameter);  break;}
    case ePIDPitchRate: { m_pPIDPitchRate->setControllerParameter(parameter);  break;}
    case ePIDRollRate: { m_pPIDRollRate->setControllerParameter(parameter);  break;}
    default: ROS_ERROR("wrong rate controller type while setting a parameter");
    }

    m_isInit=(m_pPIDYawRate->fIsInit() && m_pPIDPitchRate->fIsInit() && m_pPIDRollRate->fIsInit());
}

void CRateController::setSamplePeriod(double Ts)
{
    CCrazyController::setSamplePeriod(Ts);
    m_pPIDYawRate->setSamplePeriod(Ts);
    m_pPIDPitchRate->setSamplePeriod(Ts);
    m_pPIDRollRate->setSamplePeriod(Ts);
}

void CRateController::reset()
{
    m_pPIDYawRate->reset();
    m_pPIDPitchRate->reset();
    m_pPIDRollRate->reset();
}

bool CRateController::fIsInit()
{
    return m_isInit;
}

void CRateController::printInfo()
{
    ROS_INFO("*****************************************************************");
    ROS_INFO("        Rate CONTROLLER INFO:");
    CCrazyController::printInfo();
    ROS_INFO_STREAM("Controller init: "<<fIsInit());
    ROS_INFO_STREAM("Last output sent (roll, pitch, yaw): ");
    ROS_INFO_STREAM(m_ControllerOutput.roll<<", "<<m_ControllerOutput.pitch<<", "<<m_ControllerOutput.yaw  );

    ROS_INFO("**************************");

    ROS_INFO("  PIDYawRate CONTROLLER INFO:");
    m_pPIDYawRate->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDPitchRate CONTROLLER INFO:");
    m_pPIDPitchRate->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDRollRate CONTROLLER INFO:");
    m_pPIDRollRate->printInfo();

}

CCrazyPIDController::CCrazyPIDController(): m_pPIDX(NULL), m_pPIDY(NULL), m_pPIDZ(NULL),
    m_pPIDYaw(NULL), m_pPIDPitch(NULL), m_pPIDRoll(NULL),m_isInit(false)
{
    m_pPIDX=new CClassicPIDController();
    m_pPIDY=new CClassicPIDController();
    m_pPIDZ=new CClassicPIDController();
    m_pPIDYaw=new CClassicPIDController();
    m_pPIDPitch=new CClassicPIDController();
    m_pPIDRoll=new CClassicPIDController();
}


void CCrazyPIDController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType=ePID);
    switch (parameter.basicControllerType)
    {
    case ePIDX: { m_pPIDX->setControllerParameter(parameter);  break;}
    case ePIDY: { m_pPIDY->setControllerParameter(parameter);  break;}
    case ePIDZ: { m_pPIDZ->setControllerParameter(parameter);  break;}
    case ePIDYaw: { m_pPIDYaw->setControllerParameter(parameter);  break;}
    case ePIDPitch: { m_pPIDPitch->setControllerParameter(parameter);  break;}
    case ePIDRoll: { m_pPIDRoll->setControllerParameter(parameter);  break;}
    default: ROS_ERROR("wrong basic controller type in PID controller set parameter");
    }

    m_isInit=(m_pPIDX->fIsInit() && m_pPIDY->fIsInit() && m_pPIDZ->fIsInit() &&
              m_pPIDYaw->fIsInit() && m_pPIDPitch->fIsInit() && m_pPIDRoll->fIsInit());
}


void CCrazyPIDController::setSamplePeriod(double Ts)
{
    CCrazyController::setSamplePeriod(Ts);
    m_pPIDX->setSamplePeriod(Ts);
    m_pPIDY->setSamplePeriod(Ts);
    m_pPIDZ->setSamplePeriod(Ts);
    m_pPIDYaw->setSamplePeriod(Ts);
    m_pPIDPitch->setSamplePeriod(Ts);
    m_pPIDRoll->setSamplePeriod(Ts);
}

void CCrazyPIDController::reset()
{
    m_pPIDX->reset();
    m_pPIDY->reset();
    m_pPIDZ->reset();
    m_pPIDYaw->reset();
    m_pPIDPitch->reset();
    m_pPIDRoll->reset();
}

bool CCrazyPIDController::fIsInit()
{
    return m_isInit;
}

void CCrazyPIDController::printInfo()
{
    ROS_INFO("*****************************************************************");
    ROS_INFO("        PID CONTROLLER INFO:");
    CCrazyController::printInfo();
    ROS_INFO_STREAM("Controller init: "<<fIsInit());
    ROS_INFO_STREAM("Last Motor Commands sent: ");
    //ROS_INFO_STREAM(m_motorCommands);
    ROS_INFO_STREAM("Feedforward motor controll: ");
    //ROS_INFO_STREAM(m_feedforwardMotorCommands);
    ROS_INFO("**************************");
    ROS_INFO("  PIDX CONTROLLER INFO:");
    m_pPIDX->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDY CONTROLLER INFO:");
    m_pPIDY->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDZ CONTROLLER INFO:");
    m_pPIDZ->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDYaw CONTROLLER INFO:");
    m_pPIDYaw->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDPitch CONTROLLER INFO:");
    m_pPIDPitch->printInfo();

    ROS_INFO("**************************");
    ROS_INFO("  PIDRoll CONTROLLER INFO:");
    m_pPIDRoll->printInfo();
}


ControllerOutput CCrazyPIDController::computeOutput()
{
    m_sinYaw=sin(m_fullStateEstimation.yaw);
    m_cosYaw=cos(m_fullStateEstimation.yaw);

    m_Xerr=m_positionSetpoint.x-m_fullStateEstimation.x;
    m_Yerr=m_positionSetpoint.y-m_fullStateEstimation.y;

    //convert the error into body frame compensating only for yaw angle
    m_inputPIDX=m_Xerr*m_cosYaw+m_Yerr*m_sinYaw;
    m_inputPIDY=-m_Xerr*m_sinYaw+m_Yerr*m_cosYaw;

    //compute the roll pitch and yaw setpoints, and thrust command

    m_outputPIDZ=m_pPIDZ->computeOutput(m_positionSetpoint.z-m_fullStateEstimation.z);
    m_outputPIDX=m_pPIDX->computeOutput(m_inputPIDX);  //desired pitch angle
    m_outputPIDY=-1*m_pPIDY->computeOutput(m_inputPIDY); //desired roll angle

    m_ControllerOutput.motorCmd1=m_outputPIDZ+m_feedforwardMotorCommands.cmd1;
    m_ControllerOutput.motorCmd2=m_outputPIDZ+m_feedforwardMotorCommands.cmd2;
    m_ControllerOutput.motorCmd3=m_outputPIDZ+m_feedforwardMotorCommands.cmd3;
    m_ControllerOutput.motorCmd4=m_outputPIDZ+m_feedforwardMotorCommands.cmd4;

    if(m_controllerType==ePIDPosition)
    {
        m_ControllerOutput.roll=m_outputPIDY;
        m_ControllerOutput.pitch=-m_outputPIDX;  //minus because of different onboard orientation
        m_ControllerOutput.yaw=m_positionSetpoint.yaw;
        m_ControllerOutput.onboardControllerType=eOnboardAngleController;


    }
    else
    {
        m_ControllerOutput.roll=m_pPIDRoll->computeOutput(m_outputPIDY-m_fullStateEstimation.roll);
        m_ControllerOutput.pitch=m_pPIDPitch->computeOutput(m_outputPIDX-m_fullStateEstimation.pitch);

        double yawErr=m_positionSetpoint.yaw-m_fullStateEstimation.yaw;
        while(yawErr>PI) yawErr-=2*PI;
        while(yawErr<-PI) yawErr+=2*PI;
        m_ControllerOutput.yaw=m_pPIDYaw->computeOutput(yawErr);

        if(m_controllerType==ePIDAngle)
        {
            m_ControllerOutput.pitch*=-1; //different onboard orientation
            m_ControllerOutput.yaw*=-1;
            m_ControllerOutput.onboardControllerType=eOnboardRateController;
        }
        else if(m_controllerType==ePIDFull)
            m_ControllerOutput.onboardControllerType=eOnboardMotorCmdController;
        else
            ROS_ERROR("unknown PID controller type in CCrazyPIDController::computeOutput()");

    }

    return m_ControllerOutput;
}






CCrazyLQRNestedController::CCrazyLQRNestedController(): m_isInit(false)
{
    for(int i=0;i<4;i++)
        for(int j=0;j<9;j++)
        {
            m_KisInit[i][j]=0;
            m_K[i][j]=0;
        }

    m_K[0][1]=-1.7143;
    m_K[0][4]=-1.3371;
    m_K[0][6]=5.1154;

    m_K[1][0]=1.7143;
    m_K[1][3]=1.3371;
    m_K[1][7]=5.1154;

    m_K[2][8]=2.8431;

    m_K[3][2]=0.222;
    m_K[3][5]=0.1236;



    m_a2=2.130295e-11;
    m_a1=1.032633e-6;
    m_a0=5.484560e-4;

    m_saturationThrust=m_a2*12000*12000+m_a1*12000+m_a0;
   // m_saturationThrust=0.01;

    m_isInit=true;
}


void CCrazyLQRNestedController::setControllerParameter(const crazypkg::ControllerParam& parameter)
{
    assert(parameter.crazyControllerType=eLQRNested);

    m_K[parameter.crazyControllerType][parameter.paramType]=parameter.value;
    m_KisInit[parameter.crazyControllerType][parameter.paramType]=1;

    bool isInit=true;

    for(int i=0;i<4;i++)
        for(int j=0;j<9;j++)
            if(m_KisInit[i][j]==0)
                isInit=false;

    m_isInit=isInit;
}


void CCrazyLQRNestedController::setSamplePeriod(double Ts)
{
    CCrazyController::setSamplePeriod(Ts);
}

void CCrazyLQRNestedController::reset()
{

}

bool CCrazyLQRNestedController::fIsInit()
{
    return m_isInit;
}

void CCrazyLQRNestedController::printInfo()
{
    ROS_INFO("*****************************************************************");
    ROS_INFO("        LQR Nested CONTROLLER INFO:");
    CCrazyController::printInfo();
    ROS_INFO_STREAM("Controller init: "<<fIsInit());

    ROS_INFO_STREAM("K= ");
    for(int i=0;i<4;i++)
        ROS_INFO_STREAM(m_K[i][0]<<m_K[i][1]<<m_K[i][2]<<m_K[i][3]<<m_K[i][4]<<m_K[i][5]<<m_K[i][6]<<m_K[i][7]<<m_K[i][8]);
    ROS_INFO_STREAM("feedforward thrust: mot1: "<<m_ffCmd1Thrust<<" mot2: "<<m_ffCmd2Thrust<<
                    " mot3: "<<m_ffCmd3Thrust<<" mot4: "<<m_ffCmd4Thrust);
}


ControllerOutput CCrazyLQRNestedController::computeOutput()
{

    m_sinYaw=sin(m_fullStateEstimation.yaw);
    m_cosYaw=cos(m_fullStateEstimation.yaw);

    m_Xerr=m_fullStateEstimation.x-m_positionSetpoint.x;
    m_Yerr=m_fullStateEstimation.y-m_positionSetpoint.y;

    //convert the error into body frame compensating only for yaw angle
    m_controllerInput[0]=m_Xerr*m_cosYaw+m_Yerr*m_sinYaw;
    m_controllerInput[1]=-m_Xerr*m_sinYaw+m_Yerr*m_cosYaw;

    m_controllerInput[3]=m_fullStateEstimation.x_dot*m_cosYaw+m_fullStateEstimation.y_dot*m_sinYaw;
    m_controllerInput[4]=-m_fullStateEstimation.x_dot*m_sinYaw+m_fullStateEstimation.y_dot*m_cosYaw;

  //  m_controllerInput[0]=m_fullStateEstimation.x-m_positionSetpoint.x;
  //  m_controllerInput[1]=m_fullStateEstimation.y-m_positionSetpoint.y;
    m_controllerInput[2]=m_fullStateEstimation.z-m_positionSetpoint.z;
  //  m_controllerInput[3]=m_fullStateEstimation.x_dot;
  //  m_controllerInput[4]=m_fullStateEstimation.y_dot;
    m_controllerInput[5]=m_fullStateEstimation.z_dot;
    m_controllerInput[6]=m_fullStateEstimation.roll;
    m_controllerInput[7]=m_fullStateEstimation.pitch;

    double yawErr=-m_positionSetpoint.yaw+m_fullStateEstimation.yaw;
    while(yawErr>PI) yawErr-=2*PI;
    while(yawErr<-PI) yawErr+=2*PI;
    m_controllerInput[8]=yawErr;


    double thrust=0,cmd1Thrust,cmd2Thrust,cmd3Thrust,cmd4Thrust;

    m_ControllerOutput.roll=0;
    m_ControllerOutput.pitch=0;
    m_ControllerOutput.yaw=0;

    for(int i=0; i<9; i++)
    {
        m_ControllerOutput.roll-=m_K[0][i]*m_controllerInput[i];
        m_ControllerOutput.pitch-=m_K[1][i]*m_controllerInput[i];
        m_ControllerOutput.yaw-=m_K[2][i]*m_controllerInput[i];
        thrust-=m_K[3][i]*m_controllerInput[i]; //minus since out= -K*input
    }

        if(thrust>m_saturationThrust)
            thrust=m_saturationThrust;
        else if(thrust<-m_saturationThrust)
            thrust=-m_saturationThrust;


    m_ControllerOutput.pitch*=-1; //oposite pitch orientation
    m_ControllerOutput.yaw*=-1;

    cmd1Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd1Thrust))))/(2*m_a2);
    cmd2Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd2Thrust))))/(2*m_a2);
    cmd3Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd3Thrust))))/(2*m_a2);
    cmd4Thrust=(-m_a1+sqrt(m_a1*m_a1-4*m_a2*(m_a0-(thrust+m_ffCmd4Thrust))))/(2*m_a2);

//    ROS_DEBUG_STREAM_THROTTLE(0.2, "thrust: "<<thrust);
//    ROS_DEBUG_STREAM_THROTTLE(0.2, "cmd1 : "<<cmd1Thrust);
//    ROS_DEBUG_STREAM_THROTTLE(0.2, "cmd2 : "<<cmd2Thrust);
//    ROS_DEBUG_STREAM_THROTTLE(0.2, "cmd3 : "<<cmd3Thrust);
//    ROS_DEBUG_STREAM_THROTTLE(0.2, "cmd4 : "<<cmd4Thrust);



//    if(cmd1Thrust>8000)
//        cmd1Thrust=8000;
//    else if(cmd1Thrust<-8000)
//        cmd1Thrust=-8000;

//    if(cmd2Thrust>8000)
//        cmd2Thrust=8000;
//    else if(cmd2Thrust<-8000)
//        cmd2Thrust=-8000;

//    if(cmd3Thrust>8000)
//        cmd3Thrust=8000;
//    else if(cmd3Thrust<-8000)
//        cmd3Thrust=-8000;

//    if(cmd4Thrust>8000)
//        cmd4Thrust=8000;
//    else if(cmd4Thrust<-8000)
//        cmd4Thrust=-8000;

//    m_ControllerOutput.motorCmd1=m_feedforwardMotorCommands.cmd1+cmdThrust;
//    m_ControllerOutput.motorCmd2=m_feedforwardMotorCommands.cmd2+cmdThrust;
//    m_ControllerOutput.motorCmd3=m_feedforwardMotorCommands.cmd3+cmdThrust;
//    m_ControllerOutput.motorCmd4=m_feedforwardMotorCommands.cmd4+cmdThrust;

    m_ControllerOutput.motorCmd1=cmd1Thrust;
    m_ControllerOutput.motorCmd2=cmd2Thrust;
    m_ControllerOutput.motorCmd3=cmd3Thrust;
    m_ControllerOutput.motorCmd4=cmd4Thrust;

    if(m_controllerType==eLQRNestedOffboardRate)
        m_ControllerOutput.onboardControllerType=eOnboardMotorCmdController;
    else if(m_controllerType==eLQRNestedOnboardRate)
        m_ControllerOutput.onboardControllerType=eOnboardRateController;
    else
        ROS_ERROR("unknown LQRNested controller type in CCrazyLQRNestedController::computeOutput()");


    return m_ControllerOutput;
}

void CCrazyLQRNestedController::setFeedforwardCommands(crazypkg::MotorCommands feedforwardCmd)
{
    CCrazyController::setFeedforwardCommands(feedforwardCmd);
    m_ffCmd1Thrust=m_a2*m_feedforwardMotorCommands.cmd1*m_feedforwardMotorCommands.cmd1+
            m_a1*m_feedforwardMotorCommands.cmd1+m_a0;
    m_ffCmd2Thrust=m_a2*m_feedforwardMotorCommands.cmd2*m_feedforwardMotorCommands.cmd2+
            m_a1*m_feedforwardMotorCommands.cmd2+m_a0;
    m_ffCmd3Thrust=m_a2*m_feedforwardMotorCommands.cmd3*m_feedforwardMotorCommands.cmd3+
            m_a1*m_feedforwardMotorCommands.cmd3+m_a0;
    m_ffCmd4Thrust=m_a2*m_feedforwardMotorCommands.cmd4*m_feedforwardMotorCommands.cmd4+
            m_a1*m_feedforwardMotorCommands.cmd4+m_a0;
}
