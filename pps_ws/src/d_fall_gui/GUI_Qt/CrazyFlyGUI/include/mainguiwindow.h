#ifndef MAINGUIWINDOW_H
#define MAINGUIWINDOW_H

#define DEBUG_GUI

#include <QMainWindow>
#include <QTimer>
#include <QGridLayout>
#include <QGraphicsRectItem>


#ifndef DEBUG_GUI
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "CrazyFlieTypes.h"
#endif
#include "ui_mainguiwindow.h"
#include "myGraphicsScene.h"

namespace Ui {
class MainGUIWindow;
}


#ifndef DEBUG_GUI
struct setpoint
{
    double x;
    double y;
    double z;
    double yaw;

};

class CSetpointQueue
{
public:
    CSetpointQueue();
    void insert(setpoint newElem);
    setpoint getNext();
    void print();

private:
    struct QueueElem
    {
        QueueElem(setpoint newElem) {elem.x=newElem.x; elem.y=newElem.y; elem.z=newElem.z; elem.yaw=newElem.yaw; next=NULL;}
        setpoint elem;
        QueueElem* next;
    };

    QueueElem* startElem;
    QueueElem* currElem;
    QueueElem* lastElem;
};
#endif

class MainGUIWindow : public QMainWindow
{
    Q_OBJECT

public:
    #ifdef DEBUG_GUI
    explicit MainGUIWindow(/*ros::NodeHandle* ,*/ /*ros::CallbackQueue* callbackQueue, ros::Publisher* publisherMotorCommandsGUI,*/ QWidget *parent = 0);
    #else
    explicit MainGUIWindow(ros::NodeHandle* , /*ros::CallbackQueue* callbackQueue, ros::Publisher* publisherMotorCommandsGUI,*/ QWidget *parent = 0);
    #endif
    ~MainGUIWindow();
    #ifndef DEBUG_GUI
    void init();
    #endif

public slots:
    #ifndef DEBUG_GUI
    void runCallbacks();
    #endif
private slots:
    #ifndef DEBUG_GUI
   // void PIDParamTableChanged(double param);

   // void RateParamTableChanged(double param);

   // void positionSetpointChanged(double param);

   // void sampleTimeChanged(double param);

   // void feedforwardCmdChanged(double cmd);

   // void controllerTypeChanged(bool checked);

   // void trajectoryTypeChanged(bool checked);

   // void on_buttonStop_clicked();

   // void on_buttonPrint_clicked();

   // void on_buttonSetpointChange_clicked();

   // void on_buttonPIDDefaultParams_clicked();

   // void on_buttonSetpointCurrPos_clicked();

   // void on_buttonDefaultFeedforward_clicked();

   // void on_buttonResetMissed_clicked();

   // void on_SetpointHome_clicked();

   // void on_setpointZ200_clicked();

   // void on_slideMotorCmdTest_valueChanged(int value);

   // void on_buttonResetControllers_clicked();

   // void on_buttonSetDefaultTs_clicked();

   // void on_buttonSetDefaultRateParams_clicked();

   // void on_slideRollAngleTest_valueChanged(int value);

   // void on_slidePitchAngleTest_valueChanged(int value);

   // void on_slideYawAngleTest_valueChanged(int value);

   // void on_slideRollRateTest_valueChanged(int value);

   // void on_slidePitchRateTest_valueChanged(int value);

   // void on_slideYawRateTest_valueChanged(int value);

   // void on_buttonStop_2_clicked();
   #endif

    void set_tabs(int n);
    void transitionToMode(int mode);
    void on_removeTable_clicked();

    void on_radioButton_table_mode_toggled(bool checked);

    void on_radioButton_crazyfly_zones_mode_toggled(bool checked);
    void handleTablePiecesNumChanged(int newNum);

    void on_radioButton_lock_mode_toggled(bool checked);

    void on_checkBox_grid_toggled(bool checked);

    void on_checkBox_table_toggled(bool checked);

    void on_checkBox_crazyfly_zones_toggled(bool checked);

    void on_tabWidget_currentChanged(int index);

    void centerViewIndex(int index);

    void on_pushButton_fitAll_clicked();

private:

    Ui::MainGUIWindow *ui;
    myGraphicsScene* scene;
    QGraphicsRectItem* item1;

    void _init();

    #ifndef DEBUG_GUI
    // void refreshScreen();

    // callbacks
    // void callbackControllerOutput(const crazypkg::ControllerOutputPackage& msg);
    // void callbackViconData(const crazypkg::ViconData& msg);
    // void callbackCntViconDataMissed(const std_msgs::Int32& msg);

    void readDefaultParameters();
    // void setDefaultPIDParameters();
    // void setDefaultRateParameters();
    // void setDefaultSampleTime();
    // void setDefaultFeedforwardCmd();

    // void initPIDParamsTable();
    // void initRateParamsTable();
    // void initPositionSetpoint();
    // void initSampleTime();
    // void initControllerType();
    // void initFeedforwardCmd();
    // void initSetpointQueues();
    // void initSetpointType();

    // void publishSetpoint();
    // void updateSetpoint();

    // void publishSampleTime(EControllerType controller);

    ros::CallbackQueue m_CallbackQueue;
    ros::NodeHandle* m_pNodeHandle;

    // publishers
    // ros::Publisher* m_pPublisherMotorCommandsGUI;
    ros::Publisher* m_pPublisherControllerParam;
    ros::Publisher* m_pPublisherPositionSetpoint;
    ros::Publisher* m_pPublisherSampleTime;
    ros::Publisher* m_pPublisherControllerType;
    ros::Publisher* m_pPublisherDoSomething;
    ros::Publisher* m_pPublisherFeedforwardCmd;

    crazypkg::ControllerParam m_controllerParam;
    crazypkg::PositionSetpoint m_positionSetpoint;
    crazypkg::MotorCommands m_DummyCommands;
    crazypkg::SampleTimeParam m_sampleTimeParam;
    std_msgs::Int32 m_controllerType;
    std_msgs::Int32 m_DoSomething;
    crazypkg::MotorCommands m_feedforwardCmd;

    //subscribers
    ros::Subscriber* m_pSubscriberControllerOutput;
    ros::Subscriber* m_pSubscriberViconData;
    ros::Subscriber* m_pSubscriberCntViconDataMissed;

    // crazypkg::MotorCommands m_MotorCommands;
    // crazypkg::ViconData m_ViconData;

    // params
    // PIDParams m_PIDParams [countPIDControllers];

    // default params
    PIDParams m_DefaultPIDParams[countPIDControllers];
    PIDParams m_DefaultRateParams[countRateControllers];
    double m_DefaultSampleTime[countSampleTimeTypes];
    crazypkg::MotorCommands m_DefaultFeedforwardCmd;

    CSetpointQueue m_trajCircle;
    CSetpointQueue m_trajSquare;
    setpoint m_currSetpoint;

    bool m_isStopButtonActive;
    bool m_isCalActive;

    enum ETrajectoryType
    {
        eTrajCustom,
        eTrajCircle,
        eTrajSquare
    };

    ETrajectoryType m_trajectoryType;

    #endif
};



#endif // MAINGUIWINDOW_H
