#include "controllertabs.h"
#include "ui_controllertabs.h"

ControllerTabs::ControllerTabs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerTabs)
{
    ui->setupUi(this);



    // CONNECT THE "MEASURED POST" SIGNAL TO EACH OF
    // THE TABS
    // i.e., connect the "measured pose value changed"
    // signal to the "set measured pose" slots
    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->default_controller_tab_widget , &DefaultControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->student_controller_tab_widget , &StudentControllerTab::setMeasuredPose
        );

}

ControllerTabs::~ControllerTabs()
{
    delete ui;
}



#ifdef CATKIN_MAKE
// > For the controller currently operating, received on
//   "controllerUsedSubscriber"
void poseDataReceivedCallback(const d_fall_pps::CrazyflieData& msg)
{
    // Initialise a Qvector to sending around
    QVector<float> poseDataForSignal;
    // Fill in the data
    poseDataForSignal.push_back(msg.x);
    poseDataForSignal.push_back(msg.y);
    poseDataForSignal.push_back(msg.z);
    poseDataForSignal.push_back(msg.roll);
    poseDataForSignal.push_back(msg.pitch);
    poseDataForSignal.push_back(msg.yaw);
    // Emit the signal
    emit measuredPoseValueChanged(poseDataForSignal);


}
#endif
