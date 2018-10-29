#include "enablecontrollerloadyamlbar.h"
#include "ui_enablecontrollerloadyamlbar.h"

EnableControllerLoadYamlBar::EnableControllerLoadYamlBar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EnableControllerLoadYamlBar)
{
    ui->setupUi(this);


#ifdef CATKIN_MAKE
    //ros::init();
    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    commandAllPublisher = dfall_root_nodeHandle.advertise<std_msgs::Int32>("/my_GUI/commandAllAgents", 1);
#endif

}

EnableControllerLoadYamlBar::~EnableControllerLoadYamlBar()
{
    delete ui;
}




// ENABLE CONTROLLER BUTTONS ON-CLICK CALLBACK

void EnableControllerLoadYamlBar::on_enable_safe_button_clicked()
{
#ifdef CATKIN_MAKE
    std_msgs::Int32 msg;
    msg.data = CMD_USE_SAFE_CONTROLLER;
    this->commandAllPublisher.publish(msg);
    ROS_INFO("[FLYING AGENT GUI] Enable Safe Controller");
#endif
}

void EnableControllerLoadYamlBar::on_enable_demo_button_clicked()
{
#ifdef CATKIN_MAKE
    std_msgs::Int32 msg;
    msg.data = CMD_USE_DEMO_CONTROLLER;
    this->commandAllPublisher.publish(msg);
    ROS_INFO("[FLYING AGENT GUI] Enable Demo Controller");
#endif
}

void EnableControllerLoadYamlBar::on_enable_student_button_clicked()
{
#ifdef CATKIN_MAKE
    std_msgs::Int32 msg;
    msg.data = CMD_USE_STUDENT_CONTROLLER;
    this->commandAllPublisher.publish(msg);
    ROS_INFO("[FLYING AGENT GUI] Enable Student Controller");
#endif
}

void EnableControllerLoadYamlBar::on_enable_default_button_clicked()
{

}


// LOAD YAML BUTTONS ON-CLICK CALLBACK

void EnableControllerLoadYamlBar::on_load_yaml_safe_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_load_yaml_demo_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_load_yaml_student_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_load_yaml_default_button_clicked()
{

}
