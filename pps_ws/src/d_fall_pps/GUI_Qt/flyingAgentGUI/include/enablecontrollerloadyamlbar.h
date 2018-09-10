#ifndef ENABLECONTROLLERLOADYAMLBAR_H
#define ENABLECONTROLLERLOADYAMLBAR_H

#include <QWidget>

#ifdef CATKIN_MAKE
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// #include "d_fall_pps/AreaBounds.h"
// #include "d_fall_pps/CrazyflieContext.h"
// #include "d_fall_pps/CMQuery.h"

// using namespace d_fall_pps;
#endif


// COMMANDS FOR THE FLYING STATE/CONTROLLER USED
// The constants that "command" changes in the
// operation state of this agent. These "commands"
// are sent from this GUI node to the "PPSClient"
// node where the command is enacted
#define CMD_USE_SAFE_CONTROLLER      1
#define CMD_USE_DEMO_CONTROLLER      2
#define CMD_USE_STUDENT_CONTROLLER   3
#define CMD_USE_MPC_CONTROLLER       4
#define CMD_USE_REMOTE_CONTROLLER    5
#define CMD_USE_TUNING_CONTROLLER    6


namespace Ui {
class EnableControllerLoadYamlBar;
}

class EnableControllerLoadYamlBar : public QWidget
{
    Q_OBJECT

public:
    explicit EnableControllerLoadYamlBar(QWidget *parent = 0);
    ~EnableControllerLoadYamlBar();

private slots:

    // ENABLE CONTROLLER BUTTONS ON-CLICK CALLBACK
    void on_enable_safe_button_clicked();
    void on_enable_demo_button_clicked();
    void on_enable_student_button_clicked();
    void on_enable_mpc_button_clicked();

    // LOAD YAML BUTTONS ON-CLICK CALLBACK
    void on_load_yaml_safe_button_clicked();
    void on_load_yaml_demo_button_clicked();
    void on_load_yaml_student_button_clicked();
    void on_load_yaml_mpc_button_clicked();

private:
    Ui::EnableControllerLoadYamlBar *ui;

#ifdef CATKIN_MAKE
    // --------------------------------------------------- //
    // PRIVATE VARIABLES FOR ROS

    // PUBLISHERS AND SUBSRIBERS
    // > For Crazyradio commands based on button clicks
    ros::Publisher commandAllPublisher;
#endif
};

#endif // ENABLECONTROLLERLOADYAMLBAR_H
