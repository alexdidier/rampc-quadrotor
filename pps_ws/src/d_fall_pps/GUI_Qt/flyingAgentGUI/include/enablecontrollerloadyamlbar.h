#ifndef ENABLECONTROLLERLOADYAMLBAR_H
#define ENABLECONTROLLERLOADYAMLBAR_H

#include <QWidget>
#include <QMutex>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "d_fall_pps/IntWithHeader.h"

// Include the DFALL service types
// #include "d_fall_pps/AreaBounds.h"
// #include "d_fall_pps/CrazyflieContext.h"
// #include "d_fall_pps/CMQuery.h"

// Include the shared definitions
#include "nodes/Constants.h"

// using namespace d_fall_pps;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

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


public slots:
    void setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll);


private slots:

    // ENABLE CONTROLLER BUTTONS ON-CLICK CALLBACK
    void on_enable_safe_button_clicked();
    void on_enable_demo_button_clicked();
    void on_enable_student_button_clicked();
    void on_enable_default_button_clicked();

    // LOAD YAML BUTTONS ON-CLICK CALLBACK
    void on_load_yaml_safe_button_clicked();
    void on_load_yaml_demo_button_clicked();
    void on_load_yaml_student_button_clicked();
    void on_load_yaml_default_button_clicked();

private:
    Ui::EnableControllerLoadYamlBar *ui;


    // --------------------------------------------------- //
    // PRIVATE VARIABLES

    // The type of this node, i.e., agent or a coordinator,
    // specified as a parameter in the "*.launch" file
    int m_type = 0;

    // The ID  of this node
    int m_ID;

    // For coordinating multiple agents
    std::vector<int> m_vector_of_agentIDs_toCoordinate;
    bool m_shouldCoordinateAll = true;
    QMutex m_agentIDs_toCoordinate_mutex;


#ifdef CATKIN_MAKE
    // PUBLISHERS AND SUBSRIBERS
    // > For {take-off,land,motors-off} and controller selection
    ros::Publisher commandPublisher;
#endif

    // --------------------------------------------------- //
    // PRIVATE FUNCTIONS

#ifdef CATKIN_MAKE
    // Fill the head for a message
    void fillIntMessageHeader( d_fall_pps::IntWithHeader & msg );

    // Get the paramters that specify the type and ID
    bool getTypeAndIDParameters();
#endif


};

#endif // ENABLECONTROLLERLOADYAMLBAR_H
