#ifndef PICKERCONTROLLERTAB_H
#define PICKERCONTROLLERTAB_H

#include <QWidget>
#include <QMutex>
#include <QVector>
#include <QTextStream>

#define DECIMAL_PLACES_POSITION         2
#define DECIMAL_PLACES_ANGLE_DEGREES    1
#define DECIMAL_PLACES_MASS_GRAMS       1

#define DEFAULT_INCREMENT_POSITION_XY      0.01
#define DEFAULT_INCREMENT_POSITION_Z       0.01
#define DEFAULT_INCREMENT_ANGLE_DEGREES    5
#define DEFAULT_INCREMENT_MASS_GRAMS       0.5

// #define PICKER_STATE_UNKNOWN      -1
// #define PICKER_STATE_STANDBY       0
// #define PICKER_STATE_GOTO_START    1
// #define PICKER_STATE_ATTACH        2
// #define PICKER_STATE_LIFT_UP       3
// #define PICKER_STATE_GOTO_END      4
// #define PICKER_STATE_PUT_DOWN      5
// #define PICKER_STATE_SQUAT         6
// #define PICKER_STATE_JUMP          7

// #define PICKER_DEFAULT_X               0
// #define PICKER_DEFAULT_Y               0
// #define PICKER_DEFAULT_Z               0.4
// #define PICKER_DEFAULT_YAW_DEGREES     0
// #define PICKER_DEFAULT_MASS_GRAMS     30

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
//#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
//#include "d_fall_pps/IntWithHeader.h"
#include "d_fall_pps/SetpointWithHeader.h"
//#include "d_fall_pps/CustomButtonWithHeader.h"

// Include the DFALL service types
#include "d_fall_pps/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"
#include "nodes/PickerControllerConstants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace d_fall_pps;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"
#include "include/PickerControllerConstants_for_Qt_compile.h"

#endif



namespace Ui {
class PickerControllerTab;
}

class PickerControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit PickerControllerTab(QWidget *parent = 0);
    ~PickerControllerTab();



public slots:
    void setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll);
    void setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded);
    void poseDataUnavailableSlot();



private slots:
    void on_button_goto_start_clicked();

    void on_button_attach_clicked();

    void on_button_lift_up_clicked();

    void on_button_goto_end_clicked();

    void on_button_put_down_clicked();

    void on_button_squat_clicked();

    void on_button_jump_clicked();

    void on_button_standby_clicked();

    void on_checkbox_goto_start_clicked();

    void on_checkbox_attach_clicked();

    void on_checkbox_lift_up_clicked();

    void on_checkbox_goto_end_clicked();

    void on_checkbox_put_down_clicked();

    void on_checkbox_squat_clicked();

    void on_checkbox_jump_clicked();

    void on_checkbox_standby_clicked();

    void on_button_goto_start_inc_minus_x_clicked();

    void on_button_goto_start_inc_plus_x_clicked();

    void on_button_goto_start_inc_minus_y_clicked();

    void on_button_goto_start_inc_plus_y_clicked();

    void on_button_goto_start_inc_minus_z_clicked();

    void on_button_goto_start_inc_plus_z_clicked();

    void on_button_goto_start_inc_minus_yaw_clicked();

    void on_button_goto_start_inc_plus_yaw_clicked();

    void on_button_attach_inc_minus_z_clicked();

    void on_button_attach_inc_plus_z_clicked();

    void on_button_lift_up_inc_minus_z_clicked();

    void on_button_lift_up_inc_plus_z_clicked();

    void on_button_lift_up_inc_minus_mass_clicked();

    void on_button_lift_up_inc_plus_mass_clicked();

    void on_button_goto_end_inc_minus_x_clicked();

    void on_button_goto_end_inc_plus_x_clicked();

    void on_button_goto_end_inc_minus_y_clicked();

    void on_button_goto_end_inc_plus_y_clicked();

    void on_button_goto_end_inc_minus_yaw_clicked();

    void on_button_goto_end_inc_plus_yaw_clicked();

    void on_button_put_down_inc_minus_z_clicked();

    void on_button_put_down_inc_plus_z_clicked();

    void on_button_squat_inc_minus_z_clicked();

    void on_button_squat_inc_plus_z_clicked();

    void on_button_jump_inc_minus_z_clicked();

    void on_button_jump_inc_plus_z_clicked();

    void on_button_stanby_inc_minus_x_clicked();

    void on_button_stanby_inc_plus_x_clicked();

    void on_button_stanby_inc_minus_y_clicked();

    void on_button_stanby_inc_plus_y_clicked();

    void on_button_stanby_inc_minus_z_clicked();

    void on_button_stanby_inc_plus_z_clicked();

    void on_button_stanby_inc_minus_yaw_clicked();

    void on_button_stanby_inc_plus_yaw_clicked();

    void on_button_stanby_inc_minus_mass_clicked();

    void on_button_stanby_inc_plus_mass_clicked();

    void on_lineEdit_goto_start_x_editingFinished();

    void on_lineEdit_goto_start_y_editingFinished();

    void on_lineEdit_goto_start_z_editingFinished();

    void on_lineEdit_goto_start_yaw_editingFinished();

    void on_lineEdit_attach_z_editingFinished();

    void on_lineEdit_lift_up_z_editingFinished();

    void on_lineEdit_lift_up_mass_editingFinished();

    void on_lineEdit_goto_end_x_editingFinished();

    void on_lineEdit_goto_end_y_editingFinished();

    void on_lineEdit_goto_end_yaw_editingFinished();

    void on_lineEdit_put_down_z_editingFinished();

    void on_lineEdit_squat_z_editingFinished();

    void on_lineEdit_jump_z_editingFinished();

    void on_lineEdit_standby_x_editingFinished();

    void on_lineEdit_standby_y_editingFinished();

    void on_lineEdit_standby_z_editingFinished();

    void on_lineEdit_standby_yaw_editingFinished();

    void on_lineEdit_standby_mass_editingFinished();

    void on_lineEdit_increment_x_editingFinished();

    void on_lineEdit_increment_y_editingFinished();

    void on_lineEdit_increment_z_editingFinished();

    void on_lineEdit_increment_yaw_editingFinished();

    void on_lineEdit_increment_mass_editingFinished();



private:
    Ui::PickerControllerTab *ui;

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

    // THE CURRENT STATE OF THE PICKER
    int current_picker_state = PICKER_STATE_STANDBY;

#ifdef CATKIN_MAKE
    // PUBLISHER
    // > For requesting the setpoint to be changed
    ros::Publisher requestSetpointChangePublisher;

    // SUBSCRIBER
    // > For being notified when the setpoint is changed
    ros::Subscriber setpointChangedSubscriber;
#endif



    // --------------------------------------------------- //
    // PRIVATE FUNCTIONS

#ifdef CATKIN_MAKE
    // For receiving message that the setpoint was changed
    void setpointChangedCallback(const d_fall_pps::SetpointWithHeader& newSetpoint);

    // Fill the header for a message
    void fillSetpointMessageHeader( d_fall_pps::SetpointWithHeader & msg );

    // Get the paramters that specify the type and ID
    bool getTypeAndIDParameters();
#endif

    void publish_setpoint_if_current_state_matches(QVector<int> state_to_match);

    void publish_request_setpoint_change_for_state(int state_to_publish);
};

#endif // PICKERCONTROLLERTAB_H
