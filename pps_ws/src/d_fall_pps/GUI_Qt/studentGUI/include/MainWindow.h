#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "rosNodeThread.h"

#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CrazyflieData.h"


// commands for CrazyRadio
#define CMD_RECONNECT  0


// CrazyRadio states:
#define CONNECTED        0
#define CONNECTING       1
#define DISCONNECTED     2

// Commands for PPSClient
#define CMD_USE_SAFE_CONTROLLER   1
#define CMD_USE_CUSTOM_CONTROLLER 2
#define CMD_CRAZYFLY_TAKE_OFF     3
#define CMD_CRAZYFLY_LAND         4
#define CMD_CRAZYFLY_MOTORS_OFF   5

// Flying States
#define STATE_MOTORS_OFF 1
#define STATE_TAKE_OFF   2
#define STATE_FLYING     3
#define STATE_LAND       4

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void updateNewViconData(const ptrToMessage& p_msg);
    void on_RF_Connect_button_clicked();

    void on_take_off_button_clicked();

    void on_land_button_clicked();

    void on_motors_OFF_button_clicked();

private:
    Ui::MainWindow *ui;

    rosNodeThread* m_rosNodeThread;
    int m_radio_status;
    float m_battery_voltage;
    int m_battery_level;

    int m_student_id;
    CrazyflieContext m_context;

    ros::Publisher crazyRadioCommandPublisher;
    ros::Subscriber crazyRadioStatusSubscriber;
    ros::Publisher PPSClientCommandPublisher;
    ros::Subscriber CFBatterySubscriber;
    ros::Subscriber flyingStateSubscriber;

    ros::ServiceClient centralManager;

    // callbacks
    void crazyRadioStatusCallback(const std_msgs::Int32& msg);
    void CFBatteryCallback(const std_msgs::Float32& msg);
    void flyingStateChangedCallback(const std_msgs::Int32& msg);

    float fromVoltageToPercent(float voltage);
    void updateBatteryVoltage(float battery_voltage);
    void setCrazyRadioStatus(int radio_status);
    void loadCrazyflieContext();
    void coordinatesToLocal(CrazyflieData& cf);


    void disableGUI();
    void enableGUI();

    const std::vector<float> m_cutoff_voltages {3.1966,        3.2711,        3.3061,        3.3229,        3.3423,        3.3592,        3.3694,        3.385,        3.4006,        3.4044,        3.4228,        3.4228,        3.4301,        3.4445,        3.4531,        3.4677,        3.4705,        3.4712,        3.4756,        3.483,        3.4944,        3.5008,        3.5008,        3.5084,        3.511,        3.5122,        3.5243,        3.5329,        3.5412,        3.5529,        3.5609,        3.5625,        3.5638,        3.5848,        3.6016,        3.6089,        3.6223,        3.628,        3.6299,        3.6436,        3.6649,        3.6878,        3.6983,        3.7171,        3.7231,        3.7464,        3.7664,        3.7938,        3.8008,        3.816,        3.8313,        3.8482,        3.866,        3.8857,        3.8984,        3.9159,        3.9302,        3.9691,        3.997,        4.14    };
};

#endif // MAINWINDOW_H
