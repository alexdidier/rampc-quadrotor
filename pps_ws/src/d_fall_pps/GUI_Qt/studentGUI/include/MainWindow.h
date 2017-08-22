#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <std_msgs/Int32.h>

#include "rosNodeThread.h"

// commands for CrazyRadio
#define CMD_RECONNECT  0


// CrazyRadio states:
#define CONNECTED        0
#define CONNECTING       1
#define DISCONNECTED     2

// Commands for PPSClient
#define CMD_USE_SAFE_CONTROLLER 1
#define CMD_USE_CUSTOM_CONTROLLER 2
#define CMD_USE_CRAZYFLY_ENABLE 3
#define CMD_USE_CRAZYFLY_DISABLE 4

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

    void on_enable_disable_CF_button_clicked();

private:
    Ui::MainWindow *ui;

    rosNodeThread* m_rosNodeThread;
    int m_radio_status;

    ros::Publisher crazyRadioCommandPublisher;
    ros::Subscriber crazyRadioStatusSubscriber;

    ros::Publisher PPSClientCommadPublisher;

    void crazyRadioStatusCallback(const std_msgs::Int32& msg);
    void setCrazyRadioStatus(int radio_status);

    void disableGUI();
    void enableGUI();
};

#endif // MAINWINDOW_H
