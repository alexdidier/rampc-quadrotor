#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


#include "rosNodeThread.h"

#define CMD_RECONNECT  0

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

private:
    Ui::MainWindow *ui;

    rosNodeThread* m_rosNodeThread;

    ros::Publisher crazyRadioCommandPublisher;
};

#endif // MAINWINDOW_H
