#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


#include "rosNodeThread.h"

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

private:
    Ui::MainWindow *ui;

    rosNodeThread* m_rosNodeThread;
};

#endif // MAINWINDOW_H
