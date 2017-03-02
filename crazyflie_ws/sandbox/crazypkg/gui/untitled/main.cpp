#include "CrazyFlieInclude.h"
#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include <QApplication>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GUI");
    ros::NodeHandle nodeHandle("~");

    QApplication applicationGUI(argc, argv);

    MainGUIWindow mainWindow(&nodeHandle);
    mainWindow.init();

    QTimer *timerExecuteCallbacs = new QTimer(&applicationGUI);
    mainWindow.connect(timerExecuteCallbacs, SIGNAL(timeout()), &mainWindow, SLOT(runCallbacks()));
    timerExecuteCallbacs->start(100);

    mainWindow.show();
    applicationGUI.exec();


    ROS_WARN("GUI application terminated");
}
