#include "../../include/mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include <QApplication>

#ifndef DEBUG_GUI
#include "CrazyFlieInclude.h"
#endif

int main(int argc, char *argv[])
{
    #ifndef DEBUG_GUI
    ros::init(argc, argv, "GUI");
    ros::NodeHandle nodeHandle("~");
    #endif
    QApplication applicationGUI(argc, argv);

    #ifndef DEBUG_GUI
    MainGUIWindow mainWindow(&nodeHandle);
    mainWindow.init();
    #else
    MainGUIWindow mainWindow;
    #endif

    #ifndef DEBUG_GUI
    QTimer *timerExecuteCallbacks = new QTimer(&applicationGUI);
    mainWindow.connect(timerExecuteCallbacks, SIGNAL(timeout()), &mainWindow, SLOT(runCallbacks()));
    timerExecuteCallbacks->start(100);
    #endif

    mainWindow.show();
    applicationGUI.exec();

    #ifndef DEBUG_GUI
    ROS_WARN("GUI application terminated");
    #endif
}
