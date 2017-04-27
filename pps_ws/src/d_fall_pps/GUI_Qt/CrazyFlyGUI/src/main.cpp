#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include <QApplication>

#ifdef CATKIN_MAKE
#endif

int main(int argc, char *argv[])
{
    #ifdef CATKIN_MAKE
    ros::init(argc, argv, "GUI");
    ros::NodeHandle nodeHandle("~");
    #endif
    QApplication applicationGUI(argc, argv);

    MainGUIWindow mainWindow;

    mainWindow.show();
    applicationGUI.exec();

    #ifdef CATKIN_MAKE
    ROS_WARN("GUI application terminated");
    #endif
}
