#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication applicationGUI(argc, argv);

    MainGUIWindow mainWindow(argc, argv);
    mainWindow.show();
    applicationGUI.exec();
}
