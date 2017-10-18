//    Main file of the project Teacher's GUI.
//
//    Copyright (C) 2017  Angel Romero
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
