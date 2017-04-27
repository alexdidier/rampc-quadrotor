#-------------------------------------------------
#
# Project created by QtCreator 2017-04-20T11:20:48
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CrazyFlyGUI
TEMPLATE = app

INCLUDEPATH += $$PWD/include
CONFIG += c++11


SOURCES += \
    src/cornergrabber.cpp \
    src/crazyFlyZone.cpp \
    src/crazyFlyZoneTab.cpp \
    src/main.cpp \
    src/mainguiwindow.cpp \
    src/myGraphicsRectItem.cpp \
    src/myGraphicsScene.cpp \
    src/myGraphicsView.cpp \
    src/tablePiece.cpp \
    src/marker.cpp

HEADERS  += \
    include/cornergrabber.h \
    include/crazyFlyZone.h \
    include/crazyFlyZoneTab.h \
    include/mainguiwindow.h \
    include/myGraphicsRectItem.h \
    include/myGraphicsScene.h \
    include/myGraphicsView.h \
    include/tablePiece.h \
    include/globalDefinitions.h \
    include/marker.h

FORMS    += \
    src/mainguiwindow.ui
