#-------------------------------------------------
#
# Project created by QtCreator 2016-10-30T22:15:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = untitled
TEMPLATE = app


SOURCES += main.cpp\
        mainguiwindow.cpp \
    myGraphicsScene.cpp \
    myGraphicsRectItem.cpp \
    cornergrabber.cpp \
    myGraphicsView.cpp \
    crazyFlyZone.cpp \
    tablePiece.cpp \
    crazyFlyZoneTab.cpp

HEADERS  += mainguiwindow.h \
    myGraphicsScene.h \
    myGraphicsRectItem.h \
    cornergrabber.h \
    myGraphicsView.h \
    crazyFlyZone.h \
    tablePiece.h \
    crazyFlyZoneTab.h

FORMS    += mainguiwindow.ui
