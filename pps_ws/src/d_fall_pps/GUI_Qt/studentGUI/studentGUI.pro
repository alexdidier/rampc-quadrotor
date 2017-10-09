#-------------------------------------------------
#
# Project created by QtCreator 2017-08-21T11:01:25
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = studentGUI
TEMPLATE = app

INCLUDEPATH += $$PWD/include
CONFIG += c++11

SOURCES += \
         src/main.cpp \
         src/MainWindow.cpp

HEADERS  += \
         include/MainWindow.h \

FORMS    += \
         src/MainWindow.ui
