#-------------------------------------------------
#
# Project created by QtCreator 2018-04-26T16:04:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

#greaterThan(QT_MAJOR_VERSION, 4): QT += svg

TARGET = flyingAgentGUI
TEMPLATE = app

INCLUDEPATH += $$PWD/include
CONFIG += c++11

SOURCES += src/main.cpp\
    src/mainwindow.cpp \
    src/topbanner.cpp \
    src/connectstartstopbar.cpp \
    src/enablecontrollerloadyamlbar.cpp \
    src/controllertabs.cpp \
    src/safecontrollertab.cpp \
    src/coordinator.cpp \
    src/coordinatorrow.cpp \
    src/studentcontrollertab.cpp \
    src/defaultcontrollertab.cpp \
    src/pickercontrollertab.cpp \
    src/deepccontrollertab.cpp \
    src/tuningcontrollertab.cpp \


HEADERS  += include/mainwindow.h \
    include/topbanner.h \
    include/connectstartstopbar.h \
    include/enablecontrollerloadyamlbar.h \
    include/controllertabs.h \
    include/safecontrollertab.h \
    include/coordinator.h \
    include/coordinatorrow.h \
    include/studentcontrollertab.h \
    include/defaultcontrollertab.h \
    include/pickercontrollertab.h \
    include/deepccontrollertab.h \
    include/tuningcontrollertab.h \
    include/Constants_for_Qt_compile.h \



FORMS    += forms/mainwindow.ui \
    forms/topbanner.ui \
    forms/connectstartstopbar.ui \
    forms/enablecontrollerloadyamlbar.ui \
    forms/controllertabs.ui \
    forms/safecontrollertab.ui \
    forms/coordinator.ui \
    forms/coordinatorrow.ui \
    forms/studentcontrollertab.ui \
    forms/defaultcontrollertab.ui \
    forms/pickercontrollertab.ui \
    forms/deepccontrollertab.ui \
    forms/tuningcontrollertab.ui \


RESOURCES += \
    flyingagentgui.qrc
