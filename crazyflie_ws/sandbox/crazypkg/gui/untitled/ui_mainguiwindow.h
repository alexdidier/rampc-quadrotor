/********************************************************************************
** Form generated from reading UI file 'mainguiwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINGUIWINDOW_H
#define UI_MAINGUIWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "myGraphicsView.h"

QT_BEGIN_NAMESPACE

class Ui_MainGUIWindow
{
public:
    QWidget *centralWidget;
    QFrame *frame;
    QTabWidget *tabWidget;
    QFrame *frame_drawing;
    myGraphicsView *graphicsView;
    QPushButton *removeTable;
    QGroupBox *groupBox;
    QRadioButton *radioButton_table_mode;
    QRadioButton *radioButton_crazyfly_zones_mode;
    QRadioButton *radioButton_lock_mode;
    QGroupBox *groupBox_2;
    QCheckBox *checkBox_crazyfly_zones;
    QCheckBox *checkBox_vicon_markers;
    QCheckBox *checkBox_grid;
    QCheckBox *checkBox_table;
    QFrame *frame_3;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainGUIWindow)
    {
        if (MainGUIWindow->objectName().isEmpty())
            MainGUIWindow->setObjectName(QStringLiteral("MainGUIWindow"));
        MainGUIWindow->resize(1479, 1029);
        centralWidget = new QWidget(MainGUIWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        frame = new QFrame(centralWidget);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(40, 510, 1431, 451));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        tabWidget = new QTabWidget(frame);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(20, 19, 1391, 401));
        tabWidget->setLayoutDirection(Qt::LeftToRight);
        tabWidget->setTabsClosable(true);
        frame_drawing = new QFrame(centralWidget);
        frame_drawing->setObjectName(QStringLiteral("frame_drawing"));
        frame_drawing->setGeometry(QRect(20, 18, 951, 471));
        frame_drawing->setFrameShape(QFrame::StyledPanel);
        frame_drawing->setFrameShadow(QFrame::Raised);
        graphicsView = new myGraphicsView(frame_drawing);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(10, 9, 741, 401));
        graphicsView->setMouseTracking(true);
        removeTable = new QPushButton(frame_drawing);
        removeTable->setObjectName(QStringLiteral("removeTable"));
        removeTable->setGeometry(QRect(340, 430, 91, 21));
        groupBox = new QGroupBox(frame_drawing);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(790, 30, 121, 121));
        radioButton_table_mode = new QRadioButton(groupBox);
        radioButton_table_mode->setObjectName(QStringLiteral("radioButton_table_mode"));
        radioButton_table_mode->setGeometry(QRect(10, 20, 61, 20));
        QFont font;
        font.setPointSize(7);
        radioButton_table_mode->setFont(font);
        radioButton_table_mode->setChecked(true);
        radioButton_crazyfly_zones_mode = new QRadioButton(groupBox);
        radioButton_crazyfly_zones_mode->setObjectName(QStringLiteral("radioButton_crazyfly_zones_mode"));
        radioButton_crazyfly_zones_mode->setGeometry(QRect(10, 50, 91, 16));
        radioButton_crazyfly_zones_mode->setFont(font);
        radioButton_crazyfly_zones_mode->setCheckable(true);
        radioButton_lock_mode = new QRadioButton(groupBox);
        radioButton_lock_mode->setObjectName(QStringLiteral("radioButton_lock_mode"));
        radioButton_lock_mode->setGeometry(QRect(10, 80, 91, 16));
        radioButton_lock_mode->setFont(font);
        radioButton_lock_mode->setCheckable(true);
        groupBox_2 = new QGroupBox(frame_drawing);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(790, 230, 120, 141));
        checkBox_crazyfly_zones = new QCheckBox(groupBox_2);
        checkBox_crazyfly_zones->setObjectName(QStringLiteral("checkBox_crazyfly_zones"));
        checkBox_crazyfly_zones->setGeometry(QRect(10, 20, 101, 20));
        checkBox_crazyfly_zones->setFont(font);
        checkBox_crazyfly_zones->setChecked(true);
        checkBox_vicon_markers = new QCheckBox(groupBox_2);
        checkBox_vicon_markers->setObjectName(QStringLiteral("checkBox_vicon_markers"));
        checkBox_vicon_markers->setGeometry(QRect(10, 40, 101, 20));
        checkBox_vicon_markers->setFont(font);
        checkBox_grid = new QCheckBox(groupBox_2);
        checkBox_grid->setObjectName(QStringLiteral("checkBox_grid"));
        checkBox_grid->setGeometry(QRect(10, 60, 101, 20));
        checkBox_grid->setFont(font);
        checkBox_grid->setChecked(true);
        checkBox_table = new QCheckBox(groupBox_2);
        checkBox_table->setObjectName(QStringLiteral("checkBox_table"));
        checkBox_table->setGeometry(QRect(10, 80, 101, 20));
        checkBox_table->setFont(font);
        checkBox_table->setChecked(true);
        frame_3 = new QFrame(centralWidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(990, 20, 481, 469));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        MainGUIWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainGUIWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1479, 19));
        MainGUIWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainGUIWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainGUIWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainGUIWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainGUIWindow->setStatusBar(statusBar);

        retranslateUi(MainGUIWindow);

        tabWidget->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(MainGUIWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainGUIWindow)
    {
        MainGUIWindow->setWindowTitle(QApplication::translate("MainGUIWindow", "MainGUIWindow", 0));
        removeTable->setText(QApplication::translate("MainGUIWindow", "Remove Table", 0));
        groupBox->setTitle(QApplication::translate("MainGUIWindow", "Creation Modes", 0));
        radioButton_table_mode->setText(QApplication::translate("MainGUIWindow", "Tables", 0));
        radioButton_crazyfly_zones_mode->setText(QApplication::translate("MainGUIWindow", "CrazyFly Zones", 0));
        radioButton_lock_mode->setText(QApplication::translate("MainGUIWindow", "Lock", 0));
        groupBox_2->setTitle(QApplication::translate("MainGUIWindow", "What to show", 0));
        checkBox_crazyfly_zones->setText(QApplication::translate("MainGUIWindow", "Show CF Zones", 0));
        checkBox_vicon_markers->setText(QApplication::translate("MainGUIWindow", "Show Markers", 0));
        checkBox_grid->setText(QApplication::translate("MainGUIWindow", "Show Grid", 0));
        checkBox_table->setText(QApplication::translate("MainGUIWindow", "Show Table", 0));
    } // retranslateUi

};

namespace Ui {
    class MainGUIWindow: public Ui_MainGUIWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINGUIWINDOW_H
