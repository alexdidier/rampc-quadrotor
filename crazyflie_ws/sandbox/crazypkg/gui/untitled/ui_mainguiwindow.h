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
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainGUIWindow
{
public:
    QWidget *centralWidget;
    QFrame *frame;
    QTabWidget *tabWidget;
    QFrame *frame_2;
    QFrame *frame_3;
    QSpinBox *spinBoxNumCrazyflies;
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
        frame->setGeometry(QRect(40, 510, 1171, 451));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        tabWidget = new QTabWidget(frame);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(20, 19, 1131, 401));
        tabWidget->setLayoutDirection(Qt::LeftToRight);
        frame_2 = new QFrame(centralWidget);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(40, 10, 571, 481));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        frame_3 = new QFrame(centralWidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(630, 10, 791, 481));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        spinBoxNumCrazyflies = new QSpinBox(centralWidget);
        spinBoxNumCrazyflies->setObjectName(QStringLiteral("spinBoxNumCrazyflies"));
        spinBoxNumCrazyflies->setGeometry(QRect(1340, 560, 47, 23));
        spinBoxNumCrazyflies->setReadOnly(false);
        spinBoxNumCrazyflies->setAccelerated(false);
        spinBoxNumCrazyflies->setMaximum(6);
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
    } // retranslateUi

};

namespace Ui {
    class MainGUIWindow: public Ui_MainGUIWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINGUIWINDOW_H
