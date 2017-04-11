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
#include <QtWidgets/QPushButton>
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
    QPushButton *drawingModeButton;
    QPushButton *removeTable;
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
        frame->setGeometry(QRect(40, 510, 1171, 451));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        tabWidget = new QTabWidget(frame);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(20, 19, 1131, 401));
        tabWidget->setLayoutDirection(Qt::LeftToRight);
        tabWidget->setTabsClosable(true);
        frame_drawing = new QFrame(centralWidget);
        frame_drawing->setObjectName(QStringLiteral("frame_drawing"));
        frame_drawing->setGeometry(QRect(20, 20, 641, 469));
        frame_drawing->setFrameShape(QFrame::StyledPanel);
        frame_drawing->setFrameShadow(QFrame::Raised);
        graphicsView = new myGraphicsView(frame_drawing);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(10, 9, 571, 401));
        graphicsView->setMouseTracking(true);
        drawingModeButton = new QPushButton(frame_drawing);
        drawingModeButton->setObjectName(QStringLiteral("drawingModeButton"));
        drawingModeButton->setGeometry(QRect(20, 430, 141, 21));
        removeTable = new QPushButton(frame_drawing);
        removeTable->setObjectName(QStringLiteral("removeTable"));
        removeTable->setGeometry(QRect(200, 430, 91, 21));
        frame_3 = new QFrame(centralWidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(669, 21, 541, 469));
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
        drawingModeButton->setText(QApplication::translate("MainGUIWindow", "CreateTableMode", 0));
        removeTable->setText(QApplication::translate("MainGUIWindow", "Remove Table", 0));
    } // retranslateUi

};

namespace Ui {
    class MainGUIWindow: public Ui_MainGUIWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINGUIWINDOW_H
