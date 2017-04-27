/********************************************************************************
** Form generated from reading UI file 'mainguiwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINGUIWINDOW_H
#define UI_MAINGUIWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainGUIWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget_2;
    QWidget *tab_3;
    QFrame *frame_2;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLCDNumber *LCDMotor1Cmd;
    QLCDNumber *LCDMotor2Cmd;
    QLCDNumber *LCDMotor3Cmd;
    QLCDNumber *LCDMotor4Cmd;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_38;
    QLabel *label_10;
    QLCDNumber *LCDRollCmd;
    QLabel *label_37;
    QLabel *label_41;
    QLCDNumber *LCDPitchCmd;
    QLabel *label_39;
    QLabel *label_40;
    QLCDNumber *LCDYawCmd;
    QLCDNumber *LCDThrustCmd;
    QLabel *labelControllerOutputMode;
    QPushButton *buttonPrint;
    QFrame *frame_5;
    QWidget *gridLayoutWidget_6;
    QGridLayout *gridLayout_6;
    QLabel *label_30;
    QLabel *label_31;
    QLabel *label_32;
    QLabel *label_33;
    QLabel *label_34;
    QDoubleSpinBox *FeedforwardCmd1;
    QDoubleSpinBox *FeedforwardCmd2;
    QDoubleSpinBox *FeedforwardCmd3;
    QDoubleSpinBox *FeedforwardCmd4;
    QLabel *label_35;
    QFrame *frame;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QRadioButton *controllerLQRFull;
    QRadioButton *controllerMotorCmdTest;
    QLabel *label_4;
    QRadioButton *controllerLQRNestedOnboard;
    QLabel *label_3;
    QRadioButton *controllerPIDPosition;
    QDoubleSpinBox *LQRFullTs;
    QLabel *label;
    QSlider *slideMotorCmdTest;
    QLCDNumber *LCDMotorCmdTest;
    QLabel *label_7;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_9;
    QSlider *slideRollRateTest;
    QLabel *label_28;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label_6;
    QSlider *slideYawRateTest;
    QSlider *slidePitchAngleTest;
    QLCDNumber *LCDYawRateTest;
    QRadioButton *controllerRateCmdTest;
    QRadioButton *controllerAngleCmdTest;
    QSlider *slidePitchRateTest;
    QLabel *label_25;
    QRadioButton *controllerPIDFull;
    QLCDNumber *LCDRollAngleTest;
    QLabel *label_27;
    QSpacerItem *horizontalSpacer_3;
    QRadioButton *controllerLQRNestedOffboard;
    QSlider *slideYawAngleTest;
    QLCDNumber *LCDPitchRateTest;
    QLCDNumber *LCDYawAngleTest;
    QLCDNumber *LCDRollRateTest;
    QLabel *label_26;
    QLabel *label_29;
    QRadioButton *controllerPIDAngle;
    QLCDNumber *LCDPitchAngleTest;
    QSlider *slideRollAngleTest;
    QLabel *label_8;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_4;
    QDoubleSpinBox *PIDTs;
    QLabel *label_55;
    QLabel *label_56;
    QDoubleSpinBox *LQRNestedTs;
    QLabel *label_72;
    QPushButton *buttonSetDefaultTs;
    QPushButton *buttonDefaultFeedforward;
    QWidget *gridLayoutWidget_5;
    QGridLayout *gridLayout_7;
    QLabel *label_53;
    QLabel *labelRateMode;
    QLabel *label_54;
    QDoubleSpinBox *RateTs;
    QWidget *tab_4;
    QTabWidget *tabWidget;
    QWidget *tab_6;
    QTableWidget *RateParamTable;
    QPushButton *buttonSetDefaultRateParams;
    QWidget *tab;
    QTableWidget *PIDParamTable;
    QPushButton *buttonPIDDefaultParams;
    QFrame *frame_4;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QLabel *label_23;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;
    QLabel *label_24;
    QDoubleSpinBox *SetpointX;
    QDoubleSpinBox *SetpointY;
    QDoubleSpinBox *SetpointZ;
    QDoubleSpinBox *SetpointYaw;
    QPushButton *buttonSetpointChange;
    QPushButton *buttonSetpointCurrPos;
    QPushButton *buttonStop;
    QPushButton *buttonResetControllers;
    QLCDNumber *LCDMissedMes;
    QLabel *label_36;
    QPushButton *buttonResetMissed;
    QPushButton *SetpointHome;
    QPushButton *setpointZ200;
    QFrame *frame_3;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLabel *label_16;
    QLCDNumber *LCDViconDataYaw;
    QLCDNumber *LCDViconDataX;
    QLabel *label_18;
    QLabel *label_15;
    QLabel *label_17;
    QLabel *label_13;
    QLCDNumber *LCDViconDataZ;
    QLCDNumber *LCDViconDataPitch;
    QLabel *label_14;
    QLCDNumber *LCDViconDataY;
    QLCDNumber *LCDViconDataRoll;
    QLabel *label_19;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QRadioButton *trajCustom;
    QRadioButton *trajCircle;
    QRadioButton *trajSquare;
    QPushButton *buttonStop_2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainGUIWindow)
    {
        if (MainGUIWindow->objectName().isEmpty())
            MainGUIWindow->setObjectName(QStringLiteral("MainGUIWindow"));
        MainGUIWindow->resize(1701, 923);
        centralWidget = new QWidget(MainGUIWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        tabWidget_2 = new QTabWidget(centralWidget);
        tabWidget_2->setObjectName(QStringLiteral("tabWidget_2"));
        tabWidget_2->setGeometry(QRect(50, 210, 691, 611));
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        frame_2 = new QFrame(tab_3);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(400, 210, 201, 271));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        gridLayoutWidget = new QWidget(frame_2);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 10, 181, 251));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        LCDMotor1Cmd = new QLCDNumber(gridLayoutWidget);
        LCDMotor1Cmd->setObjectName(QStringLiteral("LCDMotor1Cmd"));
        QPalette palette;
        QBrush brush(QColor(0, 170, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDMotor1Cmd->setPalette(palette);
        LCDMotor1Cmd->setDigitCount(6);

        gridLayout->addWidget(LCDMotor1Cmd, 0, 1, 1, 1);

        LCDMotor2Cmd = new QLCDNumber(gridLayoutWidget);
        LCDMotor2Cmd->setObjectName(QStringLiteral("LCDMotor2Cmd"));
        QPalette palette1;
        palette1.setBrush(QPalette::Active, QPalette::Light, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDMotor2Cmd->setPalette(palette1);
        LCDMotor2Cmd->setDigitCount(6);

        gridLayout->addWidget(LCDMotor2Cmd, 1, 1, 1, 1);

        LCDMotor3Cmd = new QLCDNumber(gridLayoutWidget);
        LCDMotor3Cmd->setObjectName(QStringLiteral("LCDMotor3Cmd"));
        QPalette palette2;
        palette2.setBrush(QPalette::Active, QPalette::Light, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette2.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDMotor3Cmd->setPalette(palette2);
        LCDMotor3Cmd->setDigitCount(6);

        gridLayout->addWidget(LCDMotor3Cmd, 2, 1, 1, 1);

        LCDMotor4Cmd = new QLCDNumber(gridLayoutWidget);
        LCDMotor4Cmd->setObjectName(QStringLiteral("LCDMotor4Cmd"));
        QPalette palette3;
        palette3.setBrush(QPalette::Active, QPalette::Light, brush);
        palette3.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette3.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDMotor4Cmd->setPalette(palette3);
        LCDMotor4Cmd->setDigitCount(6);

        gridLayout->addWidget(LCDMotor4Cmd, 3, 1, 1, 1);

        label_11 = new QLabel(gridLayoutWidget);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout->addWidget(label_11, 2, 0, 1, 1);

        label_12 = new QLabel(gridLayoutWidget);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout->addWidget(label_12, 3, 0, 1, 1);

        label_38 = new QLabel(gridLayoutWidget);
        label_38->setObjectName(QStringLiteral("label_38"));

        gridLayout->addWidget(label_38, 5, 0, 1, 1);

        label_10 = new QLabel(gridLayoutWidget);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout->addWidget(label_10, 1, 0, 1, 1);

        LCDRollCmd = new QLCDNumber(gridLayoutWidget);
        LCDRollCmd->setObjectName(QStringLiteral("LCDRollCmd"));
        QPalette palette4;
        palette4.setBrush(QPalette::Active, QPalette::Light, brush);
        palette4.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette4.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDRollCmd->setPalette(palette4);
        LCDRollCmd->setDigitCount(6);

        gridLayout->addWidget(LCDRollCmd, 4, 1, 1, 1);

        label_37 = new QLabel(gridLayoutWidget);
        label_37->setObjectName(QStringLiteral("label_37"));

        gridLayout->addWidget(label_37, 4, 0, 1, 1);

        label_41 = new QLabel(gridLayoutWidget);
        label_41->setObjectName(QStringLiteral("label_41"));

        gridLayout->addWidget(label_41, 8, 0, 1, 1);

        LCDPitchCmd = new QLCDNumber(gridLayoutWidget);
        LCDPitchCmd->setObjectName(QStringLiteral("LCDPitchCmd"));
        QPalette palette5;
        palette5.setBrush(QPalette::Active, QPalette::Light, brush);
        palette5.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette5.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDPitchCmd->setPalette(palette5);
        LCDPitchCmd->setDigitCount(6);

        gridLayout->addWidget(LCDPitchCmd, 5, 1, 1, 1);

        label_39 = new QLabel(gridLayoutWidget);
        label_39->setObjectName(QStringLiteral("label_39"));

        gridLayout->addWidget(label_39, 6, 0, 1, 1);

        label_40 = new QLabel(gridLayoutWidget);
        label_40->setObjectName(QStringLiteral("label_40"));

        gridLayout->addWidget(label_40, 7, 0, 1, 1);

        LCDYawCmd = new QLCDNumber(gridLayoutWidget);
        LCDYawCmd->setObjectName(QStringLiteral("LCDYawCmd"));
        QPalette palette6;
        palette6.setBrush(QPalette::Active, QPalette::Light, brush);
        palette6.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette6.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDYawCmd->setPalette(palette6);
        LCDYawCmd->setDigitCount(6);

        gridLayout->addWidget(LCDYawCmd, 6, 1, 1, 1);

        LCDThrustCmd = new QLCDNumber(gridLayoutWidget);
        LCDThrustCmd->setObjectName(QStringLiteral("LCDThrustCmd"));
        QPalette palette7;
        palette7.setBrush(QPalette::Active, QPalette::Light, brush);
        palette7.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette7.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDThrustCmd->setPalette(palette7);
        LCDThrustCmd->setDigitCount(6);

        gridLayout->addWidget(LCDThrustCmd, 7, 1, 1, 1);

        labelControllerOutputMode = new QLabel(gridLayoutWidget);
        labelControllerOutputMode->setObjectName(QStringLiteral("labelControllerOutputMode"));

        gridLayout->addWidget(labelControllerOutputMode, 8, 1, 1, 1);

        buttonPrint = new QPushButton(tab_3);
        buttonPrint->setObjectName(QStringLiteral("buttonPrint"));
        buttonPrint->setGeometry(QRect(400, 490, 141, 22));
        frame_5 = new QFrame(tab_3);
        frame_5->setObjectName(QStringLiteral("frame_5"));
        frame_5->setGeometry(QRect(400, 60, 201, 141));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);
        gridLayoutWidget_6 = new QWidget(frame_5);
        gridLayoutWidget_6->setObjectName(QStringLiteral("gridLayoutWidget_6"));
        gridLayoutWidget_6->setGeometry(QRect(10, 10, 181, 132));
        gridLayout_6 = new QGridLayout(gridLayoutWidget_6);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        label_30 = new QLabel(gridLayoutWidget_6);
        label_30->setObjectName(QStringLiteral("label_30"));

        gridLayout_6->addWidget(label_30, 4, 0, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_6);
        label_31->setObjectName(QStringLiteral("label_31"));

        gridLayout_6->addWidget(label_31, 1, 0, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_6);
        label_32->setObjectName(QStringLiteral("label_32"));

        gridLayout_6->addWidget(label_32, 2, 0, 1, 1);

        label_33 = new QLabel(gridLayoutWidget_6);
        label_33->setObjectName(QStringLiteral("label_33"));

        gridLayout_6->addWidget(label_33, 3, 0, 1, 1);

        label_34 = new QLabel(gridLayoutWidget_6);
        label_34->setObjectName(QStringLiteral("label_34"));

        gridLayout_6->addWidget(label_34, 0, 0, 1, 1);

        FeedforwardCmd1 = new QDoubleSpinBox(gridLayoutWidget_6);
        FeedforwardCmd1->setObjectName(QStringLiteral("FeedforwardCmd1"));
        FeedforwardCmd1->setDecimals(1);
        FeedforwardCmd1->setMinimum(0);
        FeedforwardCmd1->setMaximum(99999);
        FeedforwardCmd1->setSingleStep(20);

        gridLayout_6->addWidget(FeedforwardCmd1, 1, 1, 1, 1);

        FeedforwardCmd2 = new QDoubleSpinBox(gridLayoutWidget_6);
        FeedforwardCmd2->setObjectName(QStringLiteral("FeedforwardCmd2"));
        FeedforwardCmd2->setDecimals(1);
        FeedforwardCmd2->setMinimum(0);
        FeedforwardCmd2->setMaximum(99999);
        FeedforwardCmd2->setSingleStep(20);

        gridLayout_6->addWidget(FeedforwardCmd2, 2, 1, 1, 1);

        FeedforwardCmd3 = new QDoubleSpinBox(gridLayoutWidget_6);
        FeedforwardCmd3->setObjectName(QStringLiteral("FeedforwardCmd3"));
        FeedforwardCmd3->setDecimals(1);
        FeedforwardCmd3->setMinimum(0);
        FeedforwardCmd3->setMaximum(99999);
        FeedforwardCmd3->setSingleStep(20);

        gridLayout_6->addWidget(FeedforwardCmd3, 3, 1, 1, 1);

        FeedforwardCmd4 = new QDoubleSpinBox(gridLayoutWidget_6);
        FeedforwardCmd4->setObjectName(QStringLiteral("FeedforwardCmd4"));
        FeedforwardCmd4->setDecimals(1);
        FeedforwardCmd4->setMinimum(0);
        FeedforwardCmd4->setMaximum(99999);

        gridLayout_6->addWidget(FeedforwardCmd4, 4, 1, 1, 1);

        label_35 = new QLabel(gridLayoutWidget_6);
        label_35->setObjectName(QStringLiteral("label_35"));

        gridLayout_6->addWidget(label_35, 0, 1, 1, 1);

        frame = new QFrame(tab_3);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(10, 10, 381, 561));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        frame->setLineWidth(2);
        gridLayoutWidget_2 = new QWidget(frame);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(10, 10, 361, 547));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        controllerLQRFull = new QRadioButton(gridLayoutWidget_2);
        controllerLQRFull->setObjectName(QStringLiteral("controllerLQRFull"));

        gridLayout_2->addWidget(controllerLQRFull, 7, 0, 1, 1);

        controllerMotorCmdTest = new QRadioButton(gridLayoutWidget_2);
        controllerMotorCmdTest->setObjectName(QStringLiteral("controllerMotorCmdTest"));

        gridLayout_2->addWidget(controllerMotorCmdTest, 11, 0, 1, 1);

        label_4 = new QLabel(gridLayoutWidget_2);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_2->addWidget(label_4, 1, 2, 1, 1);

        controllerLQRNestedOnboard = new QRadioButton(gridLayoutWidget_2);
        controllerLQRNestedOnboard->setObjectName(QStringLiteral("controllerLQRNestedOnboard"));

        gridLayout_2->addWidget(controllerLQRNestedOnboard, 8, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget_2);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_2->addWidget(label_3, 1, 0, 1, 1);

        controllerPIDPosition = new QRadioButton(gridLayoutWidget_2);
        controllerPIDPosition->setObjectName(QStringLiteral("controllerPIDPosition"));

        gridLayout_2->addWidget(controllerPIDPosition, 3, 0, 1, 1);

        LQRFullTs = new QDoubleSpinBox(gridLayoutWidget_2);
        LQRFullTs->setObjectName(QStringLiteral("LQRFullTs"));
        LQRFullTs->setDecimals(1);
        LQRFullTs->setMaximum(500);

        gridLayout_2->addWidget(LQRFullTs, 7, 2, 1, 1);

        label = new QLabel(gridLayoutWidget_2);
        label->setObjectName(QStringLiteral("label"));

        gridLayout_2->addWidget(label, 10, 2, 1, 1);

        slideMotorCmdTest = new QSlider(gridLayoutWidget_2);
        slideMotorCmdTest->setObjectName(QStringLiteral("slideMotorCmdTest"));
        slideMotorCmdTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slideMotorCmdTest, 11, 2, 1, 1);

        LCDMotorCmdTest = new QLCDNumber(gridLayoutWidget_2);
        LCDMotorCmdTest->setObjectName(QStringLiteral("LCDMotorCmdTest"));
        QPalette palette8;
        palette8.setBrush(QPalette::Active, QPalette::Light, brush);
        palette8.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette8.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDMotorCmdTest->setPalette(palette8);
        LCDMotorCmdTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDMotorCmdTest, 11, 3, 1, 1);

        label_7 = new QLabel(gridLayoutWidget_2);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_2->addWidget(label_7, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 10, 0, 1, 1);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        label_9 = new QLabel(gridLayoutWidget_2);
        label_9->setObjectName(QStringLiteral("label_9"));

        verticalLayout_4->addWidget(label_9);


        gridLayout_2->addLayout(verticalLayout_4, 0, 2, 1, 1);

        slideRollRateTest = new QSlider(gridLayoutWidget_2);
        slideRollRateTest->setObjectName(QStringLiteral("slideRollRateTest"));
        slideRollRateTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slideRollRateTest, 17, 2, 1, 1);

        label_28 = new QLabel(gridLayoutWidget_2);
        label_28->setObjectName(QStringLiteral("label_28"));

        gridLayout_2->addWidget(label_28, 18, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_2, 12, 0, 1, 1);

        label_6 = new QLabel(gridLayoutWidget_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_2->addWidget(label_6, 2, 0, 1, 1);

        slideYawRateTest = new QSlider(gridLayoutWidget_2);
        slideYawRateTest->setObjectName(QStringLiteral("slideYawRateTest"));
        slideYawRateTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slideYawRateTest, 19, 2, 1, 1);

        slidePitchAngleTest = new QSlider(gridLayoutWidget_2);
        slidePitchAngleTest->setObjectName(QStringLiteral("slidePitchAngleTest"));
        slidePitchAngleTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slidePitchAngleTest, 14, 2, 1, 1);

        LCDYawRateTest = new QLCDNumber(gridLayoutWidget_2);
        LCDYawRateTest->setObjectName(QStringLiteral("LCDYawRateTest"));
        QPalette palette9;
        palette9.setBrush(QPalette::Active, QPalette::Light, brush);
        palette9.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette9.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDYawRateTest->setPalette(palette9);
        LCDYawRateTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDYawRateTest, 19, 3, 1, 1);

        controllerRateCmdTest = new QRadioButton(gridLayoutWidget_2);
        controllerRateCmdTest->setObjectName(QStringLiteral("controllerRateCmdTest"));

        gridLayout_2->addWidget(controllerRateCmdTest, 17, 0, 1, 1);

        controllerAngleCmdTest = new QRadioButton(gridLayoutWidget_2);
        controllerAngleCmdTest->setObjectName(QStringLiteral("controllerAngleCmdTest"));

        gridLayout_2->addWidget(controllerAngleCmdTest, 13, 0, 1, 1);

        slidePitchRateTest = new QSlider(gridLayoutWidget_2);
        slidePitchRateTest->setObjectName(QStringLiteral("slidePitchRateTest"));
        slidePitchRateTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slidePitchRateTest, 18, 2, 1, 1);

        label_25 = new QLabel(gridLayoutWidget_2);
        label_25->setObjectName(QStringLiteral("label_25"));

        gridLayout_2->addWidget(label_25, 14, 1, 1, 1);

        controllerPIDFull = new QRadioButton(gridLayoutWidget_2);
        controllerPIDFull->setObjectName(QStringLiteral("controllerPIDFull"));

        gridLayout_2->addWidget(controllerPIDFull, 5, 0, 1, 1);

        LCDRollAngleTest = new QLCDNumber(gridLayoutWidget_2);
        LCDRollAngleTest->setObjectName(QStringLiteral("LCDRollAngleTest"));
        QPalette palette10;
        palette10.setBrush(QPalette::Active, QPalette::Light, brush);
        palette10.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette10.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDRollAngleTest->setPalette(palette10);
        LCDRollAngleTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDRollAngleTest, 13, 3, 1, 1);

        label_27 = new QLabel(gridLayoutWidget_2);
        label_27->setObjectName(QStringLiteral("label_27"));

        gridLayout_2->addWidget(label_27, 17, 1, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_3, 16, 0, 1, 1);

        controllerLQRNestedOffboard = new QRadioButton(gridLayoutWidget_2);
        controllerLQRNestedOffboard->setObjectName(QStringLiteral("controllerLQRNestedOffboard"));

        gridLayout_2->addWidget(controllerLQRNestedOffboard, 9, 0, 1, 1);

        slideYawAngleTest = new QSlider(gridLayoutWidget_2);
        slideYawAngleTest->setObjectName(QStringLiteral("slideYawAngleTest"));
        slideYawAngleTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slideYawAngleTest, 15, 2, 1, 1);

        LCDPitchRateTest = new QLCDNumber(gridLayoutWidget_2);
        LCDPitchRateTest->setObjectName(QStringLiteral("LCDPitchRateTest"));
        QPalette palette11;
        palette11.setBrush(QPalette::Active, QPalette::Light, brush);
        palette11.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette11.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDPitchRateTest->setPalette(palette11);
        LCDPitchRateTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDPitchRateTest, 18, 3, 1, 1);

        LCDYawAngleTest = new QLCDNumber(gridLayoutWidget_2);
        LCDYawAngleTest->setObjectName(QStringLiteral("LCDYawAngleTest"));
        QPalette palette12;
        palette12.setBrush(QPalette::Active, QPalette::Light, brush);
        palette12.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette12.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDYawAngleTest->setPalette(palette12);
        LCDYawAngleTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDYawAngleTest, 15, 3, 1, 1);

        LCDRollRateTest = new QLCDNumber(gridLayoutWidget_2);
        LCDRollRateTest->setObjectName(QStringLiteral("LCDRollRateTest"));
        QPalette palette13;
        palette13.setBrush(QPalette::Active, QPalette::Light, brush);
        palette13.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette13.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDRollRateTest->setPalette(palette13);
        LCDRollRateTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDRollRateTest, 17, 3, 1, 1);

        label_26 = new QLabel(gridLayoutWidget_2);
        label_26->setObjectName(QStringLiteral("label_26"));

        gridLayout_2->addWidget(label_26, 15, 1, 1, 1);

        label_29 = new QLabel(gridLayoutWidget_2);
        label_29->setObjectName(QStringLiteral("label_29"));

        gridLayout_2->addWidget(label_29, 19, 1, 1, 1);

        controllerPIDAngle = new QRadioButton(gridLayoutWidget_2);
        controllerPIDAngle->setObjectName(QStringLiteral("controllerPIDAngle"));

        gridLayout_2->addWidget(controllerPIDAngle, 4, 0, 1, 1);

        LCDPitchAngleTest = new QLCDNumber(gridLayoutWidget_2);
        LCDPitchAngleTest->setObjectName(QStringLiteral("LCDPitchAngleTest"));
        QPalette palette14;
        palette14.setBrush(QPalette::Active, QPalette::Light, brush);
        palette14.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette14.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDPitchAngleTest->setPalette(palette14);
        LCDPitchAngleTest->setDigitCount(6);

        gridLayout_2->addWidget(LCDPitchAngleTest, 14, 3, 1, 1);

        slideRollAngleTest = new QSlider(gridLayoutWidget_2);
        slideRollAngleTest->setObjectName(QStringLiteral("slideRollAngleTest"));
        slideRollAngleTest->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(slideRollAngleTest, 13, 2, 1, 1);

        label_8 = new QLabel(gridLayoutWidget_2);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_2->addWidget(label_8, 13, 1, 1, 1);

        label_5 = new QLabel(gridLayoutWidget_2);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout_2->addWidget(label_5, 11, 1, 1, 1);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_4, 6, 0, 1, 1);

        PIDTs = new QDoubleSpinBox(gridLayoutWidget_2);
        PIDTs->setObjectName(QStringLiteral("PIDTs"));
        PIDTs->setDecimals(1);
        PIDTs->setMaximum(500);

        gridLayout_2->addWidget(PIDTs, 4, 2, 1, 1);

        label_55 = new QLabel(gridLayoutWidget_2);
        label_55->setObjectName(QStringLiteral("label_55"));

        gridLayout_2->addWidget(label_55, 4, 3, 1, 1);

        label_56 = new QLabel(gridLayoutWidget_2);
        label_56->setObjectName(QStringLiteral("label_56"));

        gridLayout_2->addWidget(label_56, 7, 3, 1, 1);

        LQRNestedTs = new QDoubleSpinBox(gridLayoutWidget_2);
        LQRNestedTs->setObjectName(QStringLiteral("LQRNestedTs"));
        LQRNestedTs->setDecimals(1);
        LQRNestedTs->setMaximum(500);

        gridLayout_2->addWidget(LQRNestedTs, 8, 2, 1, 1);

        label_72 = new QLabel(gridLayoutWidget_2);
        label_72->setObjectName(QStringLiteral("label_72"));

        gridLayout_2->addWidget(label_72, 8, 3, 1, 1);

        buttonSetDefaultTs = new QPushButton(tab_3);
        buttonSetDefaultTs->setObjectName(QStringLiteral("buttonSetDefaultTs"));
        buttonSetDefaultTs->setGeometry(QRect(400, 520, 141, 22));
        buttonDefaultFeedforward = new QPushButton(tab_3);
        buttonDefaultFeedforward->setObjectName(QStringLiteral("buttonDefaultFeedforward"));
        buttonDefaultFeedforward->setGeometry(QRect(400, 550, 141, 22));
        gridLayoutWidget_5 = new QWidget(tab_3);
        gridLayoutWidget_5->setObjectName(QStringLiteral("gridLayoutWidget_5"));
        gridLayoutWidget_5->setGeometry(QRect(410, 10, 181, 45));
        gridLayout_7 = new QGridLayout(gridLayoutWidget_5);
        gridLayout_7->setSpacing(6);
        gridLayout_7->setContentsMargins(11, 11, 11, 11);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        gridLayout_7->setContentsMargins(0, 0, 0, 0);
        label_53 = new QLabel(gridLayoutWidget_5);
        label_53->setObjectName(QStringLiteral("label_53"));

        gridLayout_7->addWidget(label_53, 0, 0, 1, 1);

        labelRateMode = new QLabel(gridLayoutWidget_5);
        labelRateMode->setObjectName(QStringLiteral("labelRateMode"));
        QPalette palette15;
        palette15.setBrush(QPalette::Active, QPalette::Text, brush);
        palette15.setBrush(QPalette::Inactive, QPalette::Text, brush);
        QBrush brush1(QColor(190, 190, 190, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette15.setBrush(QPalette::Disabled, QPalette::Text, brush1);
        labelRateMode->setPalette(palette15);

        gridLayout_7->addWidget(labelRateMode, 0, 1, 1, 1);

        label_54 = new QLabel(gridLayoutWidget_5);
        label_54->setObjectName(QStringLiteral("label_54"));

        gridLayout_7->addWidget(label_54, 1, 0, 1, 1);

        RateTs = new QDoubleSpinBox(gridLayoutWidget_5);
        RateTs->setObjectName(QStringLiteral("RateTs"));
        RateTs->setDecimals(1);
        RateTs->setMaximum(20);

        gridLayout_7->addWidget(RateTs, 1, 1, 1, 1);

        tabWidget_2->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        tabWidget = new QTabWidget(tab_4);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(10, 10, 681, 351));
        tab_6 = new QWidget();
        tab_6->setObjectName(QStringLiteral("tab_6"));
        RateParamTable = new QTableWidget(tab_6);
        if (RateParamTable->columnCount() < 6)
            RateParamTable->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        RateParamTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        RateParamTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        RateParamTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        RateParamTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        RateParamTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        RateParamTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        if (RateParamTable->rowCount() < 3)
            RateParamTable->setRowCount(3);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        RateParamTable->setVerticalHeaderItem(0, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        RateParamTable->setVerticalHeaderItem(1, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        RateParamTable->setVerticalHeaderItem(2, __qtablewidgetitem8);
        RateParamTable->setObjectName(QStringLiteral("RateParamTable"));
        RateParamTable->setGeometry(QRect(10, 10, 651, 121));
        buttonSetDefaultRateParams = new QPushButton(tab_6);
        buttonSetDefaultRateParams->setObjectName(QStringLiteral("buttonSetDefaultRateParams"));
        buttonSetDefaultRateParams->setGeometry(QRect(10, 150, 80, 22));
        tabWidget->addTab(tab_6, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        PIDParamTable = new QTableWidget(tab);
        if (PIDParamTable->columnCount() < 6)
            PIDParamTable->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        PIDParamTable->setHorizontalHeaderItem(0, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        PIDParamTable->setHorizontalHeaderItem(1, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        PIDParamTable->setHorizontalHeaderItem(2, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        PIDParamTable->setHorizontalHeaderItem(3, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        PIDParamTable->setHorizontalHeaderItem(4, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        PIDParamTable->setHorizontalHeaderItem(5, __qtablewidgetitem14);
        if (PIDParamTable->rowCount() < 6)
            PIDParamTable->setRowCount(6);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        PIDParamTable->setVerticalHeaderItem(0, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        PIDParamTable->setVerticalHeaderItem(1, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        PIDParamTable->setVerticalHeaderItem(2, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        PIDParamTable->setVerticalHeaderItem(3, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        PIDParamTable->setVerticalHeaderItem(4, __qtablewidgetitem19);
        QTableWidgetItem *__qtablewidgetitem20 = new QTableWidgetItem();
        PIDParamTable->setVerticalHeaderItem(5, __qtablewidgetitem20);
        QTableWidgetItem *__qtablewidgetitem21 = new QTableWidgetItem();
        PIDParamTable->setItem(0, 0, __qtablewidgetitem21);
        PIDParamTable->setObjectName(QStringLiteral("PIDParamTable"));
        PIDParamTable->setGeometry(QRect(10, 10, 651, 211));
        buttonPIDDefaultParams = new QPushButton(tab);
        buttonPIDDefaultParams->setObjectName(QStringLiteral("buttonPIDDefaultParams"));
        buttonPIDDefaultParams->setGeometry(QRect(10, 230, 80, 22));
        tabWidget->addTab(tab, QString());
        tabWidget_2->addTab(tab_4, QString());
        frame_4 = new QFrame(centralWidget);
        frame_4->setObjectName(QStringLiteral("frame_4"));
        frame_4->setGeometry(QRect(780, 90, 291, 151));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);
        gridLayoutWidget_4 = new QWidget(frame_4);
        gridLayoutWidget_4->setObjectName(QStringLiteral("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(10, 10, 181, 132));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_23 = new QLabel(gridLayoutWidget_4);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_4->addWidget(label_23, 4, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));

        gridLayout_4->addWidget(label_20, 1, 0, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_4->addWidget(label_21, 2, 0, 1, 1);

        label_22 = new QLabel(gridLayoutWidget_4);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_4->addWidget(label_22, 3, 0, 1, 1);

        label_24 = new QLabel(gridLayoutWidget_4);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_4->addWidget(label_24, 0, 0, 1, 1);

        SetpointX = new QDoubleSpinBox(gridLayoutWidget_4);
        SetpointX->setObjectName(QStringLiteral("SetpointX"));
        SetpointX->setDecimals(1);
        SetpointX->setMinimum(-10000);
        SetpointX->setMaximum(10000);
        SetpointX->setSingleStep(20);

        gridLayout_4->addWidget(SetpointX, 1, 1, 1, 1);

        SetpointY = new QDoubleSpinBox(gridLayoutWidget_4);
        SetpointY->setObjectName(QStringLiteral("SetpointY"));
        SetpointY->setDecimals(1);
        SetpointY->setMinimum(-10000);
        SetpointY->setMaximum(10000);
        SetpointY->setSingleStep(20);

        gridLayout_4->addWidget(SetpointY, 2, 1, 1, 1);

        SetpointZ = new QDoubleSpinBox(gridLayoutWidget_4);
        SetpointZ->setObjectName(QStringLiteral("SetpointZ"));
        SetpointZ->setDecimals(1);
        SetpointZ->setMinimum(-10000);
        SetpointZ->setMaximum(10000);
        SetpointZ->setSingleStep(20);

        gridLayout_4->addWidget(SetpointZ, 3, 1, 1, 1);

        SetpointYaw = new QDoubleSpinBox(gridLayoutWidget_4);
        SetpointYaw->setObjectName(QStringLiteral("SetpointYaw"));
        SetpointYaw->setDecimals(3);
        SetpointYaw->setMinimum(-180);
        SetpointYaw->setMaximum(180);

        gridLayout_4->addWidget(SetpointYaw, 4, 1, 1, 1);

        buttonSetpointChange = new QPushButton(frame_4);
        buttonSetpointChange->setObjectName(QStringLiteral("buttonSetpointChange"));
        buttonSetpointChange->setGeometry(QRect(200, 10, 80, 51));
        buttonSetpointCurrPos = new QPushButton(frame_4);
        buttonSetpointCurrPos->setObjectName(QStringLiteral("buttonSetpointCurrPos"));
        buttonSetpointCurrPos->setGeometry(QRect(200, 80, 80, 51));
        buttonStop = new QPushButton(centralWidget);
        buttonStop->setObjectName(QStringLiteral("buttonStop"));
        buttonStop->setGeometry(QRect(300, 20, 141, 111));
        buttonResetControllers = new QPushButton(centralWidget);
        buttonResetControllers->setObjectName(QStringLiteral("buttonResetControllers"));
        buttonResetControllers->setGeometry(QRect(300, 140, 141, 31));
        LCDMissedMes = new QLCDNumber(centralWidget);
        LCDMissedMes->setObjectName(QStringLiteral("LCDMissedMes"));
        LCDMissedMes->setGeometry(QRect(1090, 330, 111, 24));
        QPalette palette16;
        palette16.setBrush(QPalette::Active, QPalette::Light, brush);
        palette16.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette16.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDMissedMes->setPalette(palette16);
        LCDMissedMes->setDigitCount(9);
        LCDMissedMes->setMode(QLCDNumber::Dec);
        label_36 = new QLabel(centralWidget);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setGeometry(QRect(940, 330, 141, 20));
        buttonResetMissed = new QPushButton(centralWidget);
        buttonResetMissed->setObjectName(QStringLiteral("buttonResetMissed"));
        buttonResetMissed->setGeometry(QRect(1090, 360, 101, 22));
        SetpointHome = new QPushButton(centralWidget);
        SetpointHome->setObjectName(QStringLiteral("SetpointHome"));
        SetpointHome->setGeometry(QRect(1080, 100, 80, 22));
        setpointZ200 = new QPushButton(centralWidget);
        setpointZ200->setObjectName(QStringLiteral("setpointZ200"));
        setpointZ200->setGeometry(QRect(1080, 130, 80, 22));
        frame_3 = new QFrame(centralWidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(760, 390, 181, 231));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        gridLayoutWidget_3 = new QWidget(frame_3);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 10, 161, 211));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_16 = new QLabel(gridLayoutWidget_3);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_3->addWidget(label_16, 4, 0, 1, 1);

        LCDViconDataYaw = new QLCDNumber(gridLayoutWidget_3);
        LCDViconDataYaw->setObjectName(QStringLiteral("LCDViconDataYaw"));
        QPalette palette17;
        palette17.setBrush(QPalette::Active, QPalette::Light, brush);
        palette17.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette17.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDViconDataYaw->setPalette(palette17);
        LCDViconDataYaw->setDigitCount(6);

        gridLayout_3->addWidget(LCDViconDataYaw, 4, 1, 1, 1);

        LCDViconDataX = new QLCDNumber(gridLayoutWidget_3);
        LCDViconDataX->setObjectName(QStringLiteral("LCDViconDataX"));
        QPalette palette18;
        palette18.setBrush(QPalette::Active, QPalette::Light, brush);
        palette18.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette18.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDViconDataX->setPalette(palette18);
        LCDViconDataX->setDigitCount(6);

        gridLayout_3->addWidget(LCDViconDataX, 1, 1, 1, 1);

        label_18 = new QLabel(gridLayoutWidget_3);
        label_18->setObjectName(QStringLiteral("label_18"));

        gridLayout_3->addWidget(label_18, 5, 0, 1, 1);

        label_15 = new QLabel(gridLayoutWidget_3);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_3->addWidget(label_15, 3, 0, 1, 1);

        label_17 = new QLabel(gridLayoutWidget_3);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_3->addWidget(label_17, 0, 0, 1, 1);

        label_13 = new QLabel(gridLayoutWidget_3);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_3->addWidget(label_13, 1, 0, 1, 1);

        LCDViconDataZ = new QLCDNumber(gridLayoutWidget_3);
        LCDViconDataZ->setObjectName(QStringLiteral("LCDViconDataZ"));
        QPalette palette19;
        palette19.setBrush(QPalette::Active, QPalette::Light, brush);
        palette19.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette19.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDViconDataZ->setPalette(palette19);
        LCDViconDataZ->setDigitCount(6);

        gridLayout_3->addWidget(LCDViconDataZ, 3, 1, 1, 1);

        LCDViconDataPitch = new QLCDNumber(gridLayoutWidget_3);
        LCDViconDataPitch->setObjectName(QStringLiteral("LCDViconDataPitch"));
        QPalette palette20;
        palette20.setBrush(QPalette::Active, QPalette::Light, brush);
        palette20.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette20.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDViconDataPitch->setPalette(palette20);
        LCDViconDataPitch->setDigitCount(6);

        gridLayout_3->addWidget(LCDViconDataPitch, 5, 1, 1, 1);

        label_14 = new QLabel(gridLayoutWidget_3);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout_3->addWidget(label_14, 2, 0, 1, 1);

        LCDViconDataY = new QLCDNumber(gridLayoutWidget_3);
        LCDViconDataY->setObjectName(QStringLiteral("LCDViconDataY"));
        QPalette palette21;
        palette21.setBrush(QPalette::Active, QPalette::Light, brush);
        palette21.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette21.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDViconDataY->setPalette(palette21);
        LCDViconDataY->setDigitCount(6);

        gridLayout_3->addWidget(LCDViconDataY, 2, 1, 1, 1);

        LCDViconDataRoll = new QLCDNumber(gridLayoutWidget_3);
        LCDViconDataRoll->setObjectName(QStringLiteral("LCDViconDataRoll"));
        QPalette palette22;
        palette22.setBrush(QPalette::Active, QPalette::Light, brush);
        palette22.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette22.setBrush(QPalette::Disabled, QPalette::Light, brush);
        LCDViconDataRoll->setPalette(palette22);
        LCDViconDataRoll->setDigitCount(6);

        gridLayout_3->addWidget(LCDViconDataRoll, 6, 1, 1, 1);

        label_19 = new QLabel(gridLayoutWidget_3);
        label_19->setObjectName(QStringLiteral("label_19"));

        gridLayout_3->addWidget(label_19, 6, 0, 1, 1);

        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(1100, 180, 160, 80));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        trajCustom = new QRadioButton(verticalLayoutWidget);
        trajCustom->setObjectName(QStringLiteral("trajCustom"));

        verticalLayout->addWidget(trajCustom);

        trajCircle = new QRadioButton(verticalLayoutWidget);
        trajCircle->setObjectName(QStringLiteral("trajCircle"));

        verticalLayout->addWidget(trajCircle);

        trajSquare = new QRadioButton(verticalLayoutWidget);
        trajSquare->setObjectName(QStringLiteral("trajSquare"));

        verticalLayout->addWidget(trajSquare);

        buttonStop_2 = new QPushButton(centralWidget);
        buttonStop_2->setObjectName(QStringLiteral("buttonStop_2"));
        buttonStop_2->setGeometry(QRect(640, 50, 71, 21));
        MainGUIWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainGUIWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1701, 19));
        MainGUIWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainGUIWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainGUIWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainGUIWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainGUIWindow->setStatusBar(statusBar);

        retranslateUi(MainGUIWindow);

        tabWidget_2->setCurrentIndex(0);
        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainGUIWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainGUIWindow)
    {
        MainGUIWindow->setWindowTitle(QApplication::translate("MainGUIWindow", "MainGUIWindow", 0));
        label_2->setText(QApplication::translate("MainGUIWindow", "Motor1", 0));
        label_11->setText(QApplication::translate("MainGUIWindow", "Motor 3", 0));
        label_12->setText(QApplication::translate("MainGUIWindow", "Motor 4", 0));
        label_38->setText(QApplication::translate("MainGUIWindow", "Pitch", 0));
        label_10->setText(QApplication::translate("MainGUIWindow", "Motor 2", 0));
        label_37->setText(QApplication::translate("MainGUIWindow", "Roll", 0));
        label_41->setText(QApplication::translate("MainGUIWindow", "Mode", 0));
        label_39->setText(QApplication::translate("MainGUIWindow", "Yaw", 0));
        label_40->setText(QApplication::translate("MainGUIWindow", "Thrust", 0));
        labelControllerOutputMode->setText(QApplication::translate("MainGUIWindow", "Mode", 0));
        buttonPrint->setText(QApplication::translate("MainGUIWindow", "Print Controllers Info", 0));
        label_30->setText(QApplication::translate("MainGUIWindow", "Motor4", 0));
        label_31->setText(QApplication::translate("MainGUIWindow", "Motor1", 0));
        label_32->setText(QApplication::translate("MainGUIWindow", "Motor2", 0));
        label_33->setText(QApplication::translate("MainGUIWindow", "Motor3", 0));
        label_34->setText(QApplication::translate("MainGUIWindow", "Feedforward", 0));
        label_35->setText(QApplication::translate("MainGUIWindow", "Commands", 0));
        controllerLQRFull->setText(QApplication::translate("MainGUIWindow", "LQR Full", 0));
        controllerMotorCmdTest->setText(QApplication::translate("MainGUIWindow", "Motor Cmd Test", 0));
        label_4->setText(QApplication::translate("MainGUIWindow", "Ts [ms]", 0));
        controllerLQRNestedOnboard->setText(QApplication::translate("MainGUIWindow", "LQR Nested Onboard", 0));
        label_3->setText(QApplication::translate("MainGUIWindow", "Type:", 0));
        controllerPIDPosition->setText(QApplication::translate("MainGUIWindow", "PID Position", 0));
        label->setText(QApplication::translate("MainGUIWindow", "Command", 0));
        label_7->setText(QApplication::translate("MainGUIWindow", "Controller", 0));
        label_9->setText(QApplication::translate("MainGUIWindow", "Sample time", 0));
        label_28->setText(QApplication::translate("MainGUIWindow", "Pitch", 0));
        label_6->setText(QString());
        controllerRateCmdTest->setText(QApplication::translate("MainGUIWindow", "Rate Cmd Test", 0));
        controllerAngleCmdTest->setText(QApplication::translate("MainGUIWindow", "Angle Cmd Test", 0));
        label_25->setText(QApplication::translate("MainGUIWindow", "Pitch", 0));
        controllerPIDFull->setText(QApplication::translate("MainGUIWindow", "PID Full", 0));
        label_27->setText(QApplication::translate("MainGUIWindow", "Roll", 0));
        controllerLQRNestedOffboard->setText(QApplication::translate("MainGUIWindow", "LQR Nested Offboard", 0));
        label_26->setText(QApplication::translate("MainGUIWindow", "Yaw", 0));
        label_29->setText(QApplication::translate("MainGUIWindow", "Yaw", 0));
        controllerPIDAngle->setText(QApplication::translate("MainGUIWindow", "PID Angle", 0));
        label_8->setText(QApplication::translate("MainGUIWindow", "Roll", 0));
        label_5->setText(QApplication::translate("MainGUIWindow", "Cmd", 0));
        label_55->setText(QApplication::translate("MainGUIWindow", "PID Ts", 0));
        label_56->setText(QApplication::translate("MainGUIWindow", "LQR Full Ts", 0));
        label_72->setText(QApplication::translate("MainGUIWindow", "LQR Nes Ts", 0));
        buttonSetDefaultTs->setText(QApplication::translate("MainGUIWindow", "Set Default Ts", 0));
        buttonDefaultFeedforward->setText(QApplication::translate("MainGUIWindow", "Set default ff cmd", 0));
        label_53->setText(QApplication::translate("MainGUIWindow", "Rate Controller", 0));
        labelRateMode->setText(QApplication::translate("MainGUIWindow", "Onboard", 0));
        label_54->setText(QApplication::translate("MainGUIWindow", "Ts [ms]", 0));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_3), QApplication::translate("MainGUIWindow", "Control", 0));
        QTableWidgetItem *___qtablewidgetitem = RateParamTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("MainGUIWindow", "Kp", 0));
        QTableWidgetItem *___qtablewidgetitem1 = RateParamTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("MainGUIWindow", "Ki", 0));
        QTableWidgetItem *___qtablewidgetitem2 = RateParamTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("MainGUIWindow", "Kd", 0));
        QTableWidgetItem *___qtablewidgetitem3 = RateParamTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("MainGUIWindow", "N", 0));
        QTableWidgetItem *___qtablewidgetitem4 = RateParamTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QApplication::translate("MainGUIWindow", "Min", 0));
        QTableWidgetItem *___qtablewidgetitem5 = RateParamTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QApplication::translate("MainGUIWindow", "Max", 0));
        QTableWidgetItem *___qtablewidgetitem6 = RateParamTable->verticalHeaderItem(0);
        ___qtablewidgetitem6->setText(QApplication::translate("MainGUIWindow", "Yaw", 0));
        QTableWidgetItem *___qtablewidgetitem7 = RateParamTable->verticalHeaderItem(1);
        ___qtablewidgetitem7->setText(QApplication::translate("MainGUIWindow", "Pitch", 0));
        QTableWidgetItem *___qtablewidgetitem8 = RateParamTable->verticalHeaderItem(2);
        ___qtablewidgetitem8->setText(QApplication::translate("MainGUIWindow", "Roll", 0));
        buttonSetDefaultRateParams->setText(QApplication::translate("MainGUIWindow", "Set Default", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QApplication::translate("MainGUIWindow", "Rate", 0));
        QTableWidgetItem *___qtablewidgetitem9 = PIDParamTable->horizontalHeaderItem(0);
        ___qtablewidgetitem9->setText(QApplication::translate("MainGUIWindow", "Kp", 0));
        QTableWidgetItem *___qtablewidgetitem10 = PIDParamTable->horizontalHeaderItem(1);
        ___qtablewidgetitem10->setText(QApplication::translate("MainGUIWindow", "Ki", 0));
        QTableWidgetItem *___qtablewidgetitem11 = PIDParamTable->horizontalHeaderItem(2);
        ___qtablewidgetitem11->setText(QApplication::translate("MainGUIWindow", "Kd", 0));
        QTableWidgetItem *___qtablewidgetitem12 = PIDParamTable->horizontalHeaderItem(3);
        ___qtablewidgetitem12->setText(QApplication::translate("MainGUIWindow", "N", 0));
        QTableWidgetItem *___qtablewidgetitem13 = PIDParamTable->horizontalHeaderItem(4);
        ___qtablewidgetitem13->setText(QApplication::translate("MainGUIWindow", "Min", 0));
        QTableWidgetItem *___qtablewidgetitem14 = PIDParamTable->horizontalHeaderItem(5);
        ___qtablewidgetitem14->setText(QApplication::translate("MainGUIWindow", "Max", 0));
        QTableWidgetItem *___qtablewidgetitem15 = PIDParamTable->verticalHeaderItem(0);
        ___qtablewidgetitem15->setText(QApplication::translate("MainGUIWindow", "X", 0));
        QTableWidgetItem *___qtablewidgetitem16 = PIDParamTable->verticalHeaderItem(1);
        ___qtablewidgetitem16->setText(QApplication::translate("MainGUIWindow", "Y", 0));
        QTableWidgetItem *___qtablewidgetitem17 = PIDParamTable->verticalHeaderItem(2);
        ___qtablewidgetitem17->setText(QApplication::translate("MainGUIWindow", "Z", 0));
        QTableWidgetItem *___qtablewidgetitem18 = PIDParamTable->verticalHeaderItem(3);
        ___qtablewidgetitem18->setText(QApplication::translate("MainGUIWindow", "Yaw", 0));
        QTableWidgetItem *___qtablewidgetitem19 = PIDParamTable->verticalHeaderItem(4);
        ___qtablewidgetitem19->setText(QApplication::translate("MainGUIWindow", "Pitch", 0));
        QTableWidgetItem *___qtablewidgetitem20 = PIDParamTable->verticalHeaderItem(5);
        ___qtablewidgetitem20->setText(QApplication::translate("MainGUIWindow", "Roll", 0));

        const bool __sortingEnabled = PIDParamTable->isSortingEnabled();
        PIDParamTable->setSortingEnabled(false);
        PIDParamTable->setSortingEnabled(__sortingEnabled);

        buttonPIDDefaultParams->setText(QApplication::translate("MainGUIWindow", "Set Default", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainGUIWindow", "PID", 0));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_4), QApplication::translate("MainGUIWindow", "Parameters", 0));
        label_23->setText(QApplication::translate("MainGUIWindow", "Yaw [deg]", 0));
        label_20->setText(QApplication::translate("MainGUIWindow", "X [mm]", 0));
        label_21->setText(QApplication::translate("MainGUIWindow", "Y [mm]", 0));
        label_22->setText(QApplication::translate("MainGUIWindow", "Z [mm]", 0));
        label_24->setText(QApplication::translate("MainGUIWindow", "Setpoint", 0));
        buttonSetpointChange->setText(QApplication::translate("MainGUIWindow", "Change", 0));
        buttonSetpointCurrPos->setText(QApplication::translate("MainGUIWindow", "Curr pos", 0));
        buttonStop->setText(QApplication::translate("MainGUIWindow", "STOP", 0));
        buttonResetControllers->setText(QApplication::translate("MainGUIWindow", "Reset Controllers", 0));
        label_36->setText(QApplication::translate("MainGUIWindow", "Missed measurements", 0));
        buttonResetMissed->setText(QApplication::translate("MainGUIWindow", "Reset missed", 0));
        SetpointHome->setText(QApplication::translate("MainGUIWindow", "Home", 0));
        setpointZ200->setText(QApplication::translate("MainGUIWindow", "z=200", 0));
        label_16->setText(QApplication::translate("MainGUIWindow", "Yaw [deg]", 0));
        label_18->setText(QApplication::translate("MainGUIWindow", "Pitch [deg]", 0));
        label_15->setText(QApplication::translate("MainGUIWindow", "Z [mm]", 0));
        label_17->setText(QApplication::translate("MainGUIWindow", "Vicon Data", 0));
        label_13->setText(QApplication::translate("MainGUIWindow", "X [mm]", 0));
        label_14->setText(QApplication::translate("MainGUIWindow", "Y [mm]", 0));
        label_19->setText(QApplication::translate("MainGUIWindow", "Roll [deg]", 0));
        trajCustom->setText(QApplication::translate("MainGUIWindow", "Custom", 0));
        trajCircle->setText(QApplication::translate("MainGUIWindow", "Circle", 0));
        trajSquare->setText(QApplication::translate("MainGUIWindow", "Square", 0));
        buttonStop_2->setText(QApplication::translate("MainGUIWindow", "start cal", 0));
    } // retranslateUi

};

namespace Ui {
    class MainGUIWindow: public Ui_MainGUIWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINGUIWINDOW_H
