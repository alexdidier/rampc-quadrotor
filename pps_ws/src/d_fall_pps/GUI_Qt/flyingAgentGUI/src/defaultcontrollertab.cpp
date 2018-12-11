#include "defaultcontrollertab.h"
#include "ui_defaultcontrollertab.h"

DefaultControllerTab::DefaultControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DefaultControllerTab)
{
    ui->setupUi(this);
}

DefaultControllerTab::~DefaultControllerTab()
{
    delete ui;
}

void DefaultControllerTab::on_lineEdit_setpoint_new_x_returnPressed()
{
    ui->set_setpoint_button->animateClick();

#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[DEFAULT CONTROLLER TAB] return pressed for x setpoint";
#endif
}

void DefaultControllerTab::on_lineEdit_setpoint_new_y_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DefaultControllerTab::on_lineEdit_setpoint_new_z_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DefaultControllerTab::on_lineEdit_setpoint_new_yaw_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DefaultControllerTab::on_set_setpoint_button_clicked()
{

#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[DEFAULT CONTROLLER TAB] set setpoint button clicked";
#endif
}
