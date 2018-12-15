#include "studentcontrollertab.h"
#include "ui_studentcontrollertab.h"

StudentControllerTab::StudentControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StudentControllerTab)
{
    ui->setupUi(this);
}

StudentControllerTab::~StudentControllerTab()
{
    delete ui;
}

void StudentControllerTab::setMeasuredPose(QVector<float> measuredPose)
{
    // UPDATE THE MEASUREMENT COLUMN
    ui->lineEdit_measured_x    ->setText(QString::number( measuredPose[0] ));
    ui->lineEdit_measured_y    ->setText(QString::number( measuredPose[1] ));
    ui->lineEdit_measured_z    ->setText(QString::number( measuredPose[2] ));
    ui->lineEdit_measured_roll ->setText(QString::number( measuredPose[3] ));
    ui->lineEdit_measured_pitch->setText(QString::number( measuredPose[4] ));
    ui->lineEdit_measured_yaw  ->setText(QString::number( measuredPose[5] ));

    // GET THE CURRENT SETPOINT
    float curr_x_setpoint   = (ui->lineEdit_setpoint_current_x->text()  ).toFloat();;
    float curr_y_setpoint   = (ui->lineEdit_setpoint_current_y->text()  ).toFloat();;
    float curr_z_setpoint   = (ui->lineEdit_setpoint_current_z->text()  ).toFloat();;
    float curr_yaw_setpoint = (ui->lineEdit_setpoint_current_yaw->text()).toFloat();;

    // UPDATE THE ERROR COLUMN
    ui->lineEdit_error_x  ->setText(QString::number( measuredPose[0] - curr_x_setpoint      ));
    ui->lineEdit_error_y  ->setText(QString::number( measuredPose[1] - curr_y_setpoint   ));
    ui->lineEdit_error_z  ->setText(QString::number( measuredPose[2] - curr_z_setpoint   ));
    ui->lineEdit_error_yaw->setText(QString::number( measuredPose[5] - curr_yaw_setpoint ));
}
