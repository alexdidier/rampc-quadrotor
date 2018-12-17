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

void StudentControllerTab::setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded)
{
    if (!occluded)
    {
        // UPDATE THE MEASUREMENT COLUMN
        ui->lineEdit_measured_x    ->setText(QString::number( x     ));
        ui->lineEdit_measured_y    ->setText(QString::number( y     ));
        ui->lineEdit_measured_z    ->setText(QString::number( z     ));
        ui->lineEdit_measured_roll ->setText(QString::number( roll  ));
        ui->lineEdit_measured_pitch->setText(QString::number( pitch ));
        ui->lineEdit_measured_yaw  ->setText(QString::number( yaw   ));

        // GET THE CURRENT SETPOINT
        float curr_x_setpoint   = (ui->lineEdit_setpoint_current_x->text()  ).toFloat();;
        float curr_y_setpoint   = (ui->lineEdit_setpoint_current_y->text()  ).toFloat();;
        float curr_z_setpoint   = (ui->lineEdit_setpoint_current_z->text()  ).toFloat();;
        float curr_yaw_setpoint = (ui->lineEdit_setpoint_current_yaw->text()).toFloat();;

        // UPDATE THE ERROR COLUMN
        ui->lineEdit_error_x  ->setText(QString::number( x   - curr_x_setpoint   ));
        ui->lineEdit_error_y  ->setText(QString::number( y   - curr_y_setpoint   ));
        ui->lineEdit_error_z  ->setText(QString::number( z   - curr_z_setpoint   ));
        ui->lineEdit_error_yaw->setText(QString::number( yaw - curr_yaw_setpoint ));
    }
}
