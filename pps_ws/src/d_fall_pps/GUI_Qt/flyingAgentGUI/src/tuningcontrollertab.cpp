#include "tuningcontrollertab.h"
#include "ui_tuningcontrollertab.h"

TuningControllerTab::TuningControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TuningControllerTab)
{
    ui->setupUi(this);
}

TuningControllerTab::~TuningControllerTab()
{
    delete ui;
}

void TuningControllerTab::on_button_setpoint_toggle_clicked()
{

}

void TuningControllerTab::on_lineEdit_setpoint_editingFinished()
{

}

void TuningControllerTab::on_slider_gain_P_valueChanged(int value)
{

}
