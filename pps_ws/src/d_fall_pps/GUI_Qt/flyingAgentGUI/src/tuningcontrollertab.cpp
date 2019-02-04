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
