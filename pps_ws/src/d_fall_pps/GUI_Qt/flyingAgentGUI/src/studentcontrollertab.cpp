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
