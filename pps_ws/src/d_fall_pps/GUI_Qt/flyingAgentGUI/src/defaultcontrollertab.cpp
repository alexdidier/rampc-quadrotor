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
