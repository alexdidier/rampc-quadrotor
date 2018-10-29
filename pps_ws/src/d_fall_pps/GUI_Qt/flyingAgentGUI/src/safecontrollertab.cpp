#include "safecontrollertab.h"
#include "ui_safecontrollertab.h"

SafeControllerTab::SafeControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SafeControllerTab)
{
    ui->setupUi(this);
}

SafeControllerTab::~SafeControllerTab()
{
    delete ui;
}
