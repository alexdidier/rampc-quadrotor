#include "pickercontrollertab.h"
#include "ui_pickercontrollertab.h"

PickerControllerTab::PickerControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PickerControllerTab)
{
    ui->setupUi(this);
}

PickerControllerTab::~PickerControllerTab()
{
    delete ui;
}
