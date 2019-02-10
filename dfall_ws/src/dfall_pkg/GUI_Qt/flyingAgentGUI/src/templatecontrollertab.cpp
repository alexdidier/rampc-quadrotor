#include "templatecontrollertab.h"
#include "ui_templatecontrollertab.h"

TemplateControllerTab::TemplateControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TemplateControllerTab)
{
    ui->setupUi(this);
}

TemplateControllerTab::~TemplateControllerTab()
{
    delete ui;
}
