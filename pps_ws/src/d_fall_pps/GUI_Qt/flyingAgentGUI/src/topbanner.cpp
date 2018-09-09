#include "topbanner.h"
#include "ui_topbanner.h"

TopBanner::TopBanner(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TopBanner)
{
    ui->setupUi(this);
}

TopBanner::~TopBanner()
{
    delete ui;
}
