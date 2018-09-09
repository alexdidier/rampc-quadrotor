#include "enablecontrollerloadyamlbar.h"
#include "ui_enablecontrollerloadyamlbar.h"

EnableControllerLoadYamlBar::EnableControllerLoadYamlBar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EnableControllerLoadYamlBar)
{
    ui->setupUi(this);
}

EnableControllerLoadYamlBar::~EnableControllerLoadYamlBar()
{
    delete ui;
}




// ENABLE CONTROLLER BUTTONS ON-CLICK CALLBACK

void EnableControllerLoadYamlBar::on_enable_safe_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_enable_demo_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_enable_student_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_enable_mpc_button_clicked()
{

}


// LOAD YAML BUTTONS ON-CLICK CALLBACK

void EnableControllerLoadYamlBar::on_load_yaml_safe_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_load_yaml_demo_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_load_yaml_student_button_clicked()
{

}

void EnableControllerLoadYamlBar::on_load_yaml_mpc_button_clicked()
{

}
