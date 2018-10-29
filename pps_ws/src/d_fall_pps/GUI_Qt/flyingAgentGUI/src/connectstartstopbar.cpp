#include "connectstartstopbar.h"
#include "ui_connectstartstopbar.h"

ConnectStartStopBar::ConnectStartStopBar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConnectStartStopBar)
{
    ui->setupUi(this);

    // SET A FEW PROPERTIES OF THE UI ELEMENTS
    // > Default the battery voltage field to be "blank"
    QString qstr = "-.-- V";
    ui->battery_voltage_lineEdit->setText(qstr);
    // > Red font colour for the battery message label
    //ui->battery_message_label->setStyleSheet("QLabel { color : red; }");
    // > Default the battery message label to be "blank"
    //ui->battery_message_label->setText("");

    // SET THE DEFAULT IMAGE FOR THE RF CONNECTION STATUS
    QPixmap rf_disconnected_pixmap(":/images/rf_disconnected.png");
    ui->rf_status_label->setPixmap(rf_disconnected_pixmap);
    ui->rf_status_label->setScaledContents(true);

    // SET THE DEFAULT IMAGE FOR THE BATTERY STATUS
    QPixmap battery_unknown_pixmap(":/images/battery_unknown.png");
    ui->battery_status_label->setPixmap(battery_unknown_pixmap);
    //ui->battery_status_label->setPixmap(battery_status_unknown_pixmap.scaled(ui->battery_status_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->battery_status_label->setScaledContents(true);

    // SET THE DEFAULT IMAGE FOR THE FLYING STATE
    QPixmap flying_state_unknown_pixmap(":/images/flying_state_unknown.png");
    ui->flying_state_label->setPixmap(flying_state_unknown_pixmap);
    ui->flying_state_label->setScaledContents(true);





    // ADD KEYBOARD SHORTCUTS
    // > For "all motors off", press the space bar
    ui->motors_off_button->setShortcut(tr("Space"));


}

ConnectStartStopBar::~ConnectStartStopBar()
{
    delete ui;
}

void ConnectStartStopBar::on_rf_disconnect_button_clicked()
{

}
