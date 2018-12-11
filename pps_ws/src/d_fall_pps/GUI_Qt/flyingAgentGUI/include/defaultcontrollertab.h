#ifndef DEFAULTCONTROLLERTAB_H
#define DEFAULTCONTROLLERTAB_H

#include <QWidget>
#include <QTextStream>

namespace Ui {
class DefaultControllerTab;
}

class DefaultControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit DefaultControllerTab(QWidget *parent = 0);
    ~DefaultControllerTab();

private slots:
    void on_lineEdit_setpoint_new_x_returnPressed();
    void on_lineEdit_setpoint_new_y_returnPressed();
    void on_lineEdit_setpoint_new_z_returnPressed();
    void on_lineEdit_setpoint_new_yaw_returnPressed();

    void on_set_setpoint_button_clicked();

private:
    Ui::DefaultControllerTab *ui;
};

#endif // DEFAULTCONTROLLERTAB_H
