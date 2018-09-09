#ifndef ENABLECONTROLLERLOADYAMLBAR_H
#define ENABLECONTROLLERLOADYAMLBAR_H

#include <QWidget>

namespace Ui {
class EnableControllerLoadYamlBar;
}

class EnableControllerLoadYamlBar : public QWidget
{
    Q_OBJECT

public:
    explicit EnableControllerLoadYamlBar(QWidget *parent = 0);
    ~EnableControllerLoadYamlBar();

private slots:

    // ENABLE CONTROLLER BUTTONS ON-CLICK CALLBACK
    void on_enable_safe_button_clicked();
    void on_enable_demo_button_clicked();
    void on_enable_student_button_clicked();
    void on_enable_mpc_button_clicked();

    // LOAD YAML BUTTONS ON-CLICK CALLBACK
    void on_load_yaml_safe_button_clicked();
    void on_load_yaml_demo_button_clicked();
    void on_load_yaml_student_button_clicked();
    void on_load_yaml_mpc_button_clicked();

private:
    Ui::EnableControllerLoadYamlBar *ui;
};

#endif // ENABLECONTROLLERLOADYAMLBAR_H
