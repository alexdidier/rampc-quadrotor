#ifndef TUNINGCONTROLLERTAB_H
#define TUNINGCONTROLLERTAB_H

#include <QWidget>

namespace Ui {
class TuningControllerTab;
}

class TuningControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit TuningControllerTab(QWidget *parent = 0);
    ~TuningControllerTab();

private slots:
    void on_button_setpoint_toggle_clicked();

    void on_lineEdit_setpoint_editingFinished();

    void on_slider_gain_P_valueChanged(int value);

private:
    Ui::TuningControllerTab *ui;
};

#endif // TUNINGCONTROLLERTAB_H
