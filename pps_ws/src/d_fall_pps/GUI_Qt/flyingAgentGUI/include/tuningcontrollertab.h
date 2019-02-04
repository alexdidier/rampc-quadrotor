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

private:
    Ui::TuningControllerTab *ui;
};

#endif // TUNINGCONTROLLERTAB_H
