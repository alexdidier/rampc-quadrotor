#ifndef CONTROLLERTABS_H
#define CONTROLLERTABS_H

#include <QWidget>

namespace Ui {
class ControllerTabs;
}

class ControllerTabs : public QWidget
{
    Q_OBJECT

public:
    explicit ControllerTabs(QWidget *parent = 0);
    ~ControllerTabs();

private:
    Ui::ControllerTabs *ui;
};

#endif // CONTROLLERTABS_H
