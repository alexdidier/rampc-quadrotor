#ifndef DEFAULTCONTROLLERTAB_H
#define DEFAULTCONTROLLERTAB_H

#include <QWidget>

namespace Ui {
class DefaultControllerTab;
}

class DefaultControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit DefaultControllerTab(QWidget *parent = 0);
    ~DefaultControllerTab();

private:
    Ui::DefaultControllerTab *ui;
};

#endif // DEFAULTCONTROLLERTAB_H
