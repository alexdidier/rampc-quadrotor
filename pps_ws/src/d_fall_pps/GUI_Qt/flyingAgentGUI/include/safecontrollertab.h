#ifndef SAFECONTROLLERTAB_H
#define SAFECONTROLLERTAB_H

#include <QWidget>

namespace Ui {
class SafeControllerTab;
}

class SafeControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit SafeControllerTab(QWidget *parent = 0);
    ~SafeControllerTab();

private:
    Ui::SafeControllerTab *ui;
};

#endif // SAFECONTROLLERTAB_H
