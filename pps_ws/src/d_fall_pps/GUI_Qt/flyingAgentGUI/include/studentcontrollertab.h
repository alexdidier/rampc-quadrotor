#ifndef STUDENTCONTROLLERTAB_H
#define STUDENTCONTROLLERTAB_H

#include <QWidget>

namespace Ui {
class StudentControllerTab;
}

class StudentControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit StudentControllerTab(QWidget *parent = 0);
    ~StudentControllerTab();

private:
    Ui::StudentControllerTab *ui;
};

#endif // STUDENTCONTROLLERTAB_H
