#ifndef PICKERCONTROLLERTAB_H
#define PICKERCONTROLLERTAB_H

#include <QWidget>

namespace Ui {
class PickerControllerTab;
}

class PickerControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit PickerControllerTab(QWidget *parent = 0);
    ~PickerControllerTab();

private:
    Ui::PickerControllerTab *ui;
};

#endif // PICKERCONTROLLERTAB_H
