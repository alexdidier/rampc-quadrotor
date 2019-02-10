#ifndef TEMPLATECONTROLLERTAB_H
#define TEMPLATECONTROLLERTAB_H

#include <QWidget>

namespace Ui {
class TemplateControllerTab;
}

class TemplateControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit TemplateControllerTab(QWidget *parent = 0);
    ~TemplateControllerTab();

private:
    Ui::TemplateControllerTab *ui;
};

#endif // TEMPLATECONTROLLERTAB_H
