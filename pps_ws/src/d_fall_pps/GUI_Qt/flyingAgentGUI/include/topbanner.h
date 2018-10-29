#ifndef TOPBANNER_H
#define TOPBANNER_H

#include <QWidget>

namespace Ui {
class TopBanner;
}

class TopBanner : public QWidget
{
    Q_OBJECT

public:
    explicit TopBanner(QWidget *parent = 0);
    ~TopBanner();

private:
    Ui::TopBanner *ui;
};

#endif // TOPBANNER_H
