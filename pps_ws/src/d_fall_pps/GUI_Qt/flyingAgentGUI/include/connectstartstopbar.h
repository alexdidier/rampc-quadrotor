#ifndef CONNECTSTARTSTOPBAR_H
#define CONNECTSTARTSTOPBAR_H

#include <QWidget>

//#include <QGraphicsSvgItem>
//#include <QSvgRenderer>

namespace Ui {
class ConnectStartStopBar;
}

class ConnectStartStopBar : public QWidget
{
    Q_OBJECT

public:
    explicit ConnectStartStopBar(QWidget *parent = 0);
    ~ConnectStartStopBar();

private slots:
    void on_rf_disconnect_button_clicked();

private:
    Ui::ConnectStartStopBar *ui;
};

#endif // CONNECTSTARTSTOPBAR_H
