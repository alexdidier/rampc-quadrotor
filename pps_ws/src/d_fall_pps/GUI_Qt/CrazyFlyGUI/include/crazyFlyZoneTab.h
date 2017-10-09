#ifndef CRAZYFLYZONETAB_H
#define CRAZYFLYZONETAB_H

#include <QObject>
#include <QWidget>
#include <QPushButton>

class crazyFlyZoneTab : public QWidget
{
    Q_OBJECT
public:
    explicit crazyFlyZoneTab(int index, QWidget *parent = 0);
    QPushButton* center_button;
private:
    int _index;
    int _num_rows;
    int _num_columns;
signals:
    void centerButtonClickedSignal(int index);

public slots:
    void centerButtonClicked();
};


#endif
