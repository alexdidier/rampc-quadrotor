//    Tab that is created when we create a crazyflie zone
//
//    Copyright (C) 2017  Angel Romero
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
