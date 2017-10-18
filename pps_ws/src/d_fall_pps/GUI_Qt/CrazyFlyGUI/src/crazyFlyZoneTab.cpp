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

#include "crazyFlyZoneTab.h"

#include <QLayout>

crazyFlyZoneTab::crazyFlyZoneTab(int index, QWidget *parent)
    : QWidget(parent)
{
    _index = index;
    _num_rows = 3;
    _num_columns = 3;
    center_button = new QPushButton("Fit view");
    QGridLayout *mainLayout = new QGridLayout;
    // mainLayout->setRowMinimumHeight(1, 25);
    // mainLayout->setRowMinimumHeight(2, 25);
    // mainLayout->setRowStretch(1);
    // mainLayout->setColumnMinimumWidth(5);
    for(int i = 0; i < _num_rows; i++)
    {
        mainLayout->setRowStretch(i, 1);
    }

    for(int i = 0; i < _num_columns; i++)
    {
        mainLayout->setColumnStretch(i, 1);
    }

    mainLayout->addWidget(center_button, _num_rows - 1, _num_columns - 1);
    setLayout(mainLayout);
    QObject::connect(center_button, SIGNAL(clicked()), this, SLOT(centerButtonClicked()));
    qDebug("tab widget created, index: %d", _index);
}

void crazyFlyZoneTab::centerButtonClicked()
{
    qDebug("index clicked: %d", _index);
    emit centerButtonClickedSignal(_index);
}
