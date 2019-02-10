//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
//
//    This file is part of D-FaLL-System.
//
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    The coordinator part of the Flying Agent GUI that allows multiple
//    flying agents to be controller from one place, and keeps track
//    of which subset of flying agents to control.
//
//    ----------------------------------------------------------------------------------





#ifndef COORDINATOR_H
#define COORDINATOR_H

#include "coordinatorrow.h"

#include <QWidget>
#include <QVector>
#include <regex>

#include <QTextStream>

namespace Ui {
class Coordinator;
}

class Coordinator : public QWidget
{
    Q_OBJECT

public:
    explicit Coordinator(QWidget *parent = 0);
    ~Coordinator();


public slots:
    void setShouldCoordinateThisAgent(int agentID , bool shouldCoordinate);


signals:
    void agentIDsToCoordinateChanged(QVector<int> agentIDs , bool shouldCoordinateAll);


private:
    QVector<CoordinatorRow*> vector_of_coordinatorRows;

    QVector<bool> vector_of_shouldCoordinate_perRow;

    QVector<int> vector_of_agentID_perRow;

    int level_of_detail_to_display = 1;

    void remove_all_entries_from_vector_of_coordinatorRows();

    void apply_level_of_detail_to_all_entries(int level);

private slots:
    void on_refresh_button_clicked();

    void on_toggle_details_button_clicked();

    void on_delete_button_clicked();

    void on_coordinate_all_checkBox_clicked();

    void emit_signal_with_agentIDs_toCoordinate();

private:
    Ui::Coordinator *ui;
};

#endif // COORDINATOR_H
