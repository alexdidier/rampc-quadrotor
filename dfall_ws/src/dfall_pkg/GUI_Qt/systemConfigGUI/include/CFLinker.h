//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero
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
//    Links the crazyflies with the student ids and with the crazyflie zones
//
//    ----------------------------------------------------------------------------------


#ifndef CFLINKER_H
#define CFLINKER_H

#include "globalDefinitions.h"
#include "crazyFly.h"
#include "crazyFlyZone.h"
#include "ui_mainguiwindow.h"

#include "rosNodeThread_for_systemConfigGUI.h"

#include <QObject>


class CFLinker : public QObject
{
    Q_OBJECT
public:

    struct link {
        int student_id;
        int cf_zone_index;
        std::string cf_name;
        std::string radio_address;;
    };

    explicit CFLinker(Ui::MainGUIWindow* ui, std::vector<crazyFly*> *crazyflies_vector, std::vector<crazyFlyZone*> *crazyfly_zones);
    ~CFLinker();

    void link(int student_id, int cf_zone_index, std::string cf_name, std::string radio_address);
    void unlink_selection();
    void unlink_cf_zone(int cf_zone_index);

    std::vector<struct link> links;

    bool isStudentIDLinked(int student_id);
    bool isCFZoneLinked(int cf_zone_index);
    bool isCFLinked(std::string cf_name);
    bool isRadioAddressLinked(std::string radio_address);
    int getCFZoneIndexFromName(QString name);
    int getCFIndexFromName(std::string name);

    void clear_all_links();

signals:
    void updateComboBoxes();

private:

    // QTableWidget m_p_table;

    Ui::MainGUIWindow* m_ui;
    std::vector<crazyFly*>* m_crazyflies_vector;
    std::vector<crazyFlyZone*>* m_crazyfly_zones;

    void addNewRow(int student_id, std::string crazyfly_name, int cf_zone_index, std::string radio_address);
};


#endif
