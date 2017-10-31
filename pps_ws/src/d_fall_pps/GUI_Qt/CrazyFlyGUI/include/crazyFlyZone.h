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
//    Rectangular zone that we will be able to link to a crazyflie
//
//    ----------------------------------------------------------------------------------


#ifndef CRAZYFLYZONE_H
#define CRAZYFLYZONE_H


#include <QGraphicsSimpleTextItem>
#include "myGraphicsRectItem.h"
#include "centerMarker.h"

class crazyFlyZone : public myGraphicsRectItem
{
public:
    explicit crazyFlyZone(const QRectF & rect, int index, QGraphicsItem * parent = 0);
    ~crazyFlyZone();

    int getIndex();
    void setIndex(int index);
    void setLabel(QString string);
    void setLabelPosition();
    void updateLabel(QString string);
    void rectSizeChanged();

    // stuff for linking
    void linkCF(std::string cf_name);
    bool isLinked();
    void removeLink();

    void updateCenterMarker();

protected:

private:
    int _index;
    QGraphicsSimpleTextItem* label;
    centerMarker* m_center_marker;

    // stuff for linking
    bool m_linked;
    std::string m_crazyfly_linked_name; //in the future this will be a vector of crazyFlies maybe

    void createCenterMarker();
};

#endif
