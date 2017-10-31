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


#include "crazyFlyZone.h"
#include "globalDefinitions.h"

crazyFlyZone::crazyFlyZone(const QRectF & rect, int index,  QGraphicsItem * parent)
    : myGraphicsRectItem(rect, parent)
{
    createCenterMarker();
    this->setPen(QPen(Qt::black, 0));
    setIndex(index);
    m_linked = false;
}

crazyFlyZone::~crazyFlyZone()
{
}

void crazyFlyZone::updateLabel(QString string)
{
    label->setText(string);
    setLabelPosition();
}

void crazyFlyZone::createCenterMarker()
{
    m_center_marker = new centerMarker(":/images/center_rect.svg", this);
    updateCenterMarker();
    m_center_marker->setZValue(10); //max z value, always on top of things
}

void crazyFlyZone::updateCenterMarker()
{
    qreal x_offset = this->rect().width()/2;
    qreal y_offset = this->rect().height()/2;
    m_center_marker->setPos(this->rect().topLeft().x() + x_offset,this->rect().topLeft().y() + y_offset);
}

void crazyFlyZone::setLabel(QString string)
{
    label = new QGraphicsSimpleTextItem(string, this);
    label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    label->setFont(QFont("Arial", 16, QFont::Bold, true));
    setLabelPosition();
}

void crazyFlyZone::setLabelPosition()
{
    qreal x_offset = 0.1 * FROM_METERS_TO_UNITS;
    qreal y_offset = 0.05 * FROM_METERS_TO_UNITS;
    label->setPos(this->rect().topLeft().x() + x_offset,this->rect().topLeft().y() + y_offset);
}

int crazyFlyZone::getIndex()
{
    return _index;
}

void crazyFlyZone::setIndex(int index)
{
    // TODO: how to make sure that we never have two rectangles with the same index?
    // Maybe only when we reduce the size of the rectangles vector?
    _index = index;
}

void crazyFlyZone::rectSizeChanged() // pure virtual coming from parent
{
    setLabelPosition();
    updateCenterMarker();
}

void crazyFlyZone::linkCF(std::string cf_name)
{
    m_crazyfly_linked_name = cf_name;
    m_linked = true;
}

bool crazyFlyZone::isLinked()
{
    return m_linked;
}

void crazyFlyZone::removeLink()
{
    if(m_linked)
    {
        m_crazyfly_linked_name = "";
        m_linked = false;
    }
}
