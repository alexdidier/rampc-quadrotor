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
//    Crazyflie object, representation of a Crazyflie in the scene
//
//    ----------------------------------------------------------------------------------


#include "crazyFly.h"

#include <QPen>
#include <QBrush>


crazyFly::crazyFly(const CrazyflieData* p_crazyfly_msg, QString filename, QGraphicsItem * parent)
    : QGraphicsSvgItem(filename)
{
    updateCF(p_crazyfly_msg);
    m_width = DRONE_WIDTH;
    m_height = DRONE_HEIGHT;
    m_assigned = false;
    m_added_to_scene = false;
}

crazyFly::~crazyFly()
{
}

bool crazyFly::isAddedToScene()
{
    return m_added_to_scene;
}

void crazyFly::setAddedToScene(bool added)
{
    m_added_to_scene = added;
}

void crazyFly::setScaleCFs(double scale)
{
    this->setScale(scale);
}

std::string crazyFly::getName()
{
    return m_name;
}


void crazyFly::updateCF(const CrazyflieData* p_crazyfly_msg)
{
    m_occluded = p_crazyfly_msg->occluded;

    m_name = p_crazyfly_msg->crazyflieName;
    if(!m_occluded)             //if it is occluded, the info we got is useless
    {
        m_x = p_crazyfly_msg->x;
        m_y = p_crazyfly_msg->y;
        m_z = p_crazyfly_msg->z;

        m_yaw = p_crazyfly_msg->yaw;
        m_pitch = p_crazyfly_msg->pitch;
        m_roll = p_crazyfly_msg->roll;


        this->setPos(m_x * FROM_METERS_TO_UNITS, -m_y * FROM_METERS_TO_UNITS);    // - y because of coordinates
        this->setRotation(- m_yaw * FROM_RADIANS_TO_DEGREES); //negative beacause anti-clock wise should be positive
    }
}

bool crazyFly::isOccluded()
{
    return m_occluded;
}




QRectF crazyFly::boundingRect() const
{
    // return QRectF(-original_width/2, -original_height/2, original_width, original_height);
    return QRectF(-m_width/2, -m_height/2, m_width, m_height);
}

void crazyFly::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    this->renderer()->render(painter,this->boundingRect());
}

void crazyFly::assignCFZone(int cf_zone_index)
{
    m_assigned = true;
    m_assigned_cf_zone_index = cf_zone_index;
}

void crazyFly::removeAssigned()
{
    if(m_assigned)
    {
        m_assigned = false;
        m_assigned_cf_zone_index = -1;
    }
}

bool crazyFly::isAssigned()
{
    return m_assigned;
}
