//    Marker for the center of the Crazyflie Zone
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

#include "centerMarker.h"

#include <QPen>
#include <QBrush>


centerMarker::centerMarker(QString filename, QGraphicsItem * parent)
    : QGraphicsSvgItem(filename, parent)
{
    m_diameter = DIAMETER;
}

centerMarker::~centerMarker()
{
}

QRectF centerMarker::boundingRect() const
{
    return QRectF(-m_diameter/2, -m_diameter/2, m_diameter, m_diameter);
}

void centerMarker::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    this->renderer()->render(painter,this->boundingRect());
}
