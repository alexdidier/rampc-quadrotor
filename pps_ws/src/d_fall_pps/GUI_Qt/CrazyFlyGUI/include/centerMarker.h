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

#ifndef CENTER_MARKER_H
#define CENTER_MARKER_H

#include "globalDefinitions.h"

#include <QGraphicsSvgItem>
#include <QSvgRenderer>

#define DIAMETER         50 * FROM_MILIMETERS_TO_UNITS

class centerMarker : public QGraphicsSvgItem
{
public:
    explicit centerMarker(QString filename, QGraphicsItem * parent = 0);
    ~centerMarker();
    QRectF boundingRect() const;

    void paint(QPainter * painter,
               const QStyleOptionGraphicsItem * option,
               QWidget * widget);

private:
    qreal m_diameter;
};


#endif
