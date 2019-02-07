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
//    Teacher's GUI marker object, to represent unlabeled markers.
//
//    ----------------------------------------------------------------------------------


#ifndef MARKER_H
#define MARKER_H

#include "globalDefinitions.h"

#include <QGraphicsEllipseItem>

#ifdef CATKIN_MAKE
#include "d_fall_pps/UnlabeledMarker.h"
#endif


#ifdef CATKIN_MAKE
using namespace d_fall_pps;
#endif

#define MARKER_DIAMETER        20 * FROM_MILIMETERS_TO_UNITS

#define HIGHLIGHT_DIAMETER     20
#define HIGHLIGHT_WIDTH         5


class Marker : public QGraphicsEllipseItem
{
public:
    explicit Marker(const UnlabeledMarker* p_marker_msg, QGraphicsItem *parent = 0);
    ~Marker();

    void setHighlighted(void);

    void clearHighlighted(void);

    bool getHighlighted(void);

    void updateMarker(const UnlabeledMarker* p_marker_msg);
private:

    // info to fill by message

    qreal m_x;
    qreal m_y;
    qreal m_z;

    // properties of marker itself
    qreal _diameter;

    qreal _center_x;              // coordinates of center of marker
    qreal _center_y;

    bool _highlighted;
    QGraphicsEllipseItem* _highlight_circle;

    qreal _highlight_diameter;
    qreal _x_highlight;         // coordinates of highlighting circle's top-left corner
    qreal _y_highlight;
};


#endif
