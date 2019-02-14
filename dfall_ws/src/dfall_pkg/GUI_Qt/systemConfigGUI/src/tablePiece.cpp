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
//    Table piece, rectangle that serves as a representation of the real table
//
//    ----------------------------------------------------------------------------------


#include "tablePiece.h"

tablePiece::tablePiece(const QRectF & rect,  QGraphicsItem * parent)
    : myGraphicsRectItem(rect, parent)
{
    setLightColor();
    this->setPen(Qt::NoPen);
    this->setZValue(-10);
}

void tablePiece::rectSizeChanged() // pure virtual coming from parent
{
}

void tablePiece::setLightColor()
{
    this->setBrush(QColor(123, 209, 226)); //last byte is transparency
}

void tablePiece::setDarkColor()
{
    this->setBrush(QColor(10, 103, 164)); //last byte is transparency
}
