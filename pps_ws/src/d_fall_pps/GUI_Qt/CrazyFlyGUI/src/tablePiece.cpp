//    Table piece, rectangle that serves as a representation of the real table
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
