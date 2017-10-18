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

#ifndef TABLEPIECE_H
#define TABLEPIECE_H


#include <QGraphicsSimpleTextItem>

#include "myGraphicsRectItem.h"

class tablePiece : public myGraphicsRectItem
{
public:
    explicit tablePiece(const QRectF & rect, QGraphicsItem * parent = 0);
    void rectSizeChanged();     // pure virtual, need to implement it
    void setLightColor();
    void setDarkColor();
protected:

private:
};


#endif
