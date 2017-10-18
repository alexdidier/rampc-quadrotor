//    Abstract class to represent our reimplementation of QGraphicsRectItem. Will be the base
//    for table and crazyflie zone.
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

#ifndef MYGRAPHICSRECTITEM_H
#define MYGRAPHICSRECTITEM_H

#include <QGraphicsRectItem>
#include "cornergrabber.h"

class QGraphicsSceneMouseEvent;
class QPointF;
class QColor;


class myGraphicsRectItem : public QGraphicsRectItem
{
public:
    explicit myGraphicsRectItem(const QRectF & rect, QGraphicsItem * parent = 0);
    void lock();
    void unlock();


public slots:

signals:

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

    virtual void rectSizeChanged() = 0; // pure virtual function, has to be overridden in derived class


    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

private:
    void setCornerPositions();
    bool anyGrabber();
    int checkCornerGrabbers();
    void createGrabbers();
    bool grabbersAreCreated();
    void deleteGrabbers();

    QPen* pen;
    QBrush* brush;
    QRectF* tmp_rect;
    QGraphicsRectItem* tmp_rect_item;
    QPointF* p1;
    QPointF* p2;

    CornerGrabber* _bottomLeft_corner;
    CornerGrabber* _topLeft_corner;
    CornerGrabber* _topRight_corner;
    CornerGrabber* _bottomRight_corner;

    bool _grabbers_created;
    bool resize_mode;
    bool locked;

};

#endif
