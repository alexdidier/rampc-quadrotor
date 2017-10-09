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
