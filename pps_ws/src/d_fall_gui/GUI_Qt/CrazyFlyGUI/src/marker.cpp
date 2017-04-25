#include "marker.h"

#include <QPen>
#include <QBrush>


Marker::Marker(qreal x, qreal y, QGraphicsItem * parent)
    : QGraphicsEllipseItem(parent)
{
    _highlighted = false;
    _highlight_diameter = HIGHLIGHT_DIAMETER;
    _x_highlight = x - _highlight_diameter/2;
    _y_highlight = y -_highlight_diameter/2;

    _diameter = MARKER_DIAMETER; // x and y are top left coordinates
    _x = x - _diameter/2;
    _y = y - _diameter/2;
    this->setRect(QRectF(_x, _y, _diameter, _diameter));
    this->setPen(Qt::NoPen);
    this->setBrush(QColor(255, 0, 0));
    this->setZValue(10);        // max z value, should always be seen

    setHighlighted();
}

void Marker::setHighlighted(void)
{
    prepareGeometryChange();
    _highlight_circle = new QGraphicsEllipseItem();
    _highlight_circle->setRect(QRectF(_x_highlight, _y_highlight, _highlight_diameter, _highlight_diameter));
    _highlight_circle->setPen(QPen(QBrush(Qt::black), HIGHLIGHT_WIDTH));
    _highlight_circle->setParentItem(this);
    _highlight_circle->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    _highlighted = true;
}

void Marker::clearHighlighted(void)
{
    prepareGeometryChange();
    _highlighted = false;
}


bool Marker::getHighlighted(void)
{
    return _highlighted;
}



