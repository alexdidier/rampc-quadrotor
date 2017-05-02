#include "marker.h"

#include <QPen>
#include <QBrush>


Marker::Marker(qreal x, qreal y, QGraphicsItem * parent)
    : QGraphicsEllipseItem(parent)
{
    _highlighted = false;
    _highlight_diameter = HIGHLIGHT_DIAMETER;

    // save original x and y
    _center_x = x;
    _center_y = y;

    _diameter = MARKER_DIAMETER; // x and y are top left coordinates
    _x = _center_x - _diameter/2;
    _y = _center_y - _diameter/2;
    this->setRect(QRectF(_x, _y, _diameter, _diameter));
    this->setPen(Qt::NoPen);
    this->setBrush(QColor(255, 0, 0));
    this->setZValue(10);        // max z value, should always be seen
}

void Marker::setPosMarker(QPointF new_p)
{
    prepareGeometryChange();

    _center_x = new_p.x();              // update center coordinates
    _center_y = new_p.y();

    _x = _center_x - _diameter/2;
    _y = _center_y - _diameter/2;

    // this->setPos(_x, _y);       //update top-left corner coordinates
    this->setRect(QRectF(_x, _y, _diameter, _diameter));
    if(_highlighted)
    {
        _x_highlight = _center_x - _highlight_diameter/2; // update top-left corner coordinates of highlighing circle
        _y_highlight = _center_y -_highlight_diameter/2;
        // _highlight_circle->setPos(_x_highlight, _y_highlight);
        _highlight_circle->setRect(QRectF(_x_highlight, _y_highlight, _highlight_diameter, _highlight_diameter));
    }
}

void Marker::setHighlighted(void)
{
    if(!_highlighted)
    {
        _x_highlight = _center_x - _highlight_diameter/2;
        _y_highlight = _center_y -_highlight_diameter/2;

        prepareGeometryChange();
        _highlight_circle = new QGraphicsEllipseItem();
        _highlight_circle->setRect(QRectF(_x_highlight, _y_highlight, _highlight_diameter, _highlight_diameter));
        _highlight_circle->setPen(QPen(QBrush(Qt::black), HIGHLIGHT_WIDTH));
        _highlight_circle->setParentItem(this);
        // _highlight_circle->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        _highlighted = true;
    }
}

void Marker::clearHighlighted(void)
{
    if(_highlighted)
    {
        prepareGeometryChange();
        _highlight_circle->setParentItem(NULL);
        delete _highlight_circle;
        _highlighted = false;
    }
}


bool Marker::getHighlighted(void)
{
    return _highlighted;
}

Marker::~Marker()
{
    clearHighlighted();
}



