#include "marker.h"

#include <QPen>
#include <QBrush>


Marker::Marker(QPointF p, QGraphicsItem * parent)
    : QGraphicsEllipseItem(-MARKER_DIAMETER/2, - MARKER_DIAMETER/2, MARKER_DIAMETER, MARKER_DIAMETER, parent)
{
    _highlighted = false;
    _highlight_diameter = HIGHLIGHT_DIAMETER;

    // save original x and y
    _center_x = p.x();
    _center_y = p.y();

    _diameter = MARKER_DIAMETER; // x and y are top left coordinates
    this->setPos(_center_x, _center_y);          //where it is now, it is the center

    _x_highlight = _center_x - _highlight_diameter/2; // update top-left corner coordinates of highlighing circle
    _y_highlight = _center_y - _highlight_diameter/2;
    this->setPen(Qt::NoPen);
    this->setBrush(QColor(255, 0, 0));
    this->setZValue(10);        // max z value, should always be seen
}

void Marker::setPosMarker(QPointF new_p)
{
    prepareGeometryChange();
    this->setPos(_center_x, _center_y);

    _center_x = new_p.x();              // update center coordinates
    _center_y = new_p.y();
}

void Marker::setHighlighted(void)
{
    if(!_highlighted)
    {
        prepareGeometryChange();
        _highlight_circle = new QGraphicsEllipseItem(QRectF(-_highlight_diameter/2, -_highlight_diameter/2, _highlight_diameter, _highlight_diameter), this);
        _highlight_circle->setPos(0, 0);
        _highlight_circle->setPen(QPen(QBrush(Qt::black), HIGHLIGHT_WIDTH));
        _highlight_circle->setFlag(QGraphicsItem::ItemIgnoresTransformations);
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
