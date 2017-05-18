#include "marker.h"

#include <QPen>
#include <QBrush>


Marker::Marker(const UnlabeledMarker* p_marker_msg, QGraphicsItem * parent)
    : QGraphicsEllipseItem(-MARKER_DIAMETER/2, - MARKER_DIAMETER/2, MARKER_DIAMETER, MARKER_DIAMETER, parent)
{
    updateMarker(p_marker_msg);

    _highlighted = false;
    _highlight_diameter = HIGHLIGHT_DIAMETER;

    _diameter = MARKER_DIAMETER; // x and y are top left coordinates

    _x_highlight = m_x - _highlight_diameter/2; // update top-left corner coordinates of highlighing circle
    _y_highlight = m_y - _highlight_diameter/2;
    this->setPen(Qt::NoPen);
    this->setBrush(QColor(255, 0, 0));
    this->setZValue(10);        // max z value, should always be seen
}

void Marker::updateMarker(const UnlabeledMarker* p_marker_msg)
{
    m_x = p_marker_msg->x;
    m_y = p_marker_msg->y;
    m_z = p_marker_msg->z;
    this->setPos(m_x * FROM_METERS_TO_UNITS, -m_y * FROM_METERS_TO_UNITS);    // - y because of coordinates
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
