#include "centerMarker.h"

#include <QPen>
#include <QBrush>


centerMarker::centerMarker(QString filename, QGraphicsItem * parent)
    : QGraphicsSvgItem(filename, parent)
{
    m_diameter = DIAMETER;
}

centerMarker::~centerMarker()
{
}

QRectF centerMarker::boundingRect() const
{
    return QRectF(-m_diameter/2, -m_diameter/2, m_diameter, m_diameter);
}

void centerMarker::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    this->renderer()->render(painter,this->boundingRect());
}
