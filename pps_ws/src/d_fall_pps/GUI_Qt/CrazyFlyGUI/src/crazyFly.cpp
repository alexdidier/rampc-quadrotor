#include "crazyFly.h"

#include <QPen>
#include <QBrush>


crazyFly::crazyFly(QPointF position, QGraphicsItem * parent)
    : QGraphicsSvgItem(":/images/drone_fixed.svg")
{
    m_width = DRONE_WIDTH;
    m_height = DRONE_HEIGHT;
    this->setPos(position);
}

crazyFly::~crazyFly()
{
}

QRectF crazyFly::boundingRect() const
{
    // return QRectF(-original_width/2, -original_height/2, original_width, original_height);
    return QRectF(-m_width/2, -m_height/2, m_width, m_height);
}

void crazyFly::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    this->renderer()->render(painter,this->boundingRect());
}
