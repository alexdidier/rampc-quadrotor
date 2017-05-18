#include "crazyFly.h"

#include <QPen>
#include <QBrush>


crazyFly::crazyFly(const CrazyflieData* p_crazyfly_msg, QGraphicsItem * parent)
    : QGraphicsSvgItem(":/images/drone_fixed.svg")
{
    updateCF(p_crazyfly_msg);
    m_width = DRONE_WIDTH;
    m_height = DRONE_HEIGHT;
}

crazyFly::~crazyFly()
{
}

std::string crazyFly::getName()
{
    return m_name;
}


void crazyFly::updateCF(const CrazyflieData* p_crazyfly_msg)
{
    m_name = p_crazyfly_msg->crazyflieName;
    m_x = p_crazyfly_msg->x;
    m_y = p_crazyfly_msg->y;
    m_z = p_crazyfly_msg->z;

    m_yaw = p_crazyfly_msg->yaw;
    m_pitch = p_crazyfly_msg->pitch;
    m_roll = p_crazyfly_msg->roll;
    this->setPos(m_x * FROM_MILIMETERS_TO_UNITS, -m_y * FROM_MILIMETERS_TO_UNITS);    // - y because of coordinates
    this->setRotation(- m_yaw * FROM_RADIANS_TO_DEGREES); //negative beacause anti-clock wise should be positive
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
