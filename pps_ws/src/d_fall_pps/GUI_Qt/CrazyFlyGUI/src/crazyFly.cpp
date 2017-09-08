#include "crazyFly.h"

#include <QPen>
#include <QBrush>


crazyFly::crazyFly(const CrazyflieData* p_crazyfly_msg, QString filename, QGraphicsItem * parent)
    : QGraphicsSvgItem(filename)
{
    updateCF(p_crazyfly_msg);
    m_width = DRONE_WIDTH;
    m_height = DRONE_HEIGHT;
    m_assigned = false;
    m_added_to_scene = false;
}

crazyFly::~crazyFly()
{
}

bool crazyFly::isAddedToScene()
{
    return m_added_to_scene;
}

void crazyFly::setAddedToScene(bool added)
{
    m_added_to_scene = added;
}

void crazyFly::setScaleCFs(double scale)
{
    this->setScale(scale);
}

std::string crazyFly::getName()
{
    return m_name;
}


void crazyFly::updateCF(const CrazyflieData* p_crazyfly_msg)
{
    m_occluded = p_crazyfly_msg->occluded;

    m_name = p_crazyfly_msg->crazyflieName;
    if(!m_occluded)             //if it is occluded, the info we got is useless
    {
        m_x = p_crazyfly_msg->x;
        m_y = p_crazyfly_msg->y;
        m_z = p_crazyfly_msg->z;

        m_yaw = p_crazyfly_msg->yaw;
        m_pitch = p_crazyfly_msg->pitch;
        m_roll = p_crazyfly_msg->roll;


        this->setPos(m_x * FROM_METERS_TO_UNITS, -m_y * FROM_METERS_TO_UNITS);    // - y because of coordinates
        this->setRotation(- m_yaw * FROM_RADIANS_TO_DEGREES); //negative beacause anti-clock wise should be positive
    }
}

bool crazyFly::isOccluded()
{
    return m_occluded;
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

void crazyFly::assignCFZone(int cf_zone_index)
{
    m_assigned = true;
    m_assigned_cf_zone_index = cf_zone_index;
}

void crazyFly::removeAssigned()
{
    if(m_assigned)
    {
        m_assigned = false;
        m_assigned_cf_zone_index = -1;
    }
}

bool crazyFly::isAssigned()
{
    return m_assigned;
}
