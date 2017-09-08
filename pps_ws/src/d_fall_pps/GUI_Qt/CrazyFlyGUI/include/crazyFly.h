#ifndef CRAZYFLY_H
#define CRAZYFLY_H

#include "globalDefinitions.h"

#include <QGraphicsSvgItem>
#include <QSvgRenderer>

#ifdef CATKIN_MAKE
#include "d_fall_pps/CrazyflieData.h"
#endif

#ifdef CATKIN_MAKE
using namespace d_fall_pps;
#endif

#define DRONE_HEIGHT         100 * FROM_MILIMETERS_TO_UNITS * 1.2
#define DRONE_WIDTH          100 * FROM_MILIMETERS_TO_UNITS * 1.2

class crazyFly : public QGraphicsSvgItem
{
public:
    explicit crazyFly(const CrazyflieData* p_crazyfly_msg, QString filename, QGraphicsItem * parent = 0);
    ~crazyFly();
    QRectF boundingRect() const;

    void paint(QPainter * painter,
               const QStyleOptionGraphicsItem * option,
               QWidget * widget);

    void updateCF(const CrazyflieData* p_crazyfly_msg);

    std::string getName();

    void setScaleCFs(double scale);

    bool isOccluded();

    // linking stuff
    void assignCFZone(int cf_zone_index);
    void removeAssigned();
    bool isAssigned();
    bool isAddedToScene();
    void setAddedToScene(bool added);

private:
    // item added to scene already:
    bool m_added_to_scene;

    // info to fill by message
    std::string m_name;
    qreal m_x;
    qreal m_y;
    qreal m_z;

    qreal m_roll;
    qreal m_pitch;
    qreal m_yaw;

    bool m_occluded;

    // info for plotting CF
    qreal m_width;
    qreal m_height;

    // linking stuff
    bool m_assigned;
    int m_assigned_cf_zone_index;
};


#endif
