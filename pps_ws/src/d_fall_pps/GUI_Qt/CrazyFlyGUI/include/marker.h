#ifndef MARKER_H
#define MARKER_H

#include "globalDefinitions.h"

#include <QGraphicsEllipseItem>

#ifdef CATKIN_MAKE
#include "d_fall_pps/UnlabeledMarker.h"
#endif


#ifdef CATKIN_MAKE
using namespace d_fall_pps;
#endif

#define MARKER_DIAMETER        20 * FROM_MILIMETERS_TO_UNITS

#define HIGHLIGHT_DIAMETER     20
#define HIGHLIGHT_WIDTH         5


class Marker : public QGraphicsEllipseItem
{
public:
    explicit Marker(const UnlabeledMarker* p_marker_msg, QGraphicsItem *parent = 0);
    ~Marker();

    void setHighlighted(void);

    void clearHighlighted(void);

    bool getHighlighted(void);

    void updateMarker(const UnlabeledMarker* p_marker_msg);
private:

    // info to fill by message

    qreal m_x;
    qreal m_y;
    qreal m_z;

    // properties of marker itself
    qreal _diameter;

    qreal _center_x;              // coordinates of center of marker
    qreal _center_y;

    bool _highlighted;
    QGraphicsEllipseItem* _highlight_circle;

    qreal _highlight_diameter;
    qreal _x_highlight;         // coordinates of highlighting circle's top-left corner
    qreal _y_highlight;
};


#endif
