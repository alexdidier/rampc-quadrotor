#ifndef MARKER_H
#define MARKER_H

#include "globalDefinitions.h"

#include <QGraphicsEllipseItem>

#define MARKER_DIAMETER        20 * FROM_MILIMETERS_TO_UNITS

#define HIGHLIGHT_DIAMETER     20
#define HIGHLIGHT_WIDTH         5


class Marker : public QGraphicsEllipseItem
{
public:
    explicit Marker(QPointF p, QGraphicsItem *parent = 0);
    ~Marker();

    void setHighlighted(void);

    void clearHighlighted(void);

    bool getHighlighted(void);

    void setPosMarker(QPointF new_p);

private:
    // properties of marker itself
    qreal _diameter;
    qreal _x;                   // coordinates of top-left corner of marker
    qreal _y;

    qreal _center_x;              // coordinates of center of marker
    qreal _center_y;

    bool _highlighted;
    QGraphicsEllipseItem* _highlight_circle;

    qreal _highlight_diameter;
    qreal _x_highlight;         // coordinates of highlighting circle's top-left corner
    qreal _y_highlight;
};


#endif
