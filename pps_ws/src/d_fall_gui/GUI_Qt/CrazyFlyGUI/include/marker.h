#ifndef MARKER_H
#define MARKER_H

#include "globalDefinitions.h"

#include <QGraphicsEllipseItem>

#define MARKER_DIAMETER        20 * TO_MILIMETERS

#define HIGHLIGHT_DIAMETER     20
#define HIGHLIGHT_WIDTH         5

class Marker : public QGraphicsEllipseItem
{

public:
    explicit Marker(qreal x, qreal y, QGraphicsItem *parent = 0);

    void setHighlighted(void);

    void clearHighlighted(void);

    bool getHighlighted(void);
private:
    qreal _diameter;
    qreal _x;
    qreal _y;

    bool _highlighted;
    QGraphicsEllipseItem* _highlight_circle;

    qreal _highlight_diameter;
    qreal _x_highlight;
    qreal _y_highlight;
};


#endif
