#ifndef CRAZYFLY_H
#define CRAZYFLY_H

#include "globalDefinitions.h"

#include <QGraphicsSvgItem>
#include <QSvgRenderer>


#define DRONE_HEIGHT         100 * FROM_MILIMETERS_TO_UNITS
#define DRONE_WIDTH          100 * FROM_MILIMETERS_TO_UNITS

class crazyFly : public QGraphicsSvgItem
{
public:
    explicit crazyFly(QPointF position, QGraphicsItem * parent = 0);
    ~crazyFly();
    QRectF boundingRect() const;

    void paint(QPainter * painter,
               const QStyleOptionGraphicsItem * option,
               QWidget * widget);
private:
    qreal m_width;
    qreal m_height;
};


#endif
