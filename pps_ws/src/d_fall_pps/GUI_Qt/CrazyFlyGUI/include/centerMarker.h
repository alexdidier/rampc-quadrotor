#ifndef CENTER_MARKER_H
#define CENTER_MARKER_H

#include "globalDefinitions.h"

#include <QGraphicsSvgItem>
#include <QSvgRenderer>

#define DIAMETER         50 * FROM_MILIMETERS_TO_UNITS

class centerMarker : public QGraphicsSvgItem
{
public:
    explicit centerMarker(QString filename, QGraphicsItem * parent = 0);
    ~centerMarker();
    QRectF boundingRect() const;

    void paint(QPainter * painter,
               const QStyleOptionGraphicsItem * option,
               QWidget * widget);

private:
    qreal m_diameter;
};


#endif
