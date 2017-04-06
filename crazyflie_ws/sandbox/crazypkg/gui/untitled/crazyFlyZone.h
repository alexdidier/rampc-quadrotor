#ifndef CRAZYFLYZONE_H
#define CRAZYFLYZONE_H


#include <QGraphicsSimpleTextItem>

#include "myGraphicsRectItem.h"

class crazyFlyZone : public myGraphicsRectItem
{
public:
    explicit crazyFlyZone(const QRectF & rect, int index, QGraphicsItem * parent = 0);

    int getIndex();
    void setIndex(int index);
    void setLabel(QString string);
    void setLabelPosition();
    void updateLabel(QString string);
    void rectSizeChanged();
protected:

private:
    int _index;
    QGraphicsSimpleTextItem* label;
};


#endif
