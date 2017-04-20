#ifndef TABLEPIECE_H
#define TABLEPIECE_H


#include <QGraphicsSimpleTextItem>

#include "myGraphicsRectItem.h"

class tablePiece : public myGraphicsRectItem
{
public:
    explicit tablePiece(const QRectF & rect, QGraphicsItem * parent = 0);
    void rectSizeChanged();     // pure virtual, need to implement it
    void setLightColor();
    void setDarkColor();
protected:

private:
};


#endif
