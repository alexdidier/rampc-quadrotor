#ifndef CRAZYFLYZONE_H
#define CRAZYFLYZONE_H


#include <QGraphicsSimpleTextItem>

#include "myGraphicsRectItem.h"

class crazyFlyZone : public myGraphicsRectItem
{
public:
    explicit crazyFlyZone(const QRectF & rect, int index, QGraphicsItem * parent = 0);
    ~crazyFlyZone();

    int getIndex();
    void setIndex(int index);
    void setLabel(QString string);
    void setLabelPosition();
    void updateLabel(QString string);
    void rectSizeChanged();

    // stuff for linking
    void linkCF(std::string cf_name);
    bool isLinked();
    void removeLink();

protected:

private:
    int _index;
    QGraphicsSimpleTextItem* label;

    // stuff for linking
    bool m_linked;
    std::string m_crazyfly_linked_name; //in the future this will be a vector of crazyFlies maybe
};


#endif
