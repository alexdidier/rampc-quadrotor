#ifndef CRAZYFLYZONE_H
#define CRAZYFLYZONE_H


#include <QGraphicsSimpleTextItem>
#include "myGraphicsRectItem.h"
#include "centerMarker.h"

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

    void updateCenterMarker();

protected:

private:
    int _index;
    QGraphicsSimpleTextItem* label;
    centerMarker* m_center_marker;

    // stuff for linking
    bool m_linked;
    std::string m_crazyfly_linked_name; //in the future this will be a vector of crazyFlies maybe

    void createCenterMarker();
};

#endif
