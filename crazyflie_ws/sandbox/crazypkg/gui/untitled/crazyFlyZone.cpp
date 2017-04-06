#include "crazyFlyZone.h"

crazyFlyZone::crazyFlyZone(const QRectF & rect, int index,  QGraphicsItem * parent)
    : myGraphicsRectItem(rect, parent)
{
    setIndex(index);
}

void crazyFlyZone::updateLabel(QString string)
{
    label->setText(string);
    setLabelPosition();
}

void crazyFlyZone::setLabel(QString string)
{
    label = new QGraphicsSimpleTextItem(string, this);
    label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    label->setFont(QFont("Arial", 18, QFont::Bold, true));
    // TODO: choose position and format of text
    // label->setPos(label->mapToItem(this, QPointF(0,0)));
    setLabelPosition();
}

void crazyFlyZone::setLabelPosition()
{
    qreal x_offset = 10;
    qreal y_offset = 5;
    label->setPos(this->rect().topLeft().x() + x_offset,this->rect().topLeft().y() + y_offset);
}

int crazyFlyZone::getIndex()
{
    return _index;
}

void crazyFlyZone::setIndex(int index)
{
    // TODO: how to make sure that we never have two rectangles with the same index?
    // Maybe only when we reduce the size of the rectangles vector?
    _index = index;
}

void crazyFlyZone::rectSizeChanged()
{
    setLabelPosition();
}
