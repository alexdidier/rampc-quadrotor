#include "../../include/tablePiece.h"

tablePiece::tablePiece(const QRectF & rect,  QGraphicsItem * parent)
    : myGraphicsRectItem(rect, parent)
{
    setLightColor();
    this->setPen(Qt::NoPen);
    this->setZValue(-10);
}

void tablePiece::rectSizeChanged() // pure virtual coming from parent
{
}

void tablePiece::setLightColor()
{
    this->setBrush(QColor(123, 209, 226)); //last byte is transparency
}

void tablePiece::setDarkColor()
{
    this->setBrush(QColor(10, 103, 164)); //last byte is transparency
}
