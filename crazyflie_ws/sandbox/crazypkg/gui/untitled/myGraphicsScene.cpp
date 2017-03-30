#include "myGraphicsScene.h"

#include <QGraphicsSceneMouseEvent>
#include <QRect>
#include <QApplication>

myGraphicsScene::myGraphicsScene(QObject *parent)
    : QGraphicsScene(parent)
{
    pen = new QPen(Qt::black);
    brush = new QBrush(Qt::blue);

    tmp_rect = 0;
    startedRect = false;
    // firstClick = true;
}

void myGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;
    if(Qt::ControlModifier == QApplication::keyboardModifiers())
    {
        // Drag and drop approach
        startedRect = true;
        p1 = new QPointF(mouseEvent->scenePos());
        tmp_rect = new QRectF(*p1, *p1);
        tmp_rect_item = new myGraphicsRectItem(*tmp_rect);
        addItem(tmp_rect_item);
    }

    QGraphicsScene::mousePressEvent(mouseEvent);
}

void myGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if(startedRect)
    {
        tmp_rect_item->setRect(QRectF(*p1, mouseEvent->scenePos()));
        qDebug("Mouse Position: %d, %d", (mouseEvent->scenePos()).toPoint().x(), (mouseEvent->scenePos()).toPoint().y());
        qDebug("Rectangle BottomRight Position: %d, %d", tmp_rect_item->rect().bottomRight().x(), tmp_rect_item->rect().bottomRight().y());
    }
    QGraphicsScene::mouseMoveEvent(mouseEvent);
}

void myGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if(startedRect)
    {
        if (mouseEvent->button() != Qt::LeftButton)
            return;

        // Drag and drop approach:

        // TODO: If too small, etc etc, dont add it to the container and remove it

        tmp_rect_item->setRect(tmp_rect_item->rect().normalized());
        rectangles.push_back(tmp_rect_item);

        tmp_rect = 0;
        startedRect = false;

        // update();

    }


    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
