#include "myGraphicsScene.h"

#include <QGraphicsSceneMouseEvent>
#include <QRect>
#include <QGraphicsRectItem>

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


    // Drag and drop approach

    startedRect = true;
    p1 = new QPointF(mouseEvent->scenePos());
    tmp_rect = new QRectF(*p1, *p1);
    // addRect(*tmp_rect, *pen, *brush);
    tmp_rect_item = new QGraphicsRectItem(*tmp_rect);
    rectangles.push_back(tmp_rect_item);
    addItem(rectangles.back());

    // Two-clicks approach
    // if(firstClick)
    // {
    //     p1 = new QPointF(mouseEvent->scenePos());
    //     tmp_rect_item = addRect(QRect(p1->toPoint(), p1->toPoint()), *pen, *brush); //save it to remove it after
    // }
    // else
    // {
    //     p2 = new QPointF(mouseEvent->scenePos());
    //     // QRect tmp_rect(*p2, *p2);
    //     // addRect(tmp_rect, *pen, *brush);
    // }


    update();
    QGraphicsScene::mousePressEvent(mouseEvent);
}

void myGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if(startedRect)
    {
        tmp_rect_item->setRect(QRectF(*p1, mouseEvent->scenePos()));
        qDebug("Mouse Position: %d, %d", (mouseEvent->scenePos()).toPoint().x(), (mouseEvent->scenePos()).toPoint().y());
        qDebug("Rectangle BottomRight Position: %d, %d", tmp_rect->bottomRight().x(), tmp_rect->bottomRight().y());
        update();
    }
    QGraphicsScene::mouseMoveEvent(mouseEvent);
}

void myGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{

    if (mouseEvent->button() != Qt::LeftButton)
        return;

    // Drag and drop approach:

    tmp_rect = 0;
    startedRect = false;


    // Two-clicks approach

    // if(firstClick)
    // {
    //     firstClick = false;
    // }
    // else
    // {
    //     removeItem(tmp_rect_item);
    //     tmp_rect_item = new QGraphicsRectItem(QRectF(*p1, *p2));
    //     //            *tmp_rect, *pen, *brush);
    //     rectangles.push_back(tmp_rect_item);

    //     addItem(rectangles.back());
    //     p1 = 0;
    //     p2 = 0;
    //     tmp_rect = 0;
    //     firstClick = true;
    // }

    update();
    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
