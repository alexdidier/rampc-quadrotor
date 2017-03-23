#include "myGraphicsScene.h"

#include <QGraphicsSceneMouseEvent>
#include <QRect>

myGraphicsScene::myGraphicsScene(QObject *parent)
    : QGraphicsScene(parent)
{
    pen = new QPen(Qt::black);
    brush = new QBrush(Qt::blue);
    rect = 0;
    // startedRect = false;
    firstClick = true;
}


void myGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;


    // rect = new QRect((mouseEvent->scenePos()).toPoint(), (mouseEvent->scenePos()).toPoint());
    // addRect(*rect, *pen, *brush);
    // startedRect = true;

    if(firstClick)
    {
        p1 = new QPoint((mouseEvent->scenePos()).toPoint());
        QRect tmp_rect(*p1, *p1);
        addRect(tmp_rect, *pen, *brush);
    }
    else
    {
        p2 = new QPoint((mouseEvent->scenePos()).toPoint());
        QRect tmp_rect(*p2, *p2);
        addRect(tmp_rect, *pen, *brush);
    }
    QGraphicsScene::mousePressEvent(mouseEvent);
}

void myGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    // if(startedRect)
    // {
    //     rect->moveBottomRight((mouseEvent->scenePos()).toPoint());
    //     qDebug("Mouse Position: %d, %d", (mouseEvent->scenePos()).toPoint().x(), (mouseEvent->scenePos()).toPoint().y());
    //     qDebug("Rectangle BottomRight Position: %d, %d", rect->bottomRight().x(), rect->bottomRight().y());
    // }
    QGraphicsScene::mouseMoveEvent(mouseEvent);
}

void myGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{

    if (mouseEvent->button() != Qt::LeftButton)
        return;
    // rect = 0;
    // startedRect = false;
    if(firstClick)
    {
        firstClick = false;
    }
    else
    {
        rect = new QRect(*p1, *p2);
        addRect(*rect, *pen, *brush);
        p1 = 0;
        p2 = 0;
        rect = 0;
        firstClick = true;
    }
    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
