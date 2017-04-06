#include "myGraphicsScene.h"

#include <QGraphicsSceneMouseEvent>
#include <QRect>
#include <QApplication>
#include <QKeyEvent>

myGraphicsScene::myGraphicsScene(QObject *parent)
    : QGraphicsScene(parent)
{
    pen = new QPen(Qt::black);
    brush = new QBrush(Qt::blue);

    tmp_rect = 0;
    startedRect = false;
    // firstClick = true;
}


void myGraphicsScene::keyPressEvent(QKeyEvent * keyEvent)
{
    if(keyEvent->key() == Qt::Key_Delete)
    {
        qDebug("del key pressed");
        for(unsigned int i = 0; i < rectangles.size(); i++)
        {
            if(rectangles[i]->isSelected())
            {
                qDebug("selectedRectangle: %d", i);
                removeRectangle(i);
            }
        }
    }
    QGraphicsScene::keyPressEvent(keyEvent);
}

void myGraphicsScene::setSelectedRectangle(int index)
{
    for(unsigned int i = 0; i < rectangles.size(); i++)
    {
        rectangles[i]->setSelected(false);
        if(index == i)
        {
            rectangles[index]->setSelected(true);
        }
    }
}

int myGraphicsScene::checkSelectedRectangle()
{
    for(unsigned int i = 0; i < rectangles.size(); i++)
    {
        if(rectangles[i]->isSelected())
        {
            qDebug("rectangle selected index = %d", i);
            return i;
        }
    }
    return -1;
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
        int index = rectangles.size();
        tmp_rect_item = new crazyFlyZone(*tmp_rect, index);
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

void myGraphicsScene::addRectangleToVector(crazyFlyZone* rect)
{
    rectangles.push_back(rect);
    emit numRectanglesChanged(rectangles.size());
}

void myGraphicsScene::updateIndexesAndLabels()
{
    for(int i = 0; i < rectangles.size(); i++)
    {
        rectangles[i]->setIndex(i);
        std::string str = std::to_string(i + 1);
        rectangles[i]->updateLabel(str.c_str());
        qDebug("reset Index %d and update label",i);

    }
}

void myGraphicsScene::removeRectangle(int index)
{
    this->removeItem(rectangles[index]);
    rectangles.erase(rectangles.begin() + index);
    qDebug("removed Rectangle %d", index);
    updateIndexesAndLabels();
    emit numRectanglesChanged(rectangles.size()); // for tab managing
}

void myGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if(startedRect)
    {
        if (mouseEvent->button() != Qt::LeftButton)
            return;
        // TODO: If too small, etc etc, dont add it to the container and remove it

        tmp_rect_item->setRect(tmp_rect_item->rect().normalized());
        addRectangleToVector(tmp_rect_item);
        std::string str = std::to_string(rectangles.size());
        tmp_rect_item->setLabel(str.c_str());
        setSelectedRectangle(rectangles.size() - 1); //select just created rectangle
        tmp_rect = 0;
        startedRect = false;
    }
    int selected_rect = checkSelectedRectangle();
    if(selected_rect != -1)
    {
        emit rectangleSelected(selected_rect);
    }

    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
