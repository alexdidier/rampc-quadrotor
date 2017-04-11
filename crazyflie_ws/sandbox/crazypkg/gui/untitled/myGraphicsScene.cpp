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
    setMode(mode_table);
    tmp_rect = 0;
    startedRect = false;
}


void myGraphicsScene::keyPressEvent(QKeyEvent * keyEvent)
{

    switch(mode)
    {
        case mode_table:
        {
            if(keyEvent->key() == Qt::Key_Delete)
            {
                qDebug("del key pressed");
                for(unsigned int i = 0; i < table_pieces.size(); i++)
                {
                    if(table_pieces[i]->isSelected())
                    {
                        qDebug("selectedRectangle: %d", i);
                        removeTablePiece(i);
                    }
                }
            }
            break;
        }
        case mode_crazyfly_zones:
        {
            if(keyEvent->key() == Qt::Key_Delete)
            {
                qDebug("del key pressed");
                for(unsigned int i = 0; i < crazyfly_zones.size(); i++)
                {
                    if(crazyfly_zones[i]->isSelected())
                    {
                        qDebug("selectedRectangle: %d", i);
                        removeCrazyFlyZone(i);
                    }
                }
            }
            break;
        }
    }

    QGraphicsScene::keyPressEvent(keyEvent);
}

void myGraphicsScene::setSelectedCrazyFlyZone(int index)
{
    for(unsigned int i = 0; i < crazyfly_zones.size(); i++)
    {
        crazyfly_zones[i]->setSelected(false);
        if(index == i)
        {
            crazyfly_zones[index]->setSelected(true);
        }
    }
}

int myGraphicsScene::checkSelectedCrazyFlyZone()
{
    for(unsigned int i = 0; i < crazyfly_zones.size(); i++)
    {
        if(crazyfly_zones[i]->isSelected())
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
        // TODO: implement table creation mode
        // Drag and drop approach
        startedRect = true;
        p1 = new QPointF(mouseEvent->scenePos());
        switch(mode)
        {
            case mode_table:
            {
                tmp_rect = new QRectF(*p1, *p1);
                tmp_table_piece_item = new tablePiece(*tmp_rect);
                addItem(tmp_table_piece_item);
                break;
            }
            case mode_crazyfly_zones:
            {
                tmp_rect = new QRectF(*p1, *p1);
                int index = crazyfly_zones.size();
                tmp_crazyfly_zone_item = new crazyFlyZone(*tmp_rect, index);
                addItem(tmp_crazyfly_zone_item);

                break;
            }
        }
    }
    else
    {
        QGraphicsScene::mousePressEvent(mouseEvent);
    }
}

void myGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if(startedRect)
    {
        switch(mode)
        {
            case mode_table:
            {
                tmp_table_piece_item->setRect(QRectF(*p1, mouseEvent->scenePos()));
                break;
            }
            case mode_crazyfly_zones:
            {
                tmp_crazyfly_zone_item->setRect(QRectF(*p1, mouseEvent->scenePos()));
                qDebug("Mouse Position: %d, %d", (mouseEvent->scenePos()).toPoint().x(), (mouseEvent->scenePos()).toPoint().y());
                qDebug("Rectangle BottomRight Position: %d, %d", tmp_crazyfly_zone_item->rect().bottomRight().x(), tmp_crazyfly_zone_item->rect().bottomRight().y());
                break;
            }
        }

    }
    QGraphicsScene::mouseMoveEvent(mouseEvent);
}

void myGraphicsScene::addCrazyFlyZoneToVector(crazyFlyZone* rect)
{
    crazyfly_zones.push_back(rect);
    emit numCrazyFlyZonesChanged(crazyfly_zones.size());
}

void myGraphicsScene::addTablePieceToVector(tablePiece* rect)
{
    table_pieces.push_back(rect);
    emit numTablePiecesChanged(table_pieces.size());
}

void myGraphicsScene::updateIndexesAndLabelsCrazyFlyZones()
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        crazyfly_zones[i]->setIndex(i);
        std::string str = std::to_string(i + 1);
        crazyfly_zones[i]->updateLabel(str.c_str());
        qDebug("reset Index %d and update label",i);

    }
}

void myGraphicsScene::changeModeTo(int next_mode)
{
    mode = next_mode;
}

int myGraphicsScene::getMode(void)
{
    return mode;
}

void myGraphicsScene::lockTablePieces(void)
{
    for(int i = 0; i < table_pieces.size(); i++)
    {
        table_pieces[i]->lock();
        table_pieces[i]->setDarkColor();
    }
}

void myGraphicsScene::unlockTablePieces(void)
{
    for(int i = 0; i < table_pieces.size(); i++)
    {
        table_pieces[i]->unlock();
        table_pieces[i]->setLightColor();
    }
}

void myGraphicsScene::lockCrazyFlyZones(void)
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        crazyfly_zones[i]->lock();
    }
}

void myGraphicsScene::unlockCrazyFlyZones(void)
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        crazyfly_zones[i]->unlock();
    }
}

void myGraphicsScene::setMode(int new_mode)
{
    switch(new_mode)
    {
        case mode_table:
        {
            lockCrazyFlyZones();
            unlockTablePieces();
            break;
        }
        case mode_crazyfly_zones:
        {
            lockTablePieces();
            unlockCrazyFlyZones();
            break;
        }
    }
    mode = new_mode;
    emit modeChanged(new_mode);
}

void myGraphicsScene::removeCrazyFlyZone(int index)
{
    this->removeItem(crazyfly_zones[index]);
    crazyfly_zones.erase(crazyfly_zones.begin() + index);
    qDebug("removed CFzone %d", index);
    updateIndexesAndLabelsCrazyFlyZones();
    emit numCrazyFlyZonesChanged(crazyfly_zones.size()); // for tab managing
}

void myGraphicsScene::removeTable()
{
    for(int i = 0; i < table_pieces.size(); i++)
    {
        this->removeItem(table_pieces[i]);
    }
    table_pieces.clear();
    emit numTablePiecesChanged(table_pieces.size());
}

void myGraphicsScene::removeTablePiece(int index)
{
    this->removeItem(table_pieces[index]);
    table_pieces.erase(table_pieces.begin() + index);
    qDebug("removed TabledPiece %d", index);
    emit numTablePiecesChanged(table_pieces.size());
}

void myGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
        if (mouseEvent->button() != Qt::LeftButton)
            return;

        switch(mode)
        {
            case mode_table:
            {
                if(startedRect)
                {
                    tmp_table_piece_item->setRect(tmp_table_piece_item->rect().normalized());
                    addTablePieceToVector(tmp_table_piece_item);
                    tmp_rect = 0;
                    startedRect = false;
                }
                break;
            }
            case mode_crazyfly_zones:
            {
                if(startedRect)
                {
                    tmp_crazyfly_zone_item->setRect(tmp_crazyfly_zone_item->rect().normalized());
                    addCrazyFlyZoneToVector(tmp_crazyfly_zone_item);
                    std::string str = std::to_string(crazyfly_zones.size());
                    tmp_crazyfly_zone_item->setLabel(str.c_str());
                    setSelectedCrazyFlyZone(crazyfly_zones.size() - 1); //select just created rectangle
                    tmp_rect = 0;
                    startedRect = false;
                }
                int selected_crazyfly_zone = checkSelectedCrazyFlyZone();
                if(selected_crazyfly_zone != -1)
                {
                    emit crazyFlyZoneSelected(selected_crazyfly_zone);
                }
                break;
            }
        }


    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
