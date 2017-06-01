#include "myGraphicsScene.h"
#include "globalDefinitions.h"

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
    setGrid(true);
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
                        removeCrazyFlyZone(crazyfly_zones[i]->getIndex());
                    }
                }
            }
            break;
        }
        case mode_locked:
        {
            // nothing so far
            break;
        }
        default:
            break;
    }

    QGraphicsScene::keyPressEvent(keyEvent);
}

void myGraphicsScene::setSelectedCrazyFlyZone(int index)
{
    for(unsigned int i = 0; i < crazyfly_zones.size(); i++)
    {
        qDebug("index: %d", index);
        qDebug("i: %d", i);
        qDebug("getIndex: %d", crazyfly_zones[i]->getIndex());
        crazyfly_zones[i]->setSelected(false);
        if(index == crazyfly_zones[i]->getIndex())
        {
            crazyfly_zones[i]->setSelected(true);
        }
    }
}

int myGraphicsScene::checkSelectedCrazyFlyZone()
{
    for(unsigned int i = 0; i < crazyfly_zones.size(); i++)
    {
        if(crazyfly_zones[i]->isSelected())
        {
            qDebug("rectangle selected index = %d", crazyfly_zones[i]->getIndex());
            return crazyfly_zones[i]->getIndex();
        }
    }
    return -1;
}

void myGraphicsScene::addCFZone(QRectF rect, int index)
{
    tmp_crazyfly_zone_item = new crazyFlyZone(rect, index);
    addItem(tmp_crazyfly_zone_item);
    tmp_crazyfly_zone_item->setRect(tmp_crazyfly_zone_item->rect().normalized());
    addCrazyFlyZoneToVector(tmp_crazyfly_zone_item);
    std::string str = std::to_string(tmp_crazyfly_zone_item->getIndex() + 1);
    tmp_crazyfly_zone_item->setLabel(str.c_str());
    // setSelectedCrazyFlyZone(crazyfly_zones.size() - 1); //select just created rectangle
}

void myGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;
    qDebug("Mouse Position in world coord: %d, %d", (int)mapFromSceneToWorld((mouseEvent->scenePos()).toPoint()).x(), (int)mapFromSceneToWorld((mouseEvent->scenePos()).toPoint()).y());
    if(Qt::ControlModifier == QApplication::keyboardModifiers())
    {
        switch(mode)
        {
            case mode_table:
            {
                startedRect = true;
                p1 = new QPointF(mouseEvent->scenePos());
                tmp_rect = new QRectF(*p1, *p1);
                tmp_table_piece_item = new tablePiece(*tmp_rect);
                addItem(tmp_table_piece_item);
                break;
            }
            case mode_crazyfly_zones:
            {
                startedRect = true;
                p1 = new QPointF(mouseEvent->scenePos());
                tmp_rect = new QRectF(*p1, *p1);
                // look for first available index here
                bool first_available_value_found = false;
                int first_available_value = 0;
                while(!first_available_value_found)
                {
                    if(crazyfly_zones.size() == 0)
                    {
                        first_available_value_found = true;
                        break;
                    }
                    else        // we enter the for loop for sure
                    {
                        bool index_is_present = false;
                        for(int i = 0; i < crazyfly_zones.size(); i++)
                        {
                            if(crazyfly_zones[i]->getIndex() == first_available_value)
                            {
                                index_is_present = true;
                                break; // break because we finish searching here
                            }
                        }
                        if(!index_is_present)
                        {
                            first_available_value_found = true; // finish loop here, found value
                            break;
                        }
                    }

                    first_available_value++; //first iteration 0
                }
                qDebug("---------------------------first available value: %d", first_available_value);
                tmp_crazyfly_zone_item = new crazyFlyZone(*tmp_rect, first_available_value);
                addItem(tmp_crazyfly_zone_item);
                break;
            }
            case mode_locked:
            {
                // do nothing so far
                startedRect = false;
                break;
            }
            default:
                break;
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
    int size = crazyfly_zones.size();
    qDebug("size crazyflyzones %d", size);
    emit numCrazyFlyZonesChanged(rect->getIndex());
}

void myGraphicsScene::addTablePieceToVector(tablePiece* rect)
{
    table_pieces.push_back(rect);
    emit numTablePiecesChanged(table_pieces.size());
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
        case mode_locked:
        {
            // TODO: define locked mode. Do not allow to create anything, change some color of something to state that we are in that mode
            lockTablePieces();
            lockCrazyFlyZones();
            break;
        }
    }
    mode = new_mode;
    emit modeChanged(new_mode);
}

void myGraphicsScene::setGrid(bool enable)
{
    grid_enable = enable;
    update();
}

void myGraphicsScene::hideTable()
{
    for(int i = 0; i < table_pieces.size(); i++)
    {
        this->removeItem(table_pieces[i]);
    }
}

void myGraphicsScene::showTable()
{
    for(int i = 0; i < table_pieces.size(); i++)
    {
        this->addItem(table_pieces[i]);
    }
}

void myGraphicsScene::hideCrazyFlyZones()
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        this->removeItem(crazyfly_zones[i]);
    }
}

void myGraphicsScene::showCrazyFlyZones()
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        this->addItem(crazyfly_zones[i]);
    }
}

QRectF myGraphicsScene::getRectFCrazyFlyZone(int index)
{
    QRectF rect(crazyfly_zones[index]->sceneBoundingRect());
    return rect;
}

void myGraphicsScene::removeCrazyFlyZone(int cf_zone_index)
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        if(crazyfly_zones[i]->getIndex() == cf_zone_index)
        {
            this->removeItem(crazyfly_zones[i]);
            crazyfly_zones.erase(crazyfly_zones.begin() + i);
            qDebug("removed CFzone %d", cf_zone_index);
            emit numCrazyFlyZonesChanged(cf_zone_index); // for tab managing
            break;
        }
    }
}

void myGraphicsScene::removeAllCFZones()
{
    for(int i = 0; i < crazyfly_zones.size(); i++)
    {
        this->removeItem(crazyfly_zones[i]);
    }
    crazyfly_zones.clear();
    emit numCrazyFlyZonesChanged(0);
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

void myGraphicsScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    // add coordinate origin. X is red, Y is Green

    // X axis
    painter->setPen(QPen(QColor(0xFF, 0, 0, 0xFF), 0));
    painter->drawLine(QLineF(rect.left(), 0, rect.right(), 0));

    // Y axis
    painter->setPen(QPen(QColor(0, 0xFF, 0, 0xFF), 0));
    painter->drawLine(QLineF(0, rect.top(), 0, rect.bottom()));

    if(grid_enable)
    {
        const int gridSize = 1 * FROM_METERS_TO_UNITS;

        qreal left = int(rect.left()) - (int(rect.left()) % gridSize);
        qreal top = int(rect.top()) - (int(rect.top()) % gridSize);

        QVarLengthArray<QLineF, 100> lines;

        for (qreal x = left; x < rect.right(); x += gridSize)
            if(x != 0)
            lines.append(QLineF(x, rect.top(), x, rect.bottom()));
        for (qreal y = top; y < rect.bottom(); y += gridSize)
            if(y != 0)
            lines.append(QLineF(rect.left(), y, rect.right(), y));

        // qDebug() << lines.size();

        painter->setPen(QPen(QColor(0, 0, 0, 0x40), 0));
        painter->drawLines(lines.data(), lines.size());
    }
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
                    std::string str = std::to_string(tmp_crazyfly_zone_item->getIndex() + 1);
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
            case mode_locked:
            {
                // Do nothing so far..
                break;
            }
            default:
                break;
        }


    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}


QPointF myGraphicsScene::mapFromWorldToScene(QPointF point)
{
    QPointF new_point(point.x(), -1 * point.y());
    return new_point;
}

QPointF myGraphicsScene::mapFromSceneToWorld(QPointF point)
{
    QPointF new_point(point.x(), -1 * point.y());
    return new_point;
}
