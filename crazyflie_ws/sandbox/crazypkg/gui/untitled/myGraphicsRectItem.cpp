#include "myGraphicsRectItem.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include <QApplication>
#include <QVariant>

myGraphicsRectItem::myGraphicsRectItem(const QRectF & rect, QGraphicsItem * parent)
    : QGraphicsRectItem(rect, parent)
{
    this->setFlag(QGraphicsItem::ItemIsSelectable);
    this->setFlag(QGraphicsItem::ItemIsMovable);
    pen = new QPen(Qt::red);
    brush = new QBrush(Qt::red);

    tmp_rect = 0;
    _grabbers_created = false;
    // this->setAcceptHoverEvents(true);
}

void myGraphicsRectItem::deleteGrabbers()
{
    _bottomLeft_corner->setParentItem(NULL);
    _topLeft_corner->setParentItem(NULL);
    _topRight_corner->setParentItem(NULL);
    _bottomRight_corner->setParentItem(NULL);

    delete _bottomLeft_corner;
    delete _topLeft_corner;
    delete _topRight_corner;
    delete _bottomRight_corner;
    _grabbers_created = false;
}

QVariant myGraphicsRectItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == QGraphicsItem::ItemSelectedChange)
    {
        if (value == true)
        {
            qDebug("selected!");
            createGrabbers();
        }
        else
        {
            qDebug("unselected!");
            deleteGrabbers();
        }
    }

    return QGraphicsItem::itemChange(change, value);
}

bool myGraphicsRectItem::grabbersAreCreated()
{
    return _grabbers_created;
}

void myGraphicsRectItem::setCornerPositions()
{
    QRectF rect = this->rect();

    _bottomLeft_corner->setPos(rect.bottomLeft().x(), rect.bottomLeft().y() - _bottomLeft_corner->getHeight());
    _topLeft_corner->setPos(rect.topLeft().x(), rect.topLeft().y());
    _topRight_corner->setPos(rect.topRight().x() - _topRight_corner->getWidth(), rect.topRight().y());
    _bottomRight_corner->setPos(rect.bottomRight().x() - _bottomRight_corner->getWidth(), rect.bottomRight().y() - _bottomRight_corner->getHeight());
}

void myGraphicsRectItem::createGrabbers()
{
    if(!grabbersAreCreated())
    {
        _bottomLeft_corner = new CornerGrabber(this, CornerGrabber::bottomLeft);
        _topLeft_corner = new CornerGrabber(this, CornerGrabber::topLeft);
        _topRight_corner = new CornerGrabber(this, CornerGrabber::topRight);
        _bottomRight_corner = new CornerGrabber(this, CornerGrabber::topRight);


        _bottomLeft_corner->installSceneEventFilter(this);
        _topLeft_corner->installSceneEventFilter(this);
        _topRight_corner->installSceneEventFilter(this);
        _bottomRight_corner->installSceneEventFilter(this);

        _grabbers_created = true;
        setCornerPositions();
    }
}

int myGraphicsRectItem::checkCornerGrabbers()
{

    // we return the first active found. Hopefully we never have two at the same time.
    if(_bottomLeft_corner->isActive())
        return CornerGrabber::bottomLeft;
    else if(_topLeft_corner->isActive())
        return CornerGrabber::topLeft;
    else if (_topRight_corner->isActive())
        return CornerGrabber::topRight;
    else if( _bottomRight_corner->isActive())
        return CornerGrabber::bottomRight;
    else
        return 20;
}

void myGraphicsRectItem::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;

    createGrabbers();           //This is just in case they have not been created by now.

    switch(checkCornerGrabbers())
    {
        case CornerGrabber::bottomLeft:
            qDebug("bottomLeft");
            mouseEvent->setAccepted(true);
            break;
        case CornerGrabber::topLeft:
            qDebug("topLeft");
            mouseEvent->setAccepted(true);
            break;
        case CornerGrabber::topRight:
            qDebug("topRight");
            mouseEvent->setAccepted(true);
            break;
        case CornerGrabber::bottomRight:
            qDebug("bottomRight");
            mouseEvent->setAccepted(true);
            break;
        default:
            qDebug("No corner");
            // Normal case here, mouse not in corner grabbers
            QGraphicsRectItem::mousePressEvent(mouseEvent);
            break;
    }
    // TODO: check if over handlers, if, resize mode
}

void myGraphicsRectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    this->prepareGeometryChange();
    QGraphicsRectItem::mouseMoveEvent(mouseEvent);
}

void myGraphicsRectItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{

    if (mouseEvent->button() != Qt::LeftButton)
        return;
    // TODO: stop resize mode
    QGraphicsRectItem::mouseReleaseEvent(mouseEvent);
}
