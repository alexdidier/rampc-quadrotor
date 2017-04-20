#include "../../include/cornergrabber.h"

CornerGrabber::CornerGrabber(QGraphicsItem *parent,  int corner) :
    QGraphicsItem(parent),
    _outterborderColor(Qt::black),
    _outterborderPen(),
    _width(GRABBER_WIDTH),
    _height(GRABBER_HEIGHT),
    _corner(corner),
    _is_active(false)
{
    setParentItem(parent);
    _outterborderPen.setWidth(2);
    _outterborderPen.setColor(_outterborderColor);
    this->setAcceptHoverEvents(true);
    this->setFlag(QGraphicsItem::ItemIgnoresTransformations);
}


qreal CornerGrabber::getHeight()
{
    return _height;
}

qreal CornerGrabber::getWidth()
{
    return _width;
}

int CornerGrabber::getCorner()
{
    return _corner;
}

bool CornerGrabber::isActive()
{
    return _is_active;
}


// we have to implement the mouse events to keep the linker happy,
// but just set accepted to false since are not actually handling them

void CornerGrabber::mouseMoveEvent(QGraphicsSceneDragDropEvent *event)
{
    event->setAccepted(false);
}

void CornerGrabber::mousePressEvent(QGraphicsSceneDragDropEvent *event)
{
    event->setAccepted(false);
}

void CornerGrabber::mouseReleaseEvent ( QGraphicsSceneMouseEvent * event )
{
    event->setAccepted(false);
}

void CornerGrabber::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
    event->setAccepted(false);
}

void CornerGrabber::mouseMoveEvent ( QGraphicsSceneMouseEvent * event )
{
    event->setAccepted(false);
}


// change the color on hover events to indicate to the use the object has
// been captured by the mouse

void CornerGrabber::hoverLeaveEvent ( QGraphicsSceneHoverEvent * )
{
    _outterborderColor = Qt::black;
    _is_active = false;
    this->update(0,0,_width,_height);
}

void CornerGrabber::hoverEnterEvent ( QGraphicsSceneHoverEvent * )
{
    _outterborderColor = Qt::red;
    _is_active = true;
    this->update(0,0,_width,_height);
}

QRectF CornerGrabber::boundingRect() const
{
    QRectF bounding_rect = this->rect();
    return bounding_rect;
}

QRectF CornerGrabber::rect() const
{
    return _rect;
}

void CornerGrabber::setRect(const QRectF & rectangle)
{
    _rect = rectangle;
}

void CornerGrabber::paint (QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{

    // fill the box with solid color, use sharp corners
    prepareGeometryChange();
    _outterborderPen.setCapStyle(Qt::SquareCap);
    _outterborderPen.setStyle(Qt::SolidLine);
    painter->setPen(_outterborderPen);
    QRectF rect = createRect();
    this->setRect(rect);
    QBrush brush (Qt::SolidPattern);
    brush.setColor (_outterborderColor);
    painter->fillRect(_rect,brush);

}

QRectF CornerGrabber::createRect()
{
    QPointF topLeft(0,0);
    QPointF bottomRight(_width, _height);
    QRectF rect(topLeft, bottomRight);

    switch(_corner)
    {
        case CornerGrabber::bottomLeft:
        {
            QPointF move_to(0,0);
            rect.moveBottomLeft(move_to);
            return rect;
            break;
        }
        case CornerGrabber::topLeft:
        {
            return rect;
            break;
        }
        case CornerGrabber::topRight:
        {
            QPointF move_to(0,0);
            rect.moveTopRight(move_to);
            return rect;
            break;
        }
        case CornerGrabber::bottomRight:
        {
            QPointF move_to(0,0);
            rect.moveBottomRight(move_to);
            return rect;
            break;
        }
        default:
            break;
    }
}
