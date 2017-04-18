#include "myGraphicsView.h"

#include <QApplication>
#include <QMouseEvent>

myGraphicsView::myGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
{
    translation_mode = false;
}


void myGraphicsView::wheelEvent(QWheelEvent *event)
{

    if(Qt::ControlModifier == QApplication::keyboardModifiers())
    {
        this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        // Scale the view / do the zoom
        double scaleFactor = 1.15;
        if(event->delta() > 0) {
            // Zoom in
            this->scale(scaleFactor, scaleFactor);

        } else {
            // Zooming out
            this->scale(1.0 / scaleFactor, 1.0 / scaleFactor);
        }
    }
    else
    {
        QGraphicsView::wheelEvent(event); // dont propagate if we are zooming. If we propagate, we will also scroll
    }
}

void myGraphicsView::mousePressEvent(QMouseEvent *mouseEvent)
{
    if (mouseEvent->button() == Qt::MiddleButton)
    {
        translation_mode = true;
        tmp_point = new QPointF(mouseEvent->localPos());
    }
    QGraphicsView::mousePressEvent(mouseEvent);
}

void myGraphicsView::mouseMoveEvent(QMouseEvent *mouseEvent)
{
    if(translation_mode)
    {
        translate_dx = mouseEvent->localPos().x() - tmp_point->x();
        translate_dy = mouseEvent->localPos().y() - tmp_point->y();
        this->translate(translate_dx, translate_dy);
    }
    QGraphicsView::mouseMoveEvent(mouseEvent);
}


void myGraphicsView::mouseReleaseEvent(QMouseEvent *mouseEvent)
{
    translation_mode = false;
    QGraphicsView::mouseReleaseEvent(mouseEvent);
}
