//    Our reimplementation of QGraphicsView
//
//    Copyright (C) 2017  Angel Romero
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
    if (mouseEvent->button() == Qt::RightButton)
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
