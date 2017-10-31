//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    Our reimplementation of QGraphicsView
//
//    ----------------------------------------------------------------------------------


#ifndef MYGRAPHICSVIEW_H
#define MYGRAPHICSVIEW_H

#include <vector>

#include <QGraphicsView>
#include <QWheelEvent>

class myGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:

    explicit myGraphicsView(QWidget *parent = 0);

public slots:

signals:

protected:
    void mousePressEvent(QMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QMouseEvent *mouseEvent) override;

    virtual void wheelEvent(QWheelEvent* event) override; // TODO: right now, do it in the whole MainGUIWindow. Afterwards maybe do this only in the QGraphicsScene (need to do own class)

private:

    bool translation_mode;
    qreal translate_dx;
    qreal translate_dy;
    QPointF* tmp_point;
};

#endif
