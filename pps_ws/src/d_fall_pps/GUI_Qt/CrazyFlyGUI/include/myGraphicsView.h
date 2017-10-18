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
