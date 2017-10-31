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
//    Our reimplementation of QGraphicsScene
//
//    ----------------------------------------------------------------------------------


#ifndef MYGRAPHICSSCENE_H
#define MYGRAPHICSSCENE_H

#include <vector>

#include <QGraphicsScene>

#include "crazyFlyZone.h"
#include "tablePiece.h"


class QGraphicsSceneMouseEvent;
class QPointF;
class QColor;


class myGraphicsScene : public QGraphicsScene
{
    Q_OBJECT

public:

    explicit myGraphicsScene(QObject *parent = 0);
    std::vector<crazyFlyZone*> crazyfly_zones;
    std::vector<tablePiece*> table_pieces;
    int getMode();

    void setMode(int new_mode);
    void setGrid(bool enable);

    void hideTable();
    void showTable();

    void hideCrazyFlyZones();
    void showCrazyFlyZones();

    QRectF getRectFCrazyFlyZone(int index);

    // Y axis in QGraphicsView is inverted wrt normal axis. This functions map from one way to the other
    QPointF mapFromWorldToScene(QPointF point);
    QPointF mapFromSceneToWorld(QPointF point);

    enum {mode_table, mode_crazyfly_zones, mode_locked};

    void removeTable();

    void addCFZone(QRectF rect, int index);

    void removeAllCFZones();

public slots:
    void removeCrazyFlyZone(int cf_zone_index);
    void setSelectedCrazyFlyZone(int index);
    void changeModeTo(int next_mode);

signals:
    void numCrazyFlyZonesChanged(int newNum);
    void crazyFlyZoneSelected(int index);
    void modeChanged(int mode);
    void numTablePiecesChanged(int newNum);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

    void keyPressEvent(QKeyEvent * keyEvent) override;
    void drawBackground(QPainter *painter, const QRectF &rect);

private:
    void lockTablePieces(void);
    void unlockTablePieces(void);
    void lockCrazyFlyZones(void);
    void unlockCrazyFlyZones(void);

    void addCrazyFlyZoneToVector(crazyFlyZone* rect);
    void addTablePieceToVector(tablePiece* rect);
    int checkSelectedCrazyFlyZone();
    void removeTablePiece(int index);

    QPen* pen;
    QBrush* brush;
    QRectF* tmp_rect;
    crazyFlyZone* tmp_crazyfly_zone_item;
    tablePiece* tmp_table_piece_item;
    QPointF* p1;
    QPointF* p2;


    bool startedRect;
    int mode;
    bool grid_enable;
};

#endif
