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
    enum {mode_table, mode_crazyfly_zones};

    void removeTable();

public slots:
    void removeCrazyFlyZone(int index);
    void setSelectedCrazyFlyZone(int index);
    void changeModeTo(int next_mode);

signals:
    void numCrazyFlyZonesChanged(int newNum);
    void crazyFlyZoneSelected(int index);
    void modeChanged(int mode);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

    void keyPressEvent(QKeyEvent * keyEvent) override;

private:
    void lockTablePieces(void);
    void unlockTablePieces(void);
    void lockCrazyFlyZones(void);
    void unlockCrazyFlyZones(void);

    void addCrazyFlyZoneToVector(crazyFlyZone* rect);
    void addTablePieceToVector(tablePiece* rect);
    int checkSelectedCrazyFlyZone();
    void updateIndexesAndLabelsCrazyFlyZones();
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
};

#endif
