#ifndef MYGRAPHICSSCENE_H
#define MYGRAPHICSSCENE_H

#include <vector>

#include <QGraphicsScene>

#include "crazyFlyZone.h"

class QGraphicsSceneMouseEvent;
class QPointF;
class QColor;


class myGraphicsScene : public QGraphicsScene
{
    Q_OBJECT

public:

    explicit myGraphicsScene(QObject *parent = 0);
    std::vector<crazyFlyZone*> rectangles;

public slots:
    void removeRectangle(int index);
    void setSelectedRectangle(int index);

signals:
    void numRectanglesChanged(int newNum);
    void rectangleSelected(int index);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

    void keyPressEvent(QKeyEvent * keyEvent) override;

private:
    void addRectangleToVector(crazyFlyZone* rect);
    int checkSelectedRectangle();
    void updateIndexesAndLabels();

    QPen* pen;
    QBrush* brush;
    QRectF* tmp_rect;
    crazyFlyZone* tmp_rect_item;
    QPointF* p1;
    QPointF* p2;

    bool startedRect;
};

#endif
