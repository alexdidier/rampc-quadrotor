#ifndef MYGRAPHICSSCENE_H
#define MYGRAPHICSSCENE_H

#include <vector>

#include <QGraphicsScene>

class QGraphicsSceneMouseEvent;
class QPointF;
class QColor;


class myGraphicsScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit myGraphicsScene(QObject *parent = 0);
    std::vector<QGraphicsRectItem*> rectangles;

public slots:

signals:

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

private:
    QPen* pen;
    QBrush* brush;
    QRectF* tmp_rect;
    QGraphicsRectItem* tmp_rect_item;
    QPointF* p1;
    QPointF* p2;

    // bool firstClick;

    bool startedRect;
};

#endif
