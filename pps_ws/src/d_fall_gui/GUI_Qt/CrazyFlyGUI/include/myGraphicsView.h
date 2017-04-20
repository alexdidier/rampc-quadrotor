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
