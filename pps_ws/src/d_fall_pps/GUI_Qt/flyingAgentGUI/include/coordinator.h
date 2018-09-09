#ifndef COORDINATOR_H
#define COORDINATOR_H

#include "coordinatorrow.h"

#include <QWidget>
#include <QVector>

namespace Ui {
class Coordinator;
}

class Coordinator : public QWidget
{
    Q_OBJECT

public:
    explicit Coordinator(QWidget *parent = 0);
    ~Coordinator();

private:
    QVector<CoordinatorRow*> vector_of_coordinatorRows;

private slots:
    void on_refresh_button_clicked();

    void on_delete_button_clicked();

    void on_coordinate_all_checkBox_clicked();

private:
    Ui::Coordinator *ui;
};

#endif // COORDINATOR_H
