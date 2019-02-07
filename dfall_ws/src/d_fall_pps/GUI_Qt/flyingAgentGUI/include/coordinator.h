#ifndef COORDINATOR_H
#define COORDINATOR_H

#include "coordinatorrow.h"

#include <QWidget>
#include <QVector>
#include <regex>

#include <QTextStream>

namespace Ui {
class Coordinator;
}

class Coordinator : public QWidget
{
    Q_OBJECT

public:
    explicit Coordinator(QWidget *parent = 0);
    ~Coordinator();


public slots:
    void setShouldCoordinateThisAgent(int agentID , bool shouldCoordinate);


signals:
    void agentIDsToCoordinateChanged(QVector<int> agentIDs , bool shouldCoordinateAll);


private:
    QVector<CoordinatorRow*> vector_of_coordinatorRows;

    QVector<bool> vector_of_shouldCoordinate_perRow;

    QVector<int> vector_of_agentID_perRow;

    int level_of_detail_to_display = 1;

    void remove_all_entries_from_vector_of_coordinatorRows();

    void apply_level_of_detail_to_all_entries(int level);

private slots:
    void on_refresh_button_clicked();

    void on_toggle_details_button_clicked();

    void on_delete_button_clicked();

    void on_coordinate_all_checkBox_clicked();

    void emit_signal_with_agentIDs_toCoordinate();

private:
    Ui::Coordinator *ui;
};

#endif // COORDINATOR_H
