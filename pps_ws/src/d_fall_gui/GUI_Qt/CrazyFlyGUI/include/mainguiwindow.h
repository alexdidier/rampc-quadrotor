#ifndef MAINGUIWINDOW_H
#define MAINGUIWINDOW_H

// The #define CATKIN_MAKE comes from cmake file
#include <QMainWindow>
#include <QTimer>
#include <QGridLayout>
#include <QGraphicsRectItem>

#ifdef CATKIN_MAKE
#include "ros/callback_queue.h"
#include "ros/ros.h"
#endif

#include "ui_mainguiwindow.h"
#include "myGraphicsScene.h"
#include "globalDefinitions.h"

#include "marker.h"             // temporal, just to check


namespace Ui {
class MainGUIWindow;
}


#ifdef CATKIN_MAKE
struct setpoint
{
    double x;
    double y;
    double z;
    double yaw;

};

class CSetpointQueue
{
public:
    CSetpointQueue();
    void insert(setpoint newElem);
    setpoint getNext();
    void print();

private:
    struct QueueElem
    {
        QueueElem(setpoint newElem) {elem.x=newElem.x; elem.y=newElem.y; elem.z=newElem.z; elem.yaw=newElem.yaw; next=NULL;}
        setpoint elem;
        QueueElem* next;
    };

    QueueElem* startElem;
    QueueElem* currElem;
    QueueElem* lastElem;
};
#endif

class MainGUIWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainGUIWindow(QWidget *parent = 0);
    ~MainGUIWindow();

public slots:

private slots:
    #ifdef CATKIN_MAKE
    #endif

    void set_tabs(int n);
    void transitionToMode(int mode);
    void on_removeTable_clicked();

    void on_radioButton_table_mode_toggled(bool checked);

    void on_radioButton_crazyfly_zones_mode_toggled(bool checked);
    void handleTablePiecesNumChanged(int newNum);

    void on_radioButton_lock_mode_toggled(bool checked);

    void on_checkBox_grid_toggled(bool checked);

    void on_checkBox_table_toggled(bool checked);

    void on_checkBox_crazyfly_zones_toggled(bool checked);

    void on_tabWidget_currentChanged(int index);

    void centerViewIndex(int index);

    void on_pushButton_fitAll_clicked();

    void on_checkBox_vicon_markers_toggled(bool checked);

    void on_checkBox_vicon_highlight_markers_toggled(bool checked);

private:

    Ui::MainGUIWindow *ui;
    myGraphicsScene* scene;
    void _init();

    Marker* marker;

    #ifdef CATKIN_MAKE

    #endif
};



#endif // MAINGUIWINDOW_H
