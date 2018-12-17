#ifndef STUDENTCONTROLLERTAB_H
#define STUDENTCONTROLLERTAB_H

#include <QWidget>
#include <QVector>
#include <QTextStream>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
//#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
//#include "d_fall_pps/IntWithHeader.h"
#include "d_fall_pps/SetpointWithHeader.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace d_fall_pps;

#else
// Include the shared definitions
//#include "include/Constants_for_Qt_compile.h"

#endif


namespace Ui {
class StudentControllerTab;
}

class StudentControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit StudentControllerTab(QWidget *parent = 0);
    ~StudentControllerTab();


public slots:
    void setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded);


private:
    Ui::StudentControllerTab *ui;
};

#endif // STUDENTCONTROLLERTAB_H
