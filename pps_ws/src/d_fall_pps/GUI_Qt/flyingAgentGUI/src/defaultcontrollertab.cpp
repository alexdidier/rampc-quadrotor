#include "defaultcontrollertab.h"
#include "ui_defaultcontrollertab.h"

DefaultControllerTab::DefaultControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DefaultControllerTab)
{
    ui->setupUi(this);



#ifdef CATKIN_MAKE
    m_defaultSetpoint.x   = 0.0f;
    m_defaultSetpoint.y   = 0.0f;
    m_defaultSetpoint.z   = 0.5f;
    m_defaultSetpoint.yaw = 0.0f;
#endif

}

DefaultControllerTab::~DefaultControllerTab()
{
    delete ui;
}


void DefaultControllerTab::setMeasuredPose(QVector<float> measuredPose)
{
    // UPDATE THE MEASUREMENT COLUMN
    ui->lineEdit_measured_x    ->setText(QString::number( measuredPose[0] ));
    ui->lineEdit_measured_y    ->setText(QString::number( measuredPose[1] ));
    ui->lineEdit_measured_z    ->setText(QString::number( measuredPose[2] ));
    ui->lineEdit_measured_roll ->setText(QString::number( measuredPose[3] ));
    ui->lineEdit_measured_pitch->setText(QString::number( measuredPose[4] ));
    ui->lineEdit_measured_yaw  ->setText(QString::number( measuredPose[5] ));

    // GET THE CURRENT SETPOINT
    float curr_x_setpoint   = (ui->lineEdit_setpoint_current_x->text()  ).toFloat();;
    float curr_y_setpoint   = (ui->lineEdit_setpoint_current_y->text()  ).toFloat();;
    float curr_z_setpoint   = (ui->lineEdit_setpoint_current_z->text()  ).toFloat();;
    float curr_yaw_setpoint = (ui->lineEdit_setpoint_current_yaw->text()).toFloat();;

    // UPDATE THE ERROR COLUMN
    ui->lineEdit_error_x  ->setText(QString::number( measuredPose[0] - curr_x_setpoint   ));
    ui->lineEdit_error_y  ->setText(QString::number( measuredPose[1] - curr_y_setpoint   ));
    ui->lineEdit_error_z  ->setText(QString::number( measuredPose[2] - curr_z_setpoint   ));
    ui->lineEdit_error_yaw->setText(QString::number( measuredPose[5] - curr_yaw_setpoint ));
}



void publishSetpoint(float x, float y, float z, float yaw)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointHeader( msg );

    // Fill in the (x,y,z,yaw) values
    msg.x   = x;
    msg.y   = y;
    msg.z   = z;
    msg.yaw = yaw;

    // Publish the setpoint
    this->controllerSetpointPublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Published request for setpoint request for: [" << x << ", "<< y << ", "<< z << ", "<< yaw << "]");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[DEFAULT CONTROLLER GUI] would publish request for: [" << x << ", "<< y << ", "<< z << ", "<< yaw << "]";
#endif
}



void DefaultControllerTab::on_lineEdit_setpoint_new_x_returnPressed()
{
    ui->set_setpoint_button->animateClick();

#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[DEFAULT CONTROLLER TAB] return pressed for x setpoint";
#endif
}

void DefaultControllerTab::on_lineEdit_setpoint_new_y_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DefaultControllerTab::on_lineEdit_setpoint_new_z_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DefaultControllerTab::on_lineEdit_setpoint_new_yaw_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DefaultControllerTab::on_set_setpoint_button_clicked()
{
#ifdef CATKIN_MAKE
    // Initialise local variable for each of (x,y,z,yaw)
    float x = 0.0f, y = 0.0f, z = 0.0f, yaw = 0.0f;

    // Take the new value if available, otherwise use the old value
    // > For x
    if(!ui->lineEdit_setpoint_new_x->text().isEmpty())
        x = (ui->lineEdit_setpoint_new_x->text()).toFloat();
    else
        x = (ui->lineEdit_setpoint_current_x->text()).toFloat();
    // > For x
    if(!ui->lineEdit_setpoint_new_y->text().isEmpty())
        y = (ui->lineEdit_setpoint_new_y->text()).toFloat();
    else
        y = (ui->lineEdit_setpoint_current_y->text()).toFloat();
    // > For x
    if(!ui->lineEdit_setpoint_new_z->text().isEmpty())
        z = (ui->lineEdit_setpoint_new_z->text()).toFloat();
    else
        z = (ui->lineEdit_setpoint_current_z->text()).toFloat();
    // > For x
    if(!ui->lineEdit_setpoint_new_yaw->text().isEmpty())
        yaw = (ui->lineEdit_setpoint_new_yaw->text()).toFloat();
    else
        yaw = (ui->lineEdit_setpoint_current_yaw->text()).toFloat();

    // Call the function to publish the setpoint
    publishSetpoint(x,y,z,yaw);

#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[DEFAULT CONTROLLER TAB] set setpoint button clicked";
#endif
}

void DefaultControllerTab::on_default_setpoint_button_clicked()
{
#ifdef CATKIN_MAKE
    // Call the function to publish the setpoint
    publishSetpoint(m_defaultSetpoint.x, m_defaultSetpoint.y, m_defaultSetpoint.z, m_defaultSetpoint.yaw );
#endif
}

void DefaultControllerTab::on_x_increment_plus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_x->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat() + (ui->lineEdit_setpoint_increment_x->text()).toFloat()
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat()
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat()
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}

void DefaultControllerTab::on_x_increment_minus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_x->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat() - (ui->lineEdit_setpoint_increment_x->text()).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}

void DefaultControllerTab::on_y_increment_plus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_y->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat() + (ui->lineEdit_setpoint_increment_y->text()).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}

void DefaultControllerTab::on_y_increment_minus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_y->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat() - (ui->lineEdit_setpoint_increment_y->text()).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}

void DefaultControllerTab::on_z_increment_plus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_z->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat() + (ui->lineEdit_setpoint_increment_z->text()).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}

void DefaultControllerTab::on_z_increment_minus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_z->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat() - (ui->lineEdit_setpoint_increment_z->text()).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}

void DefaultControllerTab::on_yaw_increment_plus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_yaw->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat() + (ui->lineEdit_setpoint_increment_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}
void DefaultControllerTab::on_yaw_increment_minus_button_clicked()
{
#ifdef CATKIN_MAKE
    // Only need to do something if the field is not empty
    if(!ui->lineEdit_setpoint_increment_yaw->text().isEmpty())
    {
        // Call the function to publish the setpoint
        publishSetpoint(
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_y->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_z->text()  ).toFloat(),
                (ui->lineEdit_setpoint_current_yaw->text()).toFloat() - (ui->lineEdit_setpoint_increment_yaw->text()).toFloat()
            );
    }
    else
    {
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");

    }
#endif
}
