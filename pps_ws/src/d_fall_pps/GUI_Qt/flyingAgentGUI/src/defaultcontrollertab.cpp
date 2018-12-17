#include "defaultcontrollertab.h"
#include "ui_defaultcontrollertab.h"

DefaultControllerTab::DefaultControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DefaultControllerTab)
{
    ui->setupUi(this);

    // Hide the two red frames that are used to indcated
    // when pose data is occluded
    ui->red_frame_position_left->setVisible(false);
    ui->red_frame_position_right->setVisible(false);



#ifdef CATKIN_MAKE

    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[DEFAULT CONTROLLER TAB GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE THE COMMAND PUBLISHER
    requestSetpointChangePublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::SetpointWithHeader>("DefaultControllerService/RequestSetpointChange", 1);


    // Set the default setpoint
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


void DefaultControllerTab::setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded)
{
    if (!occluded)
    {
        // INITIALISE A STRING VARIABLE FOR ADDING THE "+"
        QString qstr = "";
        // UPDATE THE MEASUREMENT COLUMN
        if (x < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_x->setText(qstr + QString::number( x, 'f', 3));
        if (y < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_y->setText(qstr + QString::number( y, 'f', 3));
        if (z < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_z->setText(qstr + QString::number( z, 'f', 3));

        if (roll < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_roll->setText(qstr + QString::number( roll  * RAD2DEG, 'f', 1));
        if (pitch < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_pitch->setText(qstr + QString::number( pitch * RAD2DEG, 'f', 1));
        if (yaw < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_yaw->setText(qstr + QString::number( yaw   * RAD2DEG, 'f', 1));

        // GET THE CURRENT SETPOINT
        float error_x   = x   - (ui->lineEdit_setpoint_current_x->text()  ).toFloat();
        float error_y   = y   - (ui->lineEdit_setpoint_current_y->text()  ).toFloat();
        float error_z   = z   - (ui->lineEdit_setpoint_current_z->text()  ).toFloat();
        float error_yaw = yaw - (ui->lineEdit_setpoint_current_yaw->text()).toFloat();

        // UPDATE THE ERROR COLUMN
        if (error_x < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_x->setText(qstr + QString::number( error_x, 'f', 3));
        if (error_y < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_y->setText(qstr + QString::number( error_y, 'f', 3));
        if (error_z < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_z->setText(qstr + QString::number( error_z, 'f', 3));

        if (error_yaw < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_yaw->setText(qstr + QString::number( error_yaw * RAD2DEG, 'f', 1));

        // Ensure the red frames are not visible
        if ( ui->red_frame_position_left->isVisible() )
            ui->red_frame_position_left->setVisible(false);
        if ( ui->red_frame_position_right->isVisible() )
            ui->red_frame_position_right->setVisible(false);
    }
    else
    {
        // Make visible the red frames to indicate occluded
        if ( !(ui->red_frame_position_left->isVisible()) )
            ui->red_frame_position_left->setVisible(true);
        if ( !(ui->red_frame_position_right->isVisible()) )
            ui->red_frame_position_right->setVisible(true);
    }
}


void DefaultControllerTab::poseDataUnavailableSlot()
{
    ui->lineEdit_measured_x->setText("xx.xx");
    ui->lineEdit_measured_y->setText("xx.xx");
    ui->lineEdit_measured_z->setText("xx.xx");

    ui->lineEdit_measured_roll->setText("xx.xx");
    ui->lineEdit_measured_pitch->setText("xx.xx");
    ui->lineEdit_measured_yaw->setText("xx.xx");

    ui->lineEdit_error_x->setText("xx.xx");
    ui->lineEdit_error_y->setText("xx.xx");
    ui->lineEdit_error_z->setText("xx.xx");
    ui->lineEdit_error_yaw->setText("xx.xx");


}



void DefaultControllerTab::publishSetpoint(float x, float y, float z, float yaw)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    d_fall_pps::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the (x,y,z,yaw) values
    msg.x   = x;
    msg.y   = y;
    msg.z   = z;
    msg.yaw = yaw;

    // Publish the setpoint
    this->requestSetpointChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Published request for setpoint change to: [" << x << ", "<< y << ", "<< z << ", "<< yaw << "]");
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
                (ui->lineEdit_setpoint_current_x->text()  ).toFloat() + (ui->lineEdit_setpoint_increment_x->text()).toFloat(),
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





//    ----------------------------------------------------------------------------------
//    M   M   SSSS   GGG      H   H  EEEEE    A    DDDD   EEEEE  RRRR
//    MM MM  S      G   G     H   H  E       A A   D   D  E      R   R
//    M M M   SSS   G         HHHHH  EEE    A   A  D   D  EEE    RRRR
//    M   M      S  G   G     H   H  E      AAAAA  D   D  E      R   R
//    M   M  SSSS    GGGG     H   H  EEEEE  A   A  DDDD   EEEEE  R   R
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// Fill the head for a message
void DefaultControllerTab::fillSetpointMessageHeader( d_fall_pps::SetpointWithHeader & msg )
{
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            msg.shouldCheckForAgentID = false;
            break;
        }
        case TYPE_COORDINATOR:
        {
            // Lock the mutex
            m_agentIDs_toCoordinate_mutex.lock();
            // Add the "coordinate all" flag
            msg.shouldCheckForAgentID = !(m_shouldCoordinateAll);
            // Add the agent IDs if necessary
            if (!m_shouldCoordinateAll)
            {
                for ( int irow = 0 ; irow < m_vector_of_agentIDs_toCoordinate.size() ; irow++ )
                {
                    msg.agentIDs.push_back( m_vector_of_agentIDs_toCoordinate[irow] );
                }
            }
            // Unlock the mutex
            m_agentIDs_toCoordinate_mutex.unlock();
            break;
        }

        default:
        {
            msg.shouldCheckForAgentID = true;
            ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    } 
}
#endif





//    ----------------------------------------------------------------------------------
//    III  DDDD       &&&      TTTTT  Y   Y  PPPP   EEEEE
//     I   D   D     &           T     Y Y   P   P  E
//     I   D   D      &          T      Y    PPPP   EEE
//     I   D   D     & & &       T      Y    P      E
//    III  DDDD       &&&        T      Y    P      EEEEE
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
bool DefaultControllerTab::getTypeAndIDParameters()
{
    // Initialise the return variable as success
    bool return_was_successful = true;

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the value of the "type" parameter into a local string variable
    std::string type_string;
    if(!nodeHandle.getParam("type", type_string))
    {
        // Throw an error if the agent ID parameter could not be obtained
        ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Failed to get type");
    }

    // Set the "m_type" class variable based on this string loaded
    if ((!type_string.compare("coordinator")))
    {
        m_type = TYPE_COORDINATOR;
    }
    else if ((!type_string.compare("agent")))
    {
        m_type = TYPE_AGENT;
    }
    else
    {
        // Set "m_type" to the value indicating that it is invlid
        m_type = TYPE_INVALID;
        return_was_successful = false;
        ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] The 'type' parameter retrieved was not recognised.");
    }


    // Construct the string to the namespace of this Paramater Service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            // Get the value of the "agentID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("agentID", m_ID))
            {
                // Throw an error if the agent ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[DEFAULT CONTROLLER TAB GUI] Is of type AGENT with ID = " << m_ID);
            }
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            // Get the value of the "coordID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("coordID", m_ID))
            {
                // Throw an error if the coord ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[DEFAULT CONTROLLER TAB GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
