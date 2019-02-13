//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
//
//    This file is part of D-FaLL-System.
//
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    The GUI for the Default Controller
//
//    ----------------------------------------------------------------------------------





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

    // Make the current state label blank
    ui->label_current_state->setText("");



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

    // CREATE THE REQUEST SETPOINT CHANGE PUBLISHER
    requestSetpointChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::SetpointWithHeader>("DefaultControllerService/RequestSetpointChange", 1);

    // SUBSCRIBE TO SETPOINT CHANGES
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        setpointChangedSubscriber = nodeHandle_for_this_gui.subscribe("DefaultControllerService/SetpointChanged", 1, &DefaultControllerTab::setpointChangedCallback, this);
    }

    // GET THE CURRENT SETPOINT
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = nodeHandle_for_this_gui.serviceClient<dfall_pkg::GetSetpointService>("DefaultControllerService/GetCurrentSetpoint", false);
        dfall_pkg::GetSetpointService getSetpointCall;
        getSetpointCall.request.data = 0;
        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(2.0));
        if(getCurrentSetpointServiceClient.call(getSetpointCall))
        {
            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
        }
        else
        {
            // Inform the user
            ROS_INFO("[DEFAULT CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }
    }

#endif

}

DefaultControllerTab::~DefaultControllerTab()
{
    delete ui;
}





//    ----------------------------------------------------------------------------------
//    PPPP    OOO    SSSS  EEEEE     DDDD     A    TTTTT    A
//    P   P  O   O  S      E         D   D   A A     T     A A
//    PPPP   O   O   SSS   EEE       D   D  A   A    T    A   A
//    P      O   O      S  E         D   D  AAAAA    T    AAAAA
//    P       OOO   SSSS   EEEEE     DDDD   A   A    T    A   A
//    ----------------------------------------------------------------------------------


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
        ui->lineEdit_measured_roll->setText(qstr + QString::number( roll * RAD2DEG, 'f', 1));
        if (pitch < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_pitch->setText(qstr + QString::number( pitch * RAD2DEG, 'f', 1));
        if (yaw < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_yaw->setText(qstr + QString::number( yaw * RAD2DEG, 'f', 1));

        // GET THE CURRENT SETPOINT ERROR
        float error_x   = x   - (ui->lineEdit_setpoint_current_x->text()  ).toFloat();
        float error_y   = y   - (ui->lineEdit_setpoint_current_y->text()  ).toFloat();
        float error_z   = z   - (ui->lineEdit_setpoint_current_z->text()  ).toFloat();

        float error_yaw_deg = yaw * RAD2DEG - (ui->lineEdit_setpoint_current_yaw->text()).toFloat();

        // UPDATE THE ERROR COLUMN
        if (error_x < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_x->setText(qstr + QString::number( error_x, 'f', 3));
        if (error_y < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_y->setText(qstr + QString::number( error_y, 'f', 3));
        if (error_z < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_z->setText(qstr + QString::number( error_z, 'f', 3));

        if (error_yaw_deg < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_yaw->setText(qstr + QString::number( error_yaw_deg , 'f', 1));

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










//    ----------------------------------------------------------------------------------
//     SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    S      E        T    P   P  O   O   I   NN  N    T
//     SSS   EEE      T    PPPP   O   O   I   N N N    T
//        S  E        T    P      O   O   I   N  NN    T
//    SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC  H   H    A    N   N   GGGG  EEEEE  DDDD
//    C      H   H   A A   NN  N  G      E      D   D
//    C      HHHHH  A   A  N N N  G      EEE    D   D
//    C      H   H  AAAAA  N  NN  G   G  E      D   D
//     CCCC  H   H  A   A  N   N   GGGG  EEEEE  DDDD
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


#ifdef CATKIN_MAKE
void DefaultControllerTab::setpointChangedCallback(const dfall_pkg::SetpointWithHeader& newSetpoint)
{
    // INITIALISE A STRING VARIABLE FOR ADDING THE "+"
    QString qstr = "";

    // EXTRACT THE SETPOINT
    float x = newSetpoint.x;
    float y = newSetpoint.y;
    float z = newSetpoint.z;
    float yaw = newSetpoint.yaw;

    // UPDATE THE SETPOINT COLUMN
    if (x < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_setpoint_current_x->setText(qstr + QString::number( x, 'f', 3));
    if (y < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_setpoint_current_y->setText(qstr + QString::number( y, 'f', 3));
    if (z < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_setpoint_current_z->setText(qstr + QString::number( z, 'f', 3));

    if (yaw < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_setpoint_current_yaw->setText(qstr + QString::number( yaw * RAD2DEG, 'f', 3));

    // UPDATE THE CURRENT STATE LABEL
    int current_state = newSetpoint.buttonID;
    m_label_current_state_mutex.lock();
    switch (current_state)
    {
    case DEFAULT_CONTROLLER_STATE_NORMAL:
    {
        ui->label_current_state->setText("normal");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_STANDBY:
    {
        ui->label_current_state->setText("standby");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_TAKEOFF_SPIN_MOTORS:
    {
        ui->label_current_state->setText("Take-off, spinning motors");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_TAKEOFF_MOVE_UP:
    {
        ui->label_current_state->setText("Take-off, moving up");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_TAKEOFF_GOTO_SETPOINT:
    {
        ui->label_current_state->setText("Take-off, goto setpoint");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_TAKEOFF_INTEGRATOR_ON:
    {
        ui->label_current_state->setText("Take-off, integrator on");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_LANDING_MOVE_DOWN:
    {
        ui->label_current_state->setText("Landing, move down");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
    {
        ui->label_current_state->setText("Landing, spinning motors");
        break;
    }
    case DEFAULT_CONTROLLER_STATE_UNKNOWN:
    {
        ui->label_current_state->setText("Unknown");
        break;
    }
    default:
    {
        ui->label_current_state->setText("Not recognised");
        break;
    }
    }
    m_label_current_state_mutex.unlock();
}
#endif










//    ----------------------------------------------------------------------------------
//    RRRR   EEEEE   QQQ   U   U  EEEEE   SSSS  TTTTT     N   N  EEEEE  W     W
//    R   R  E      Q   Q  U   U  E      S        T       NN  N  E      W     W
//    RRRR   EEE    Q   Q  U   U  EEE     SSS     T       N N N  EEE    W     W 
//    R   R  E      Q  Q   U   U  E          S    T       N  NN  E       W W W
//    R   R  EEEEE   QQ Q   UUU   EEEEE  SSSS     T       N   N  EEEEE    W W
//
//     SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    S      E        T    P   P  O   O   I   NN  N    T
//     SSS   EEE      T    PPPP   O   O   I   N N N    T
//        S  E        T    P      O   O   I   N  NN    T
//    SSSS   EEEEE    T    P       OOO   III  N   N    T
//    ----------------------------------------------------------------------------------


void DefaultControllerTab::publishSetpoint(float x, float y, float z, float yaw)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the (x,y,z,yaw) values
    msg.x   = x;
    msg.y   = y;
    msg.z   = z;
    msg.yaw = yaw * DEG2RAD;

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

#ifdef CATKIN_MAKE
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
    // Publish this as a blank setpoint with the 
    // "buttonID" field set appropriately

    // Initialise the message as a local variable
    dfall_pkg::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the (x,y,z,yaw) values
    msg.buttonID = REQUEST_DEFAULT_SETPOINT_BUTTON_ID;

    // Publish the default setpoint button press
    this->requestSetpointChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Published request for setpoint change to the default");
#endif
}

void DefaultControllerTab::on_x_increment_plus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif

    }
}

void DefaultControllerTab::on_x_increment_minus_button_clicked()
{

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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}

void DefaultControllerTab::on_y_increment_plus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}

void DefaultControllerTab::on_y_increment_minus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}

void DefaultControllerTab::on_z_increment_plus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}

void DefaultControllerTab::on_z_increment_minus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}

void DefaultControllerTab::on_yaw_increment_plus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}
void DefaultControllerTab::on_yaw_increment_minus_button_clicked()
{
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
        #ifdef CATKIN_MAKE
        // Inform the user that nothing can be done
        ROS_INFO_STREAM("[DEFAULT CONTROLLER GUI] Increment x setpoint clicked but field is empty");
        #endif
    }
}





//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void DefaultControllerTab::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
{

    // Lock the mutex
    m_agentIDs_toCoordinate_mutex.lock();
    // Add the "coordinate all" flag
    m_shouldCoordinateAll = shouldCoordinateAll;
    // Clear the previous list of agent IDs
    m_vector_of_agentIDs_toCoordinate.clear();
    // Copy across the agent IDs, if necessary
    if (!shouldCoordinateAll)
    {
        for ( int irow = 0 ; irow < agentIDs.length() ; irow++ )
        {
            m_vector_of_agentIDs_toCoordinate.push_back( agentIDs[irow] );
        }
    }
    // Unlock the mutex
    m_agentIDs_toCoordinate_mutex.unlock();


#ifdef CATKIN_MAKE
    // If there is only one agent to coordinate,
    // then subscribe to the relevant data
    if (agentIDs.length() == 1)
    {

        // // > Create the appropriate node handle
        QString agent_base_namespace = "/dfall/agent" + QString::number(agentIDs[0]).rightJustified(3, '0');
        ros::NodeHandle agent_base_nodeHandle(agent_base_namespace.toStdString());

        // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = agent_base_nodeHandle.serviceClient<dfall_pkg::GetSetpointService>("DefaultControllerService/GetCurrentSetpoint", false);
        dfall_pkg::GetSetpointService getSetpointCall;
        getSetpointCall.request.data = 0;
        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(2.0));
        if(getCurrentSetpointServiceClient.call(getSetpointCall))
        {
            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
        }
        else
        {
            // Inform the user
            ROS_INFO("[DEFAULT CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }

        // SUBSCRIBERS
        // > For receiving message that the setpoint was changed
        setpointChangedSubscriber = agent_base_nodeHandle.subscribe("DefaultControllerService/SetpointChanged", 1, &DefaultControllerTab::setpointChangedCallback, this);
    }
    else
    {
        // Unsubscribe
        setpointChangedSubscriber.shutdown();

        // Set information back to the default
        ui->lineEdit_setpoint_current_x->setText("xx.xx");
        ui->lineEdit_setpoint_current_y->setText("xx.xx");
        ui->lineEdit_setpoint_current_z->setText("xx.xx");
        ui->lineEdit_setpoint_current_yaw->setText("xx.xx");

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
void DefaultControllerTab::fillSetpointMessageHeader( dfall_pkg::SetpointWithHeader & msg )
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
