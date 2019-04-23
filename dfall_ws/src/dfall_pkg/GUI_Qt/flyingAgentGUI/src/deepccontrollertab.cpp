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
//    The GUI for a Deepc Controller for students build from
//
//    ----------------------------------------------------------------------------------





#include "deepccontrollertab.h"
#include "ui_deepccontrollertab.h"

DeepcControllerTab::DeepcControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DeepcControllerTab)
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
    ROS_INFO_STREAM("[DEEPC CONTROLLER TAB GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[DEEPC CONTROLLER TAB GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE THE REQUEST SETPOINT CHANGE PUBLISHER
    requestSetpointChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::SetpointWithHeader>("DeepcControllerService/RequestSetpointChange", 1);

    // SUBSCRIBE TO SETPOINT CHANGES
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        setpointChangedSubscriber = nodeHandle_for_this_gui.subscribe("DeepcControllerService/SetpointChanged", 1, &DeepcControllerTab::setpointChangedCallback, this);
    }

    // CREATE THE CUSTOM BUTTON PRESSED PUBLISHER
    customButtonPublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::CustomButtonWithHeader>("DeepcControllerService/CustomButtonPressed", 1);

    // GET THE CURRENT SETPOINT
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = nodeHandle_for_this_gui.serviceClient<dfall_pkg::GetSetpointService>("DeepcControllerService/GetCurrentSetpoint", false);
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
            ROS_INFO("[DEEPC CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }
    }

#endif

}

DeepcControllerTab::~DeepcControllerTab()
{
    delete ui;
}





//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//    BBBB   U   U  TTTTT  TTTTT   OOO   N   N   SSSS
//    B   B  U   U    T      T    O   O  NN  N  S
//    BBBB   U   U    T      T    O   O  N N N   SSS
//    B   B  U   U    T      T    O   O  N  NN      S
//    BBBB    UUU     T      T     OOO   N   N  SSSS
//    ----------------------------------------------------------------------------------


#ifdef CATKIN_MAKE
void DeepcControllerTab::publish_custom_button_command(int button_index , QLineEdit * lineEdit_pointer)
{
    // Initialise the message as a local variable
    dfall_pkg::CustomButtonWithHeader msg;
    // Fill the header of the message
    fillCustomButtonMessageHeader( msg );
    // Fill in the button index
    msg.button_index = button_index;
    // Get the line edit data, as a float if possible
    bool isValidFloat = false;
    float lineEdit_as_float = (lineEdit_pointer->text()).toFloat(&isValidFloat);
    // Fill in the data
    if (isValidFloat)
        msg.float_data = lineEdit_as_float;
    else
        msg.string_data = (lineEdit_pointer->text()).toStdString();
    // Publish the setpoint
    this->customButtonPublisher.publish(msg);
    // Inform the user about the change
    ROS_INFO_STREAM("[DEEPC CONTROLLER TAB GUI] button " << button_index << " clicked.");
}
#endif


void DeepcControllerTab::on_custom_button_1_clicked()
{
#ifdef CATKIN_MAKE
    publish_custom_button_command(1,ui->lineEdit_custom_1);
#endif
}

void DeepcControllerTab::on_custom_button_2_clicked()
{
#ifdef CATKIN_MAKE
    publish_custom_button_command(2,ui->lineEdit_custom_2);
#endif
}

void DeepcControllerTab::on_custom_button_3_clicked()
{
#ifdef CATKIN_MAKE
    publish_custom_button_command(3,ui->lineEdit_custom_3);
#endif
}

void DeepcControllerTab::on_custom_button_4_clicked()
{
#ifdef CATKIN_MAKE
    publish_custom_button_command(4,ui->lineEdit_custom_4);
#endif
}

void DeepcControllerTab::on_custom_button_5_clicked()
{
#ifdef CATKIN_MAKE
    publish_custom_button_command(5,ui->lineEdit_custom_5);
#endif
}

//    ----------------------------------------------------------------------------------
//    PPPP    OOO    SSSS  EEEEE     DDDD     A    TTTTT    A
//    P   P  O   O  S      E         D   D   A A     T     A A
//    PPPP   O   O   SSS   EEE       D   D  A   A    T    A   A
//    P      O   O      S  E         D   D  AAAAA    T    AAAAA
//    P       OOO   SSSS   EEEEE     DDDD   A   A    T    A   A
//    ----------------------------------------------------------------------------------


void DeepcControllerTab::setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded)
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


void DeepcControllerTab::poseDataUnavailableSlot()
{
    ui->lineEdit_measured_x->setText("xx.xx");
    ui->lineEdit_measured_y->setText("xx.xx");
    ui->lineEdit_measured_z->setText("xx.xx");

    ui->lineEdit_measured_roll->setText("xx.xx");
    ui->lineEdit_measured_pitch->setText("xx.xx");
    ui->lineEdit_measured_yaw->setText("xx.xx");
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
void DeepcControllerTab::setpointChangedCallback(const dfall_pkg::SetpointWithHeader& newSetpoint)
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
    ui->lineEdit_setpoint_current_yaw->setText(qstr + QString::number( yaw * RAD2DEG, 'f', 1));

    // UPDATE THE CURRENT STATE LABEL
    int current_state = newSetpoint.buttonID;
    m_label_current_state_mutex.lock();
    switch (current_state)
    {
        case DEEPC_CONTROLLER_STATE_STANDBY:
            ui->label_current_state->setText("Standby");
            break;

        case DEEPC_CONTROLLER_STATE_NORMAL:
            ui->label_current_state->setText("Normal");
            break;

        case DEEPC_CONTROLLER_STATE_EXCITATION:
            ui->label_current_state->setText("Excitation");
            break;

        case DEEPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
            ui->label_current_state->setText("Landing, move down");
            break;

        case DEEPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
            ui->label_current_state->setText("Landing, spinning motors");
            break;

        default:
            ui->label_current_state->setText("Not recognised");
            break;
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


void DeepcControllerTab::publishSetpoint(float x, float y, float z, float yaw_degrees)
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
    msg.yaw = yaw_degrees * DEG2RAD;

    // Publish the setpoint
    this->requestSetpointChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[DEEPC CONTROLLER GUI] Published request for setpoint change to: [" << x << ", "<< y << ", "<< z << ", "<< yaw_degrees << "]");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[DEEPC CONTROLLER GUI] would publish request for: [" << x << ", "<< y << ", "<< z << ", "<< yaw_degrees << "]";
#endif
}



void DeepcControllerTab::on_lineEdit_setpoint_new_x_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DeepcControllerTab::on_lineEdit_setpoint_new_y_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DeepcControllerTab::on_lineEdit_setpoint_new_z_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DeepcControllerTab::on_lineEdit_setpoint_new_yaw_returnPressed()
{
    ui->set_setpoint_button->animateClick();
}

void DeepcControllerTab::on_set_setpoint_button_clicked()
{

    // Initialise local variable for each of (x,y,z,yaw)
    float x = 0.0f, y = 0.0f, z = 0.0f, yaw = 0.0f;

    // Take the new value if available, otherwise use the old value
    // > For x
    if(!ui->lineEdit_setpoint_new_x->text().isEmpty())
        x = (ui->lineEdit_setpoint_new_x->text()).toFloat();
    else
        x = (ui->lineEdit_setpoint_current_x->text()).toFloat();
    // > For y
    if(!ui->lineEdit_setpoint_new_y->text().isEmpty())
        y = (ui->lineEdit_setpoint_new_y->text()).toFloat();
    else
        y = (ui->lineEdit_setpoint_current_y->text()).toFloat();
    // > For z
    if(!ui->lineEdit_setpoint_new_z->text().isEmpty())
        z = (ui->lineEdit_setpoint_new_z->text()).toFloat();
    else
        z = (ui->lineEdit_setpoint_current_z->text()).toFloat();
    // > For yaw
    if(!ui->lineEdit_setpoint_new_yaw->text().isEmpty())
        yaw = (ui->lineEdit_setpoint_new_yaw->text()).toFloat();
    else
        yaw = (ui->lineEdit_setpoint_current_yaw->text()).toFloat();

    // Call the function to publish the setpoint
    publishSetpoint(x,y,z,yaw);
}

void DeepcControllerTab::on_default_setpoint_button_clicked()
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
    ROS_INFO_STREAM("[DEEPC CONTROLLER GUI] Published request for setpoint change to the default");
#endif
}





//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void DeepcControllerTab::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
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

        // // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = agent_base_nodeHandle.serviceClient<dfall_pkg::GetSetpointService>("DeepcControllerService/GetCurrentSetpoint", false);
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
            ROS_INFO("[DEEPC CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }

        // SUBSCRIBERS
        // > For receiving message that the setpoint was changed
        setpointChangedSubscriber = agent_base_nodeHandle.subscribe("DeepcControllerService/SetpointChanged", 1, &DeepcControllerTab::setpointChangedCallback, this);
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
// Fill the header for a message
void DeepcControllerTab::fillSetpointMessageHeader( dfall_pkg::SetpointWithHeader & msg )
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
            ROS_ERROR("[DEEPC CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }
}
#endif



#ifdef CATKIN_MAKE
// Fill the header for a message
void DeepcControllerTab::fillCustomButtonMessageHeader( dfall_pkg::CustomButtonWithHeader & msg )
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
            ROS_ERROR("[DEEPC CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
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
bool DeepcControllerTab::getTypeAndIDParameters()
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
        ROS_ERROR("[DEEPC CONTROLLER TAB GUI] Failed to get type");
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
        ROS_ERROR("[DEEPC CONTROLLER TAB GUI] The 'type' parameter retrieved was not recognised.");
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
                ROS_ERROR("[DEEPC CONTROLLER TAB GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[DEEPC CONTROLLER TAB GUI] Is of type AGENT with ID = " << m_ID);
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
                ROS_ERROR("[DEEPC CONTROLLER TAB GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[DEEPC CONTROLLER TAB GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[DEEPC CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
