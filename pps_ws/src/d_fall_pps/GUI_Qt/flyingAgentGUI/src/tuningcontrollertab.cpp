#include "tuningcontrollertab.h"
#include "ui_tuningcontrollertab.h"

TuningControllerTab::TuningControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TuningControllerTab)
{
    ui->setupUi(this);

    // Set the gain field
    int starting_slider_value = ui->slider_gain_P->value();
    float slider_ratio = float(starting_slider_value) / 1000.0;

    m_gain_P = P_GAIN_MIN_GUI + slider_ratio * (P_GAIN_MAX_GUI-P_GAIN_MIN_GUI);
    m_gain_D = m_gain_P * P_TO_D_GAIN_RATIO_GUI;

    ui->lineEdit_gain_P->setText( QString::number( m_gain_P, 'f', DECIMAL_PLACES_GAIN) );
    ui->lineEdit_gain_D->setText( QString::number( m_gain_D, 'f', DECIMAL_PLACES_GAIN) );

    ui->lineEdit_setpoint->setText( QString::number( m_current_setpoint, 'f', DECIMAL_PLACES_SETPOINT) );


#ifdef CATKIN_MAKE

    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[TUNING CONTROLLER TAB GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[TUNING CONTROLLER TAB GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE THE REQUEST SETPOINT CHANGE PUBLISHER
    requestSetpointChangePublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::SetpointWithHeader>("TuningControllerService/RequestSetpointChange", 1);

    // SUBSCRIBE TO SETPOINT CHANGES
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        setpointChangedSubscriber = nodeHandle_for_this_gui.subscribe("TuningControllerService/SetpointChanged", 1, &TuningControllerTab::setpointChangedCallback, this);
    }


    // CREATE THE NEW GAIN PUBLISHER
    requestNewGainChangePublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::FloatWithHeader>("TuningControllerService/RequestGainChange", 1);

    // CREATE THE CUSTOM BUTTON PRESSED PUBLISHER
    //customButtonPublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::CustomButtonWithHeader>("TuningControllerService/CustomButtonPressed", 1);

    // GET THE CURRENT SETPOINT
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
//        // > Request the current setpoint
//        ros::ServiceClient getCurrentSetpointServiceClient = nodeHandle_for_this_gui.serviceClient<d_fall_pps::GetSetpointService>("TuningControllerService/GetCurrentSetpoint", false);
//        d_fall_pps::GetSetpointService getSetpointCall;
//        getSetpointCall.request.data = 0;
//        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(2.0));
//        if(getCurrentSetpointServiceClient.call(getSetpointCall))
//        {
//            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
//        }
//        else
//        {
//            // Inform the user
//            ROS_INFO("[TUNING CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
//        }
    }

#endif
}

TuningControllerTab::~TuningControllerTab()
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


void TuningControllerTab::setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded)
{
    if (!occluded)
    {
        m_gain_setpoint_mutex.lock();

        // INITIALISE A STRING VARIABLE FOR ADDING THE "+"
        QString qstr = "";

        // PUT IN THE POSITION ERROR
        float error_x = x - m_current_setpoint;
        ui->lineEdit_error_P->setText( QString::number( error_x, 'f', DECIMAL_PLACES_POSITION_MEASURED) );

        // COMPUTE AND PUT IN THE VELOCITY
        float velocity_x = (x - m_x_previous) * MEASUREMENT_FRQUENCY;
        m_x_previous = x;
        ui->lineEdit_error_D->setText( QString::number( velocity_x, 'f', DECIMAL_PLACES_VELOCITY) );

        // COMPUTE AND PUT IN THE ANGLE
        float angle = (m_gain_P * error_x + m_gain_D * velocity_x) * RAD2DEG;
        ui->lineEdit_angle->setText( QString::number( angle, 'f', DECIMAL_PLACES_ANGLE_DEGREES) );

        m_gain_setpoint_mutex.unlock();

//        // UPDATE THE MEASUREMENT COLUMN
//        if (x < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_measured_x->setText(qstr + QString::number( x, 'f', 3));
//        if (y < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_measured_y->setText(qstr + QString::number( y, 'f', 3));
//        if (z < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_measured_z->setText(qstr + QString::number( z, 'f', 3));

//        if (roll < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_measured_roll->setText(qstr + QString::number( roll  * RAD2DEG, 'f', 1));
//        if (pitch < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_measured_pitch->setText(qstr + QString::number( pitch * RAD2DEG, 'f', 1));
//        if (yaw < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_measured_yaw->setText(qstr + QString::number( yaw   * RAD2DEG, 'f', 1));

//        // GET THE CURRENT SETPOINT
//        float error_x   = x   - (ui->lineEdit_setpoint_current_x->text()  ).toFloat();
//        float error_y   = y   - (ui->lineEdit_setpoint_current_y->text()  ).toFloat();
//        float error_z   = z   - (ui->lineEdit_setpoint_current_z->text()  ).toFloat();
//        float error_yaw = yaw - (ui->lineEdit_setpoint_current_yaw->text()).toFloat();

//        // UPDATE THE ERROR COLUMN
//        if (error_x < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_error_x->setText(qstr + QString::number( error_x, 'f', 3));
//        if (error_y < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_error_y->setText(qstr + QString::number( error_y, 'f', 3));
//        if (error_z < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_error_z->setText(qstr + QString::number( error_z, 'f', 3));

//        if (error_yaw < 0.0f) qstr = ""; else qstr = "+";
//        ui->lineEdit_error_yaw->setText(qstr + QString::number( error_yaw * RAD2DEG, 'f', 1));

//        // Ensure the red frames are not visible
//        if ( ui->red_frame_position_left->isVisible() )
//            ui->red_frame_position_left->setVisible(false);
//        if ( ui->red_frame_position_right->isVisible() )
//            ui->red_frame_position_right->setVisible(false);
    }
    else
    {
//        // Make visible the red frames to indicate occluded
//        if ( !(ui->red_frame_position_left->isVisible()) )
//            ui->red_frame_position_left->setVisible(true);
//        if ( !(ui->red_frame_position_right->isVisible()) )
//            ui->red_frame_position_right->setVisible(true);
    }
}


void TuningControllerTab::poseDataUnavailableSlot()
{
    ui->lineEdit_angle->setText("xx.xx");
    ui->lineEdit_error_P->setText("xx.xx");
    ui->lineEdit_error_D->setText("xx.xx");
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
void TuningControllerTab::setpointChangedCallback(const d_fall_pps::SetpointWithHeader& newSetpoint)
{
//    // INITIALISE A STRING VARIABLE FOR ADDING THE "+"
//    QString qstr = "";

//    // EXTRACT THE SETPOINT
//    float x = newSetpoint.x;
//    float y = newSetpoint.y;
//    float z = newSetpoint.z;
//    float yaw = newSetpoint.yaw;

//    // UPDATE THE SETPOINT COLUMN
//    if (x < 0.0f) qstr = ""; else qstr = "+";
//    ui->lineEdit_setpoint_current_x->setText(qstr + QString::number( x, 'f', 3));
//    if (y < 0.0f) qstr = ""; else qstr = "+";
//    ui->lineEdit_setpoint_current_y->setText(qstr + QString::number( y, 'f', 3));
//    if (z < 0.0f) qstr = ""; else qstr = "+";
//    ui->lineEdit_setpoint_current_z->setText(qstr + QString::number( z, 'f', 3));

//    if (yaw < 0.0f) qstr = ""; else qstr = "+";
//    ui->lineEdit_setpoint_current_yaw->setText(qstr + QString::number( yaw * RAD2DEG, 'f', 3));
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


void TuningControllerTab::publishSetpoint(float x, float y, float z, float yaw)
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
    msg.yaw = yaw * DEG2RAD;

    // Publish the setpoint
    this->requestSetpointChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[TUNING CONTROLLER GUI] Published request for setpoint change to: [" << x << ", "<< y << ", "<< z << ", "<< yaw << "]");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[TUNING CONTROLLER GUI] would publish request for: [" << x << ", "<< y << ", "<< z << ", "<< yaw << "]";
#endif
}





void TuningControllerTab::on_button_setpoint_toggle_clicked()
{

    m_gain_setpoint_mutex.lock();

    float setpoint_copy = m_current_setpoint;
    float new_setpoint = 0.0;

    if (setpoint_copy < SETPOINT_X_MINUS)
        new_setpoint = SETPOINT_X_MINUS;
    else if (setpoint_copy < 0.0)
        new_setpoint = SETPOINT_X_PLUS;
    else if (setpoint_copy > SETPOINT_X_PLUS)
        new_setpoint = SETPOINT_X_PLUS;
    else
        new_setpoint = SETPOINT_X_MINUS;

    ui->lineEdit_setpoint->setText( QString::number( new_setpoint, 'f', DECIMAL_PLACES_SETPOINT) );
    float lineEdit_as_float_rounded = (ui->lineEdit_setpoint->text()).toFloat();
    m_current_setpoint = lineEdit_as_float_rounded;
    publishSetpoint( m_current_setpoint, SETPOINT_Y, SETPOINT_Z, SETPOINT_YAW_DEGREES);

    m_gain_setpoint_mutex.unlock();

}

void TuningControllerTab::on_lineEdit_setpoint_editingFinished()
{
    m_gain_setpoint_mutex.lock();


    // Get the line edit data, as a float if possible
    bool isValidFloat = false;
    float lineEdit_as_float = (ui->lineEdit_setpoint->text()).toFloat(&isValidFloat);

    // Fill in the data
    if (isValidFloat)
    {
        ui->lineEdit_setpoint->setText( QString::number( lineEdit_as_float, 'f', DECIMAL_PLACES_SETPOINT) );
        float lineEdit_as_float_rounded = (ui->lineEdit_setpoint->text()).toFloat();
        m_current_setpoint = lineEdit_as_float_rounded;
        publishSetpoint( m_current_setpoint, SETPOINT_Y, SETPOINT_Z, SETPOINT_YAW_DEGREES);
    }
    else
    {
        ui->lineEdit_setpoint->setText( QString::number( m_current_setpoint, 'f', DECIMAL_PLACES_SETPOINT) );
    }
    m_gain_setpoint_mutex.unlock();
}














void TuningControllerTab::publishGain(float new_gain)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    d_fall_pps::FloatWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the gain value
    msg.data = new_gain;

    // Publish the setpoint
    this->requestNewGainChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[TUNING CONTROLLER GUI] Published request for gain change to: " << new_gain );
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[TUNING CONTROLLER GUI] would publish gain value: " << new_gain;
#endif
}


void TuningControllerTab::on_slider_gain_P_valueChanged(int value)
{

    m_gain_setpoint_mutex.lock();

    float slider_ratio = float(value) / 1000.0;

    float new_gain_P = P_GAIN_MIN_GUI + slider_ratio * (P_GAIN_MAX_GUI-P_GAIN_MIN_GUI);
    float new_gain_D = new_gain_P * P_TO_D_GAIN_RATIO_GUI;

    ui->lineEdit_gain_P->setText( QString::number( new_gain_P, 'f', DECIMAL_PLACES_GAIN) );
    ui->lineEdit_gain_D->setText( QString::number( new_gain_D, 'f', DECIMAL_PLACES_GAIN) );

    float gain_P_rounded = (ui->lineEdit_gain_P->text()).toFloat();
    float gain_D_rounded = (ui->lineEdit_gain_D->text()).toFloat();

    m_gain_P = gain_P_rounded;
    m_gain_D = gain_D_rounded;

    publishGain(m_gain_P);

    m_gain_setpoint_mutex.unlock();

}







//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void TuningControllerTab::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
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

//        // // > Create the appropriate node handle
//        QString agent_base_namespace = "/dfall/agent" + QString::number(agentIDs[0]).rightJustified(3, '0');
//        ros::NodeHandle agent_base_nodeHandle(agent_base_namespace.toStdString());

//        // // > Request the current setpoint
//        ros::ServiceClient getCurrentSetpointServiceClient = agent_base_nodeHandle.serviceClient<d_fall_pps::GetSetpointService>("TuningControllerService/GetCurrentSetpoint", false);
//        d_fall_pps::GetSetpointService getSetpointCall;
//        getSetpointCall.request.data = 0;
//        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(2.0));
//        if(getCurrentSetpointServiceClient.call(getSetpointCall))
//        {
//            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
//        }
//        else
//        {
//            // Inform the user
//            ROS_INFO("[TUNING CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
//        }

//        // SUBSCRIBERS
//        // > For receiving message that the setpoint was changed
//        setpointChangedSubscriber = agent_base_nodeHandle.subscribe("TuningControllerService/SetpointChanged", 1, &TuningControllerTab::setpointChangedCallback, this);
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
void TuningControllerTab::fillSetpointMessageHeader( d_fall_pps::SetpointWithHeader & msg )
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
            ROS_ERROR("[TUNING CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }
}
#endif



#ifdef CATKIN_MAKE
// Fill the header for a message
void TuningControllerTab::fillFloatMessageHeader( d_fall_pps::FloatWithHeader & msg )
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
            ROS_ERROR("[TUNING CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
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
bool TuningControllerTab::getTypeAndIDParameters()
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
        ROS_ERROR("[TUNING CONTROLLER TAB GUI] Failed to get type");
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
        ROS_ERROR("[TUNING CONTROLLER TAB GUI] The 'type' parameter retrieved was not recognised.");
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
                ROS_ERROR("[TUNING CONTROLLER TAB GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[TUNING CONTROLLER TAB GUI] Is of type AGENT with ID = " << m_ID);
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
                ROS_ERROR("[TUNING CONTROLLER TAB GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[TUNING CONTROLLER TAB GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[TUNING CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
