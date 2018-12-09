//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
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
//    The bar of the GUI for (dis-)connecting (from)to the Crazyradio
//    and for sending the {take-off,land,motors-off} commands
//
//    ----------------------------------------------------------------------------------





#include "connectstartstopbar.h"
#include "ui_connectstartstopbar.h"





ConnectStartStopBar::ConnectStartStopBar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConnectStartStopBar)
{
    ui->setupUi(this);


#ifdef CATKIN_MAKE
    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[CONNECT START STOP GUI BAR] ros::this_node::getNamespace() =  " << this_namespace);


    // Get the type and ID of this parameter service
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the TYPE and ID are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[CONNECT START STOP GUI BAR] Node NOT FUNCTIONING :-)");
        ros::spin();
    }
#else
    // Default as a coordinator when compiling with QtCreator
    m_type = TYPE_COORDINATOR;
    m_ID = 1;
#endif

    // SET THE INITIAL VALUE OF THE PRIVATE VARIABLES FOR THIS CLASS
    // > For keeping track of the current battery state
    m_battery_state = BATTERY_STATE_NORMAL;
    // > For keeping track of which image is currently displayed
    m_battery_status_label_image_current_index = -999;


    // SET THE STARTING RADIO STATUS TO BE: DISCONNECTED
    setCrazyRadioStatus(CRAZY_RADIO_STATE_DISCONNECTED);

    // SET THE STARTING BATTERY VOLTAGE TO BE: UNKNOWN
    setBatteryVoltageText(-1.0f);

    // SET THE STARTING BATTERY LEVEL TO BE: UNAVAILABLE
    setBatteryImageBasedOnLevel(BATTERY_LEVEL_UNAVAILABLE);

    // SET THE STARTING FLYING STATE STATUS TO BE: MOTORS OFF
    setFlyingState(CMD_CRAZYFLY_MOTORS_OFF);

    // ENSURE THE F:YING STATE BUTTONS ARE AVAILABLE FOR A COORDINATOR
    if (m_type == TYPE_COORDINATOR)
    {
        enableFlyingStateButtons();
    }



#ifdef CATKIN_MAKE
    // CREATE A NODE HANDLE TO THE BASE NAMESPACE
    ros::NodeHandle base_nodeHandle(this_namespace);

    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    // SUBSCRIBERS AND PUBLISHERS:

    // > For Crazyradio commands based on button clicks
    crazyRadioCommandPublisher = base_nodeHandle.advertise<d_fall_pps::IntWithHeader>("PPSClient/crazyRadioCommand", 1);

    // > For Flying state commands based on button clicks
    flyingStateCommandPublisher = base_nodeHandle.advertise<d_fall_pps::IntWithHeader>("PPSClient/Command", 1);

    if (m_type == TYPE_AGENT)
    {
        // > For updating the "rf_status_label" picture
        crazyRadioStatusSubscriber = base_nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &ConnectStartStopBar::crazyRadioStatusCallback, this);

        // > For updating the current battery voltage
        batteryVoltageSubscriber = base_nodeHandle.subscribe("BatteryMonitor/FilteredVoltage", 1, &ConnectStartStopBar::batteryVoltageCallback, this);
    
        // > For updating the current battery state
        //batteryStateSubscriber = base_nodeHandle.subscribe("BatteryMonitor/ChangedStateTo", 1, &ConnectStartStopBar::batteryStateChangedCallback, this);

        // > For updating the current battery level
        batteryLevelSubscriber = base_nodeHandle.subscribe("BatteryMonitor/Level", 1, &ConnectStartStopBar::batteryLevelCallback, this);
    
        // > For updating the "flying_state_label" picture
        flyingStateSubscriber = base_nodeHandle.subscribe("PPSClient/flyingState", 1, &ConnectStartStopBar::flyingStateChangedCallback, this);
    }
#endif

    // FURTHER INITILIASATIONS NEED TO OCCUR AFTER THE ROS RELATED
    // INITIALISATIONS ARE COMPLETE
    if (m_type == TYPE_AGENT)
    {
        //loadCrazyflieContext();
    }

    // ADD KEYBOARD SHORTCUTS
    // > For "all motors off", press the space bar
    ui->motors_off_button->setShortcut(tr("Space"));
}

ConnectStartStopBar::~ConnectStartStopBar()
{
    delete ui;
}



// > For making the "enable flight" and "disable flight" buttons unavailable
void ConnectStartStopBar::disableFlyingStateButtons()
{
    //ui->motors_off_button->setEnabled(true);
    //ui->enable_flying_button->setEnabled(false);
    //ui->disable_flying_button->setEnabled(false);
}

// > For making the "enable flight" and "disable flight" buttons available
void ConnectStartStopBar::enableFlyingStateButtons()
{
    //ui->motors_off_button->setEnabled(true);
    //ui->enable_flying_button->setEnabled(true);
    //ui->disable_flying_button->setEnabled(true);
}




//    ----------------------------------------------------------------------------------
//     CCCC  RRRR     A    ZZZZZ  Y   Y  RRRR     A    DDDD   III   OOO
//    C      R   R   A A      Z    Y Y   R   R   A A   D   D   I   O   O
//    C      RRRR   A   A    Z      Y    RRRR   A   A  D   D   I   O   O
//    C      R   R  AAAAA   Z       Y    R   R  AAAAA  D   D   I   O   O
//     CCCC  R   R  A   A  ZZZZZ    Y    R   R  A   A  DDDD   III   OOO
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

// > For the Battery Voltage
void ConnectStartStopBar::crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Crazy Radio Status Callback called for agentID = " << m_agentID);
    setCrazyRadioStatus( msg.data );
}
#endif


// PRIVATE METHODS FOR SETTING PROPERTIES

void ConnectStartStopBar::setCrazyRadioStatus(int new_radio_status)
{
    // add more things whenever the status is changed
    switch(new_radio_status)
    {
        case CRAZY_RADIO_STATE_CONNECTED:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIOSTATUS LABEL
            //m_rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_connected_pixmap(":/images/rf_connected.png");
            ui->rf_status_label->setPixmap(rf_connected_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            //m_rf_status_label_mutex.unlock();

            // ENABLE THE REMAINDER OF THE GUI
            if (m_type == TYPE_AGENT)
            {
                enableFlyingStateButtons();
            }
            break;
        }

        case CRAZY_RADIO_STATE_CONNECTING:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIO STATUS LABEL
            //m_rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_connecting_pixmap(":/images/rf_connecting.png");
            ui->rf_status_label->setPixmap(rf_connecting_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            //m_rf_status_label_mutex.unlock();
            break;
        }

        case CRAZY_RADIO_STATE_DISCONNECTED:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIO STATUS LABEL
            //m_rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_disconnected_pixmap(":/images/rf_disconnected.png");
            ui->rf_status_label->setPixmap(rf_disconnected_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            //m_rf_status_label_mutex.unlock();

            // DISABLE THE REMAINDER OF THE GUI
            if (m_type == TYPE_AGENT)
            {
                disableFlyingStateButtons();
            }
            break;
        }

        default:
        {
            break;
        }
    }
}





//    ----------------------------------------------------------------------------------
//    BBBB     A    TTTTT  TTTTT  EEEEE  RRRR   Y   Y
//    B   B   A A     T      T    E      R   R   Y Y
//    BBBB   A   A    T      T    EEE    RRRR     Y
//    B   B  AAAAA    T      T    E      R   R    Y
//    BBBB   A   A    T      T    EEEEE  R   R    Y
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

// > For the Battery Voltage
void ConnectStartStopBar::batteryVoltageCallback(const std_msgs::Float32& msg)
{
    //setBatteryVoltageTextAndImage( msg.data );
    setBatteryVoltageText( msg.data );
}


void ConnectStartStopBar::batteryStateChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Battery State Changed Callback called for agentID = " << m_agentID);
    setBatteryState( msg.data );
}


void ConnectStartStopBar::batteryLevelCallback(const std_msgs::Int32& msg)
{
    setBatteryImageBasedOnLevel( msg.data );
}
#endif



// PRIVATE METHODS FOR SETTING PROPERTIES

// > For updating the battery state
void ConnectStartStopBar::setBatteryState(int new_battery_state)
{
    // SET THE CLASS VARIABLE FOR TRACKING THE BATTERY STATE
    m_battery_state = new_battery_state;
}



// > For updating the battery voltage shown in the UI elements of "battery_voltage_lineEdit"
void ConnectStartStopBar::setBatteryVoltageText(float battery_voltage)
{
    // Lock the mutex
    //m_battery_voltage_lineEdit_mutex.lock();
    // Construct the text string
    QString qstr = "";
    if (battery_voltage >= 0.0f)
    {
        qstr.append(QString::number(battery_voltage, 'f', 2));
    }
    else
    {
        qstr.append("-.--");
    }
    qstr.append(" V");
    // Set the text to the battery voltage line edit
    ui->battery_voltage_lineEdit->setText(qstr);
    // Unlock the mutex
    //m_battery_voltage_lineEdit_mutex.unlock();
}



// > For updating the battery image shown in the UI element of "battery_status_label"
void ConnectStartStopBar::setBatteryImageBasedOnLevel(int battery_level)
{
    // COMPUTE THE BATTERY VOLTAGE AS A PERCENTAGE
    //float battery_voltage_percentage = fromVoltageToPercent(battery_voltage);

    // CONVERT THE VOLTAGE LEVEL TO AN INDEX OF WHICH BATTERY LEVEL IMAGE TO DISPLAY
    // > Initialise a local variable that will be set in the switch case below
    int new_image_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
    // > Initialise a local variable for the string of which image to use
    QString qstr_new_image = "";
    qstr_new_image.append(":/images/");
    
    // Fill in these two local variables accordingly
    switch(battery_level)
    {
        case BATTERY_LEVEL_000:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_EMPTY;
            qstr_new_image.append("battery_empty.png");
            break;
        }
        case BATTERY_LEVEL_010:
        case BATTERY_LEVEL_020:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_20;
            qstr_new_image.append("battery_20.png");
            break;
        }
        case BATTERY_LEVEL_030:
        case BATTERY_LEVEL_040:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_40;
            qstr_new_image.append("battery_40.png");
            break;
        }
        case BATTERY_LEVEL_050:
        case BATTERY_LEVEL_060:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_60;
            qstr_new_image.append("battery_60.png");
            break;
        }
        case BATTERY_LEVEL_070:
        case BATTERY_LEVEL_080:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_80;
            qstr_new_image.append("battery_80.png");
            break;
        }
        case BATTERY_LEVEL_090:
        case BATTERY_LEVEL_100:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_FULL;
            qstr_new_image.append("battery_full.png");
            break;
        }
        case BATTERY_LEVEL_UNAVAILABLE:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_UNVAILABLE;
            qstr_new_image.append("battery_unknown.png");
            break;
        }
        default:
        {
            new_image_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
            qstr_new_image.append("battery_unknown.png");
            break;
        }
    }
    // UPDATE THE IMAGE DISPLAYED BASED ON THE "new index"
    // > Only if it is different from the current index
    m_battery_status_label_mutex.lock();
    if (m_battery_status_label_image_current_index != new_image_index)
    {
        // SET THE IMAGE FOR THE BATTERY STATUS LABEL
        ui->battery_status_label->clear();
        QPixmap battery_image_pixmap(qstr_new_image);
        ui->battery_status_label->setPixmap(battery_image_pixmap);
        ui->battery_status_label->setScaledContents(true);
        m_battery_status_label_image_current_index = new_image_index;
        ui->battery_status_label->update();
    }
    m_battery_status_label_mutex.unlock();
}





//    ----------------------------------------------------------------------------------
//    FFFFF  L      Y   Y  III   GGGG      SSSS  TTTTT    A    TTTTT  EEEE
//    F      L       Y Y    I   G         S        T     A A     T    E
//    FFF    L        Y     I   G          SSS     T    A   A    T    EEE
//    F      L        Y     I   G   G         S    T    AAAAA    T    E
//    F      LLLLL    Y    III   GGGG     SSSS     T    A   A    T    EEEEE
//    ----------------------------------------------------------------------------------



// RESPONDING TO CHANGES IN THE FLYING STATE
#ifdef CATKIN_MAKE
void ConnectStartStopBar::flyingStateChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Flying State Changed Callback called for agentID = " << m_agentID);
    setFlyingState(msg.data);
}
#endif

void ConnectStartStopBar::setFlyingState(int new_flying_state)
{
    // UPDATE THE LABEL TO DISPLAY THE FLYING STATE
    switch(new_flying_state)
    {
        case STATE_MOTORS_OFF:
        {
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_off_pixmap(":/images/flying_state_off.png");
            ui->flying_state_label->setPixmap(flying_state_off_pixmap);
            ui->flying_state_label->setScaledContents(true);
            break;
        }

        case STATE_TAKE_OFF:
        {
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_enabling_pixmap(":/images/flying_state_enabling.png");
            ui->flying_state_label->setPixmap(flying_state_enabling_pixmap);
            ui->flying_state_label->setScaledContents(true);
            break;
        }

        case STATE_FLYING:
        {
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_flying_pixmap(":/images/flying_state_flying.png");
            ui->flying_state_label->setPixmap(flying_state_flying_pixmap);
            ui->flying_state_label->setScaledContents(true);
            break;
        }

        case STATE_LAND:
        {
            //qstr.append("Land");
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_disabling_pixmap(":/images/flying_state_disabling.png");
            ui->flying_state_label->setPixmap(flying_state_disabling_pixmap);
            ui->flying_state_label->setScaledContents(true);
            break;
        }

        default:
        {
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_unknown_pixmap(":/images/flying_state_unknown.png");
            ui->flying_state_label->setPixmap(flying_state_unknown_pixmap);
            ui->flying_state_label->setScaledContents(true);
            break;
        }
    }
}





//    ----------------------------------------------------------------------------------
//    BBBB   U   U  TTTTT  TTTTT   OOO   N   N   SSSS
//    B   B  U   U    T      T    O   O  NN  N  S
//    BBBB   U   U    T      T    O   O  N N N   SSS
//    B   B  U   U    T      T    O   O  N  NN      S
//    BBBB    UUU     T      T     OOO   N   N  SSSS
//    ----------------------------------------------------------------------------------



void ConnectStartStopBar::on_rf_connect_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_RECONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Connect button clicked");
#endif
}

void ConnectStartStopBar::on_rf_disconnect_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_DISCONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Disconnect button clicked");
#endif
}

void ConnectStartStopBar::on_enable_flying_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_CRAZYFLY_TAKE_OFF;
    this->flyingStateCommandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Enable flying button clicked");
#endif
}

void ConnectStartStopBar::on_disable_flying_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_CRAZYFLY_LAND;
    this->flyingStateCommandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Disable flying button clicked");
#endif
}

void ConnectStartStopBar::on_motors_off_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_CRAZYFLY_MOTORS_OFF;
    this->flyingStateCommandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Motors-off button clicked");
#endif
}




//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void ConnectStartStopBar::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
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
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CONNECT START STOP GUI BAR] is coordinating agentIDs:";
    for ( int irow = 0 ; irow < agentIDs.length() ; irow++ )
    {
        QTextStream(stdout) << " " << agentIDs[irow];
    }
    QTextStream(stdout) << " " << endl;
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
void ConnectStartStopBar::fillIntMessageHeader( d_fall_pps::IntWithHeader & msg )
{
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            msg.shouldCheckForID = false;
            break;
        }
        case TYPE_COORDINATOR:
        {
            // Lock the mutex
            m_agentIDs_toCoordinate_mutex.lock();
            // Add the "coordinate all" flag
            msg.shouldCheckForID = !(m_shouldCoordinateAll);
            // Add the agent IDs if necessary
            if (!m_shouldCoordinateAll)
            {
                for ( int irow = 0 ; irow < m_vector_of_agentIDs_toCoordinate.length() ; irow++ )
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
            msg.shouldCheckForID = true;
            ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] The 'm_type' variable was not recognised.");
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
bool ConnectStartStopBar::getTypeAndIDParameters()
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
        ROS_ERROR("[CONNECT START STOP GUI BAR] Failed to get type");
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
        ROS_ERROR("[CONNECT START STOP GUI BAR] The 'type' parameter retrieved was not recognised.");
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
                ROS_ERROR("[CONNECT START STOP GUI BAR] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[CONNECT START STOP GUI BAR] Is of type AGENT with ID = " << m_ID);
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
                ROS_ERROR("[CONNECT START STOP GUI BAR] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[CONNECT START STOP GUI BAR] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[CONNECT START STOP GUI BAR] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
