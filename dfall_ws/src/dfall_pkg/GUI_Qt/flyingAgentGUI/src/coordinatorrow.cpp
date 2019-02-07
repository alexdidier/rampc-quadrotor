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
//    Coordinator Row GUI.
//
//    ----------------------------------------------------------------------------------





#include "coordinatorrow.h"
#include "ui_coordinatorrow.h"





CoordinatorRow::CoordinatorRow(QWidget *parent, int agentID) :
    QWidget(parent),
    ui(new Ui::CoordinatorRow),
    m_agentID(agentID)
{
    ui->setupUi(this);

    // CONVERT THE AGENT ID TO A ZERO PADDED STRING
    // > This is the c++ method:
    //std::ostringstream str_stream;
    //str_stream << std::setw(3) << std::setfill('0') << m_agentID;
    //std::string agentID_as_string(str_stream.str());
    // > This is the Qt method
    //m_agentID_as_string = QString("%1").arg(m_agentID, 3, 10, QChar('0'));
    //   For which the syntax is:
    //   - Arg1: the number
    //   - Arg2: how many 0 you want?
    //   - Arg3: The base (10 - decimal, 16 hexadecimal)
    // > Alternate Qt method:
    m_agentID_as_string = QString::number(m_agentID).rightJustified(3, '0');

    // CONVERT THE AGENT ID TO A STRING FOR THE BASE NAMESPACE
    QString qstr_base_namespace = "/dfall/agent";
    qstr_base_namespace.append(m_agentID_as_string);
    std::string base_namespace = qstr_base_namespace.toStdString();

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
    setFlyingState(STATE_UNAVAILABLE);
    
    // SET THE DEFAULT NAME FOR THE SELECTED CONTROLLER
    setControllerEnabled(SAFE_CONTROLLER);


#ifdef CATKIN_MAKE

    // LET THE USER KNOW WHAT THE BASE NAMESPACE IS
    ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] using base namespace: " << base_namespace.c_str() << ", for agentID = " << m_agentID);

    // DEBUGGING FOR NAMESPACES
    //std::string temp_ros_namespace = ros::this_node::getNamespace();
    //ROS_INFO_STREAM("[COORDINATOR ROW GUI] compared to: ros::this_node::getNamespace() = " << temp_ros_namespace.c_str());

    // CREATE A NODE HANDLE TO THE BASE NAMESPACE
    ros::NodeHandle base_nodeHandle(base_namespace);

    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    // SUBSCRIBERS AND PUBLISHERS:

    // > For Crazyradio commands based on button clicks
    crazyRadioCommandPublisher = base_nodeHandle.advertise<dfall_pkg::IntWithHeader>("PPSClient/crazyRadioCommand", 1);
    // > For updating the "rf_status_label" picture
    crazyRadioStatusSubscriber = base_nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &CoordinatorRow::crazyRadioStatusCallback, this);

    // > For updating the current battery voltage
    batteryVoltageSubscriber = base_nodeHandle.subscribe("BatteryMonitor/FilteredVoltage", 1, &CoordinatorRow::batteryVoltageCallback, this);
    // > For updating the current battery state
    //batteryStateSubscriber = base_nodeHandle.subscribe("BatteryMonitor/ChangedStateTo", 1, &CoordinatorRow::batteryStateChangedCallback, this);
    // > For updating the current battery level
    batteryLevelSubscriber = base_nodeHandle.subscribe("BatteryMonitor/Level", 1, &CoordinatorRow::batteryLevelCallback, this);

    // > For Flying state commands based on button clicks
    flyingStateCommandPublisher = base_nodeHandle.advertise<dfall_pkg::IntWithHeader>("PPSClient/Command", 1);
    // > For updating the "flying_state_label" picture
    flyingStateSubscriber = base_nodeHandle.subscribe("PPSClient/flyingState", 1, &CoordinatorRow::flyingStateChangedCallback, this);

    // > For changes in the database that defines {agentID,cfID,flying zone} links
    databaseChangedSubscriber = dfall_root_nodeHandle.subscribe("CentralManagerService/DBChanged", 1, &CoordinatorRow::databaseChangedCallback, this);;
    centralManagerDatabaseService = dfall_root_nodeHandle.serviceClient<dfall_pkg::CMQuery>("CentralManagerService/Query", false);

    // > For updating the controller that is currently operating
    controllerUsedSubscriber = base_nodeHandle.subscribe("PPSClient/controllerUsed", 1, &CoordinatorRow::controllerUsedChangedCallback, this);

    // > For requesting the current flying state,
    //   this is used only for initialising the icon
    getCurrentFlyingStateService = base_nodeHandle.serviceClient<dfall_pkg::IntIntService>("PPSClient/getCurrentFlyingState", false);

    // > For requesting the current state of the Crazy Radio,
    //   this is used only for initialising the icon
    getCurrentCrazyRadioStateService = base_nodeHandle.serviceClient<dfall_pkg::IntIntService>("CrazyRadio/getCurrentCrazyRadioStatus", false);

#endif

    // FURTHER INITILIASATIONS NEED TO OCCUR AFTER THE ROS RELATED
    // INITIALISATIONS ARE COMPLETE
    loadCrazyflieContext();

    // > Request the current flying state
    getCurrentFlyingState();

    // > Request the current state of the Crazy Radio
    getCurrentCrazyRadioState();

}

CoordinatorRow::~CoordinatorRow()
{
    delete ui;
}



// PUBLIC METHODS FOR SETTING PROPERTIES

// > Set the state of the checkbox
void CoordinatorRow::setShouldCoordinate(bool shouldCoordinate)
{
    ui->shouldCoordinate_checkBox->setChecked( shouldCoordinate );
}

void CoordinatorRow::setLevelOfDetailToDisplay(int level)
{
    switch (level)
    {
    case 0:
    {
        ui->motors_off_button->setVisible(true);
        ui->enable_flying_button->setVisible(true);
        ui->disable_flying_button->setVisible(true);

        ui->battery_voltage_lineEdit->setVisible(true);

        ui->rf_connect_button->setVisible(true);
        ui->rf_disconnect_button->setVisible(true);

        QString qstr_for_checkbox_label = "ID";
        qstr_for_checkbox_label.append(QString::number(m_agentID));
        qstr_for_checkbox_label.append(", ");
        qstr_for_checkbox_label.append(m_crazyflie_name_as_string);
        ui->shouldCoordinate_checkBox->setText(qstr_for_checkbox_label);
        break;
    }
    case 1:
    {
        ui->motors_off_button->setVisible(false);
        ui->enable_flying_button->setVisible(false);
        ui->disable_flying_button->setVisible(false);

        ui->battery_voltage_lineEdit->setVisible(false);

        ui->rf_connect_button->setVisible(false);
        ui->rf_disconnect_button->setVisible(false);

        QString qstr_for_checkbox_label = "ID";
        qstr_for_checkbox_label.append(QString::number(m_agentID));
        ui->shouldCoordinate_checkBox->setText(qstr_for_checkbox_label);


        break;
    }
    default:
    {
        ui->motors_off_button->setVisible(true);
        ui->enable_flying_button->setVisible(true);
        ui->disable_flying_button->setVisible(true);

        ui->battery_voltage_lineEdit->setVisible(true);

        ui->rf_connect_button->setVisible(true);
        ui->rf_disconnect_button->setVisible(true);

        QString qstr_for_checkbox_label = "ID";
        qstr_for_checkbox_label.append(QString::number(m_agentID));
        qstr_for_checkbox_label.append(", ");
        qstr_for_checkbox_label.append(m_crazyflie_name_as_string);
        ui->shouldCoordinate_checkBox->setText(qstr_for_checkbox_label);
        break;
    }
    }
}


// > For making the "enable flight" and "disable flight" buttons unavailable
void CoordinatorRow::disableFlyingStateButtons()
{
    ui->motors_off_button->setEnabled(true);
    ui->enable_flying_button->setEnabled(false);
    ui->disable_flying_button->setEnabled(false);
}

// > For making the "enable flight" and "disable flight" buttons available
void CoordinatorRow::enableFlyingStateButtons()
{
    ui->motors_off_button->setEnabled(true);
    ui->enable_flying_button->setEnabled(true);
    ui->disable_flying_button->setEnabled(true);
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
void CoordinatorRow::crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Crazy Radio Status Callback called for agentID = " << m_agentID);
    setCrazyRadioStatus( msg.data );
}
#endif


// PRIVATE METHODS FOR SETTING PROPERTIES

void CoordinatorRow::setCrazyRadioStatus(int new_radio_status)
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
            enableFlyingStateButtons();
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
            disableFlyingStateButtons();
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
void CoordinatorRow::batteryVoltageCallback(const std_msgs::Float32& msg)
{
    //setBatteryVoltageTextAndImage( msg.data );
    setBatteryVoltageText( msg.data );
}


void CoordinatorRow::batteryStateChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Battery State Changed Callback called for agentID = " << m_agentID);
    setBatteryState( msg.data );
}


void CoordinatorRow::batteryLevelCallback(const std_msgs::Int32& msg)
{
    setBatteryImageBasedOnLevel( msg.data );
}
#endif



// PRIVATE METHODS FOR SETTING PROPERTIES

// > For updating the battery state
void CoordinatorRow::setBatteryState(int new_battery_state)
{
    // SET THE CLASS VARIABLE FOR TRACKING THE BATTERY STATE
    m_battery_state = new_battery_state;
}



// > For updating the battery voltage shown in the UI elements of "battery_voltage_lineEdit"
void CoordinatorRow::setBatteryVoltageText(float battery_voltage)
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
void CoordinatorRow::setBatteryImageBasedOnLevel(int battery_level)
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
            qstr_new_image.append("battery_unavailable.png");
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
void CoordinatorRow::flyingStateChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Flying State Changed Callback called for agentID = " << m_agentID);
    setFlyingState(msg.data);
}
#endif

void CoordinatorRow::setFlyingState(int new_flying_state)
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
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_disabling_pixmap(":/images/flying_state_disabling.png");
            ui->flying_state_label->setPixmap(flying_state_disabling_pixmap);
            ui->flying_state_label->setScaledContents(true);
            break;
        }

        case STATE_UNAVAILABLE:
        {
            // SET THE APPROPRIATE IMAGE FOR THE FLYING STATE LABEL
            QPixmap flying_state_disabling_pixmap(":/images/flying_state_unavailable.png");
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
//     CCCC   OOO   N   N  TTTTT  EEEEE  X   X  TTTTT
//    C      O   O  NN  N    T    E       X X     T
//    C      O   O  N N N    T    EEE      X      T
//    C      O   O  N  NN    T    E       X X     T
//     CCCC   OOO   N   N    T    EEEEE  X   X    T
//    ----------------------------------------------------------------------------------



// RESPONDING TO CHANGES IN THE DATABASE
#ifdef CATKIN_MAKE
// > For the notification that the database was changes, received on the "DatabaseChangedSubscriber"
void CoordinatorRow::databaseChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Database Changed Callback called for agentID = " << m_agentID);
    loadCrazyflieContext();
}
#endif

// > For loading the "context" for this agent, i.e., the {agentID,cfID,flying zone} tuple
void CoordinatorRow::loadCrazyflieContext()
{
    QString qstr_crazyflie_name = "";
#ifdef CATKIN_MAKE
    dfall_pkg::CMQuery contextCall;
    contextCall.request.studentID = m_agentID;
    //ROS_INFO_STREAM("StudentID:" << m_agentID);

    centralManagerDatabaseService.waitForExistence(ros::Duration(-1));

    if(centralManagerDatabaseService.call(contextCall))
    {
        my_context = contextCall.response.crazyflieContext;
        ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] CrazyflieContext:\n" << my_context);

        qstr_crazyflie_name.append(QString::fromStdString(my_context.crazyflieName));
    }
    else
    {
        ROS_ERROR_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] Failed to load context for agentID = " << m_agentID);
    }
    // This updating of the radio only needs to be done by the actual agent's node
    //ros::NodeHandle nh("CrazyRadio");
    //nh.setParam("crazyFlieAddress", m_context.crazyflieAddress);
#else
    // Set the Crazyflie Name String to be a question mark
    qstr_crazyflie_name.append("?");
#endif

    // Construct and set the string for the checkbox label
    QString qstr_for_checkbox_label = "ID";
    qstr_for_checkbox_label.append(QString::number(m_agentID));
    qstr_for_checkbox_label.append(", ");
    qstr_for_checkbox_label.append(qstr_crazyflie_name);
    ui->shouldCoordinate_checkBox->setText(qstr_for_checkbox_label);

    // Set the name of the Crazyflie to the class variable
    m_crazyflie_name_as_string = qstr_crazyflie_name;
}



void CoordinatorRow::getCurrentFlyingState()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntIntService getFlyingStateCall;
    getFlyingStateCall.request.data = 0;
    getCurrentFlyingStateService.waitForExistence(ros::Duration(2.0));
    if(getCurrentFlyingStateService.call(getFlyingStateCall))
    {
        setFlyingState(getFlyingStateCall.response.data);
    }
    else
    {
        setFlyingState(STATE_UNAVAILABLE);
    }
#endif
}




void CoordinatorRow::getCurrentCrazyRadioState()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntIntService getCrazyRadioCall;
    getCrazyRadioCall.request.data = 0;
    getCurrentCrazyRadioStateService.waitForExistence(ros::Duration(2.0));
    if(getCurrentCrazyRadioStateService.call(getCrazyRadioCall))
    {
        setCrazyRadioStatus(getCrazyRadioCall.response.data);
    }
    else
    {
        setCrazyRadioStatus(CRAZY_RADIO_STATE_DISCONNECTED);
    }
#endif
}




//    ----------------------------------------------------------------------------------
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// > For the controller currently operating, received on "controllerUsedSubscriber"
void CoordinatorRow::controllerUsedChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Controller Used Changed Callback called for agentID = " << m_agentID);
    setControllerEnabled(msg.data);
}
#endif


void CoordinatorRow::setControllerEnabled(int new_controller)
{
    switch(new_controller)
    {
        case SAFE_CONTROLLER:
        {
            ui->controller_enabled_label->setText("Safe");
            break;
        }
        case DEMO_CONTROLLER:
        {
            ui->controller_enabled_label->setText("Demo");
            break;
        }
        case STUDENT_CONTROLLER:
        {
            ui->controller_enabled_label->setText("Student");
            break;
        }
        case MPC_CONTROLLER:
        {
            ui->controller_enabled_label->setText("MPC");
            break;
        }
        case REMOTE_CONTROLLER:
        {
            ui->controller_enabled_label->setText("Remote");
            break;
        }
        case TUNING_CONTROLLER:
        {
            ui->controller_enabled_label->setText("Tuning");
            break;
        }
        default:
        {
            ui->controller_enabled_label->setText("Unknown");
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



void CoordinatorRow::on_rf_connect_button_clicked()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_RECONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] Connect button clicked");
#endif
}

void CoordinatorRow::on_rf_disconnect_button_clicked()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_DISCONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] Disconnect button clicked");
#endif
}

void CoordinatorRow::on_enable_flying_button_clicked()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_TAKE_OFF;
    this->flyingStateCommandPublisher.publish(msg);
    ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] Enable flying button clicked");
#endif
}

void CoordinatorRow::on_disable_flying_button_clicked()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_LAND;
    this->flyingStateCommandPublisher.publish(msg);
    ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] Disable flying button clicked");
#endif
}

void CoordinatorRow::on_motors_off_button_clicked()
{
#ifdef CATKIN_MAKE
    dfall_pkg::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_MOTORS_OFF;
    this->flyingStateCommandPublisher.publish(msg);
    ROS_INFO_STREAM("[COORDINATOR ROW agentID:" << m_agentID << " GUI] Motors-off button clicked");
#endif
}

void CoordinatorRow::on_shouldCoordinate_checkBox_clicked()
{
    // Get the state of the "should coordinate" check box
    bool shouldCoordinate = ui->shouldCoordinate_checkBox->isChecked();

    // Send out a signal with this information
    emit shouldCoordinateThisAgentValueChanged( m_agentID , shouldCoordinate );
}
