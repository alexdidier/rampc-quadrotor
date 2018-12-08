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
    my_agentID(agentID)
{
    ui->setupUi(this);

    // CONVERT THE AGENT ID TO A ZERO PADDED STRING
    // > This is the c++ method:
    //std::ostringstream str_stream;
    //str_stream << std::setw(3) << std::setfill('0') << my_agentID;
    //std::string agentID_as_string(str_stream.str());
    // > This is the Qt method
    //my_agentID_as_string = QString("%1").arg(my_agentID, 3, 10, QChar('0'));
    //   For which the syntax is:
    //   - Arg1: the number
    //   - Arg2: how many 0 you want?
    //   - Arg3: The base (10 - decimal, 16 hexadecimal)
    // > Alternate Qt method:
    my_agentID_as_string = QString::number(my_agentID).rightJustified(3, '0');

    // CONVERT THE AGENT ID TO A STRING FOR THE BASE NAMESPACE
    QString qstr_ros_base_namespace = "/dfall/agent";
    qstr_ros_base_namespace.append(my_agentID_as_string);
    std::string ros_base_namespace = qstr_ros_base_namespace.toStdString();

    // SET THE INITIAL VALUE OF THE PRIVATE VARIABLES FOR THIS CLASS
    // > For keeping track of the current RF Crazyradio state
    my_radio_status = CONNECTED;
    // > For keeping track of the current battery state
    my_battery_state = BATTERY_STATE_NORMAL;
    // > For keeping track of which image is currently displayed
    my_battery_status_label_image_current_index = -999;
    // > For keeping track of the current operating state
    my_flying_state = STATE_MOTORS_OFF;

    // FOR BATTERY VOLTAGE LIMITS (THESE SHOULD BE READ IN AS PARAMTERS)
    // > When in a "standby" type of state
    battery_voltage_standby_empty  =  3.30f;
    battery_voltage_standby_full   =  4.20f;
    // > When in a "flying" type of state
    battery_voltage_flying_empty   =  2.60f;
    battery_voltage_flying_full    =  3.70f;

    // SET THE STARTING RADIO STATUS TO BE: DISCONNECTED
    // > this also updates the image for the "rf_status_label", "battery_voltage_lineEdit", and "battery_status_label"
    setCrazyRadioStatus(DISCONNECTED);
    // SET THE STARTING FLYING STATE STATUS TO BE: MOTORS OFF
    // > this also updates the image for the "flying_state_label"
    setFlyingState(CMD_CRAZYFLY_MOTORS_OFF);
    // SET THE DEFAULT NAME FOR THE SELECTED CONTROLLER
    setControllerEnabled(SAFE_CONTROLLER);

#ifdef CATKIN_MAKE
    //m_rosNodeThread = new rosNodeThread(argc, argv, "coordinatorRowGUI");
    //m_rosNodeThread->init();

    //m_ros_namespace = ros::this_node::getNamespace();

    //qRegisterMetaType<ptrToMessage>("ptrToMessage");
    //QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));

    //ros::NodeHandle nodeHandle(m_ros_namespace);

    // communication with PPS Client, just to make it possible to communicate through terminal also we use PPSClient's name
    //ros::NodeHandle nh_PPSClient(m_ros_namespace + "/PPSClient");
    //ros::NodeHandle nh_PPSClient("PPSClient");


    // LET THE USER KNOW WHAT THE BASE NAMESPACE IS
    ROS_INFO_STREAM("[Coordinator Row GUI] using base namespace: " << ros_base_namespace.c_str() << ", for agentID = " << my_agentID);

    // DEBUGGING FOR NAMESPACES
    //std::string temp_ros_namespace = ros::this_node::getNamespace();
    //ROS_INFO_STREAM("[Coordinator Row GUI] compared to: ros::this_node::getNamespace() = " << temp_ros_namespace.c_str());

    // CREATE A NODE HANDLE TO THE BASE NAMESPACE
    ros::NodeHandle base_nodeHandle(ros_base_namespace);

    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    // SUBSCRIBERS AND PUBLISHERS:
    // > For Crazyradio commands based on button clicks
    crazyRadioCommandPublisher = base_nodeHandle.advertise<std_msgs::Int32>("PPSClient/crazyRadioCommand", 1);
    // > For updating the "rf_status_label" picture
    crazyRadioStatusSubscriber = base_nodeHandle.subscribe("CrazyRadio/CrazyRadioStatus", 1, &CoordinatorRow::crazyRadioStatusCallback, this);
    // > For updating the current battery voltage
    batteryVoltageSubscriber = base_nodeHandle.subscribe("CrazyRadio/CFBattery", 1, &CoordinatorRow::batteryVoltageCallback, this);
    // > For updating the current battery state
    batteryStateSubscriber = base_nodeHandle.subscribe("PPSClient/batteryState", 1, &CoordinatorRow::batteryStateChangedCallback, this);
    // > For Flying state commands based on button clicks
    flyingStateCommandPublisher = base_nodeHandle.advertise<d_fall_pps::IntWithHeader>("PPSClient/Command", 1);
    // > For updating the "flying_state_label" picture
    flyingStateSubscriber = base_nodeHandle.subscribe("PPSClient/flyingState", 1, &CoordinatorRow::flyingStateChangedCallback, this);
    // > For changes in the database that defines {agentID,cfID,flying zone} links
    databaseChangedSubscriber = dfall_root_nodeHandle.subscribe("/my_GUI/DBChanged", 1, &CoordinatorRow::databaseChangedCallback, this);;
    centralManagerDatabaseService = dfall_root_nodeHandle.serviceClient<CMQuery>("CentralManagerService/Query", false);
    // > For updating the controller that is currently operating
    controllerUsedSubscriber = base_nodeHandle.subscribe("PPSClient/controllerUsed", 1, &CoordinatorRow::controllerUsedChangedCallback, this);

#endif

    // FURTHER INITILIASATIONS NEED TO OCCUR AFTER THE ROS RELATED
    // INITIALISATIONS ARE COMPLETE
    loadCrazyflieContext();

    // FOR DEBUGGING:
    //ui->shouldCoordinate_checkBox->setText(my_agentID_as_string);
    //ui->shouldCoordinate_checkBox->setText(QString::fromStdString(base_namespace));
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





#ifdef CATKIN_MAKE
// PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

// > For the Battery Voltage
void CoordinatorRow::crazyRadioStatusCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[Coordinator Row GUI] Crazy Radio Status Callback called for agentID = " << my_agentID);
    setCrazyRadioStatus( msg.data );
}
#endif


// PRIVATE METHODS FOR SETTING PROPERTIES

void CoordinatorRow::setCrazyRadioStatus(int new_radio_status)
{
    // add more things whenever the status is changed
    switch(new_radio_status)
    {
        case CONNECTED:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIOSTATUS LABEL
            my_rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_connected_pixmap(":/images/rf_connected.png");
            ui->rf_status_label->setPixmap(rf_connected_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            my_rf_status_label_mutex.unlock();
            // ENABLE THE REMAINDER OF THE GUI
            my_battery_state_mutex.lock();
            if (my_battery_state == BATTERY_STATE_NORMAL)
            {
                enableFlyingStateButtons();
            }
            my_battery_state_mutex.unlock();

            break;
        }

        case CONNECTING:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIO STATUS LABEL
            my_rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_connecting_pixmap(":/images/rf_connecting.png");
            ui->rf_status_label->setPixmap(rf_connecting_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            my_rf_status_label_mutex.unlock();
            break;
        }

        case DISCONNECTED:
        {
            // SET THE APPROPRIATE IMAGE FOR THE RADIO STATUS LABEL
            my_rf_status_label_mutex.lock();
            //ui->rf_status_label->clear();
            QPixmap rf_disconnected_pixmap(":/images/rf_disconnected.png");
            ui->rf_status_label->setPixmap(rf_disconnected_pixmap);
            ui->rf_status_label->setScaledContents(true);
            //ui->rf_status_label->update();
            my_rf_status_label_mutex.unlock();
            // SET THE BATTERY VOLTAGE FIELD TO BE BLANK
            QString qstr = "-.-- V";
            my_battery_voltage_lineEdit_mutex.lock();
            ui->battery_voltage_lineEdit->setText(qstr);
            my_battery_voltage_lineEdit_mutex.unlock();
            // SET THE APPROPRIATE IMAGE FOR THE BATTERY STATUS LABEL
            // > Lock the mutex for accessing both "my_battery_status_label_image_current_index"
            //   and "ui->battery_status_label"
            my_battery_status_label_mutex.lock();
            if (my_battery_status_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_UNKNOWN)
            {
                ui->battery_status_label->clear();
                QPixmap battery_unknown_pixmap(":/images/battery_unknown.png");
                ui->battery_status_label->setPixmap(battery_unknown_pixmap);
                ui->battery_status_label->setScaledContents(true);
                my_battery_status_label_image_current_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
                ui->battery_status_label->update();
            }
            my_battery_status_label_mutex.unlock();
            // DISABLE THE REMAINDER OF THE GUI
            disableFlyingStateButtons();
            break;
        }

        default:
        {
            break;
        }
    }
    my_radio_status = new_radio_status;
}



#ifdef CATKIN_MAKE
// PRIVATE CALLBACKS IN RESPONSE TO ROS MESSAGES

// > For the Battery Voltage
void CoordinatorRow::batteryVoltageCallback(const std_msgs::Float32& msg)
{
    setBatteryVoltageTextAndImage( msg.data );
}


void CoordinatorRow::batteryStateChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[Coordinator Row GUI] Battery State Changed Callback called for agentID = " << my_agentID);
    setBatteryState( msg.data );
}
#endif



// PRIVATE METHODS FOR SETTING PROPERTIES

// > For updating the battery state
void CoordinatorRow::setBatteryState(int new_battery_state)
{
    // LOCK THE MUTEX FOR THE WHOLE SWITCH CASE STATEMENT
    my_battery_state_mutex.lock();
    // Switch depending the the new battery state provided
    switch(new_battery_state)
    {
        case BATTERY_STATE_LOW:
        {
            // MAKE UNAVAILABLE THE BUTTONS FOR ENABLING AND DISABLING FLIGHT
            disableFlyingStateButtons();

            // SET THE CLASS VARIABLE FOR TRACKING THE BATTERY STATE
            my_battery_state = BATTERY_STATE_LOW;
            break;
        }

        case BATTERY_STATE_NORMAL:
        {
            // MAKE UNAVAILABLE THE BUTTONS FOR ENABLING AND DISABLING FLIGHT
            enableFlyingStateButtons();

            // SET THE CLASS VARIABLE FOR TRACKING THE BATTERY STATE
            my_battery_state = BATTERY_STATE_NORMAL;
            break;
        }

        default:
            break;
    }
    // UNLOCK THE MUTEX
    my_battery_state_mutex.unlock();
}

// > For the battery voltage label and image
void CoordinatorRow::setBatteryVoltageTextAndImage(float battery_voltage)
{
    setBatteryVoltageText( battery_voltage );
    setBatteryVoltageImage( battery_voltage );
}

// > For updating the battery voltage shown in the UI elements of "battery_voltage_lineEdit"
void CoordinatorRow::setBatteryVoltageText(float battery_voltage)
{
    // Lock the mutex
    my_battery_voltage_lineEdit_mutex.lock();
    // Construct the text string
    QString qstr = "";
    qstr.append(QString::number(battery_voltage, 'f', 2));
    qstr.append(" V");
    // Set the text to the battery voltage line edit
    ui->battery_voltage_lineEdit->setText(qstr);
    // Unlock the mutex
    my_battery_voltage_lineEdit_mutex.unlock();
}

// > For updating the battery voltage shown in the UI elements of "battery_status_label"
void CoordinatorRow::setBatteryVoltageImage(float battery_voltage)
{
    // COMPUTE THE BATTERY VOLTAGE AS A PERCENTAGE
    float battery_voltage_percentage = fromVoltageToPercent(battery_voltage);

    // CONVERT THE VOLTAGE PERCENTAGE TO AN INDEX OF WHICH BATTERY LEVEL IMAGE TO DISPLAY
    // > Initialise a local variable that will be set in the switch case below
    int new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
    // > Initialise a local variable for the string of which image to use
    QString qstr_new_image = "";
    qstr_new_image.append(":/images/");
    // > Get the value of the "my_battery_state" variable into a local variable
    my_battery_state_mutex.lock();
    int local_copy_of_my_battery_state = my_battery_state;
    my_battery_state_mutex.unlock();
    // > Switch based on the current battery state, first locking the mutex for accessing
    //   both "my_battery_status_label_image_current_index" and "ui->battery_status_label"
    my_battery_status_label_mutex.lock();
    switch(local_copy_of_my_battery_state)
    {
        // WHEN THE BATTERY IS IN A LOW STATE
        case BATTERY_STATE_LOW:
        {
            new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_EMPTY;
            qstr_new_image.append("battery_empty.png");
            break;
        }

        // WHEN THE BATTERY IS IN A NORMAL STATE
        case BATTERY_STATE_NORMAL:
        {

            if (
                ((my_battery_status_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_EMPTY) && (battery_voltage_percentage <= 0.0f))
                ||
                ((my_battery_status_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_EMPTY) && (battery_voltage_percentage <= 2.0f))
            )
            {
                new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_EMPTY;
                qstr_new_image.append("battery_empty.png");
            }
            else if (
                ((my_battery_status_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_20) && (battery_voltage_percentage <= 20.0f))
                ||
                ((my_battery_status_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_20) && (battery_voltage_percentage <= 22.0f))
            )
            {
                new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_20;
                qstr_new_image.append("battery_20.png");
            }
            else if (
                ((my_battery_status_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_40) && (battery_voltage_percentage <= 40.0f))
                ||
                ((my_battery_status_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_40) && (battery_voltage_percentage <= 42.0f))
            )
            {
                new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_40;
                qstr_new_image.append("battery_40.png");
            }
            else if (
                ((my_battery_status_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_60) && (battery_voltage_percentage <= 60.0f))
                ||
                ((my_battery_status_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_60) && (battery_voltage_percentage <= 62.0f))
            )
            {
                new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_60;
                qstr_new_image.append("battery_60.png");
            }
            else if (
                ((my_battery_status_label_image_current_index != BATTERY_LABEL_IMAGE_INDEX_80) && (battery_voltage_percentage <= 80.0f))
                ||
                ((my_battery_status_label_image_current_index == BATTERY_LABEL_IMAGE_INDEX_80) && (battery_voltage_percentage <= 82.0f))
            )
            {
                new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_80;
                qstr_new_image.append("battery_80.png");
            }
            else
            {
                new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_FULL;
                qstr_new_image.append("battery_full.png");
            }
            break;
        }

        default:
        {
            new_battery_label_image_index = BATTERY_LABEL_IMAGE_INDEX_UNKNOWN;
            qstr_new_image.append("battery_unknown.png");
            break;
        }
    }
    // UPDATE THE IMAGE DISPLAYED BASED ON THE "new index"
    // > Only if it is different from the current index
    if (my_battery_status_label_image_current_index != new_battery_label_image_index)
    {
        // SET THE IMAGE FOR THE BATTERY STATUS LABEL
        ui->battery_status_label->clear();
        QPixmap battery_image_pixmap(qstr_new_image);
        ui->battery_status_label->setPixmap(battery_image_pixmap);
        ui->battery_status_label->setScaledContents(true);
        my_battery_status_label_image_current_index = new_battery_label_image_index;
        ui->battery_status_label->update();
    }
    // Finally unlock the mutex
    my_battery_status_label_mutex.unlock();
}


// > For converting a voltage to a percentage, depending on the current "my_flying_state" value
float CoordinatorRow::fromVoltageToPercent(float voltage)
{
    // INITIALISE THE LOCAL VARIABLE FOR THE VOLTAGE WHEN FULL/EMPTY
    float voltage_when_full;
    float voltage_when_empty;

    // COMPUTE THE PERCENTAGE DIFFERENTLY DEPENDING ON
    // THE CURRENT FLYING STATE
    // > First lock the mutex before accessing the "my_flying_state" variable
    my_flying_state_mutex.lock();
    if (my_flying_state == STATE_MOTORS_OFF)
    {
        // Voltage limits for a "standby" type of state
        voltage_when_empty = battery_voltage_standby_empty;
        voltage_when_full  = battery_voltage_standby_full;
    }
    else
    {
        // Voltage limits for a "flying" type of state
        voltage_when_empty = battery_voltage_flying_empty;
        voltage_when_full  = battery_voltage_flying_full;
    }
    // > Unlock the mutex
    my_flying_state_mutex.unlock();

    // COMPUTE THE PERCENTAGE
    float percentage = 100.0f * (voltage-voltage_when_empty)/(voltage_when_full-voltage_when_empty);

    // CLIP THE PERCENTAGE TO BE BETWEEN [0,100]
    // > This should not happen to often
    if(percentage > 100.0f)
    {
        percentage = 100.0f;
    }
    if(percentage < 0.0f)
    {
        percentage = 0.0f;
    }

    // RETURN THE PERCENTAGE
    return percentage;
}




// RESPONDING TO CHANGES IN THE FLYING STATE
#ifdef CATKIN_MAKE
void CoordinatorRow::flyingStateChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[Coordinator Row GUI] Flying State Changed Callback called for agentID = " << my_agentID);
    setFlyingState(msg.data);
}
#endif

void CoordinatorRow::setFlyingState(int new_flying_state)
{
    // PUT THE CURRENT STATE INTO THE CLASS VARIABLE
    my_flying_state_mutex.lock();
    my_flying_state = new_flying_state;
    my_flying_state_mutex.unlock();

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


// RESPONDING TO CHANGES IN THE DATABASE
#ifdef CATKIN_MAKE
// > For the notification that the database was changes, received on the "DatabaseChangedSubscriber"
void CoordinatorRow::databaseChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[Coordinator Row GUI] Database Changed Callback called for agentID = " << my_agentID);
    loadCrazyflieContext();
}
#endif

// > For loading the "context" for this agent, i.e., the {agentID,cfID,flying zone} tuple
void CoordinatorRow::loadCrazyflieContext()
{
    QString qstr_crazyflie_name = "";
#ifdef CATKIN_MAKE
    CMQuery contextCall;
    contextCall.request.studentID = my_agentID;
    //ROS_INFO_STREAM("StudentID:" << my_agentID);

    centralManagerDatabaseService.waitForExistence(ros::Duration(-1));

    if(centralManagerDatabaseService.call(contextCall))
    {
        my_context = contextCall.response.crazyflieContext;
        ROS_INFO_STREAM("[Coordinator Row GUI] CrazyflieContext:\n" << my_context);

        qstr_crazyflie_name.append(QString::fromStdString(my_context.crazyflieName));
    }
    else
    {
        ROS_ERROR_STREAM("[Coordinator Row GUI] Failed to load context for agentID = " << my_agentID);
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
    qstr_for_checkbox_label.append(QString::number(my_agentID));
    qstr_for_checkbox_label.append(", CF");
    qstr_for_checkbox_label.append(qstr_crazyflie_name);
    ui->shouldCoordinate_checkBox->setText(qstr_for_checkbox_label);

}


#ifdef CATKIN_MAKE
// > For the controller currently operating, received on "controllerUsedSubscriber"
void CoordinatorRow::controllerUsedChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[Coordinator Row GUI] Controller Used Changed Callback called for agentID = " << my_agentID);
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




//    ------------------------------------------------------------------- //
// # RF Crazyradio Connect Disconnect
void CoordinatorRow::on_rf_connect_button_clicked()
{
#ifdef CATKIN_MAKE
    std_msgs::Int32 msg;
    msg.data = CMD_RECONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("[FLYING AGENT GUI] Command to RF reconnect published");
#endif
}

void CoordinatorRow::on_rf_disconnect_button_clicked()
{
#ifdef CATKIN_MAKE
    std_msgs::Int32 msg;
    msg.data = CMD_DISCONNECT;
    this->crazyRadioCommandPublisher.publish(msg);
    ROS_INFO("[FLYING AGENT GUI] Command to RF disconnect published");
#endif
}

//    ------------------------------------------------------------------- //
// # Take off, land, motors off
void CoordinatorRow::on_enable_flying_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_TAKE_OFF;
    this->flyingStateCommandPublisher.publish(msg);
#endif
}

void CoordinatorRow::on_disable_flying_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_LAND;
    this->flyingStateCommandPublisher.publish(msg);
#endif
}

void CoordinatorRow::on_motors_off_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_MOTORS_OFF;
    this->flyingStateCommandPublisher.publish(msg);
#endif
}
