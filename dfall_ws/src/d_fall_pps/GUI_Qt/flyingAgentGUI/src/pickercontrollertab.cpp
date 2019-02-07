#include "pickercontrollertab.h"
#include "ui_pickercontrollertab.h"

PickerControllerTab::PickerControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PickerControllerTab)
{
    ui->setupUi(this);


    // HIDE ALL THE "GREEN FRAMES"
    // > These indicate which state is currently active
    ui->frame_goto_start_active->setVisible(true);
    ui->frame_attach_active    ->setVisible(true);
    ui->frame_lift_up_active   ->setVisible(true);
    ui->frame_goto_end_active  ->setVisible(true);
    ui->frame_put_down_active  ->setVisible(true);
    ui->frame_squat_active     ->setVisible(true);
    ui->frame_jump_active      ->setVisible(true);
    ui->frame_standby_active   ->setVisible(true);
    // > Make them all white
    ui->frame_goto_start_active->setStyleSheet("background-color:white;");
    ui->frame_attach_active    ->setStyleSheet("background-color:white;");
    ui->frame_lift_up_active   ->setStyleSheet("background-color:white;");
    ui->frame_goto_end_active  ->setStyleSheet("background-color:white;");
    ui->frame_put_down_active  ->setStyleSheet("background-color:white;");
    ui->frame_squat_active     ->setStyleSheet("background-color:white;");
    ui->frame_jump_active      ->setStyleSheet("background-color:white;");
    ui->frame_standby_active   ->setStyleSheet("background-color:white;");
    

    // SET DEFAULTS FOR THE INCREMENT
    ui->lineEdit_increment_x   ->setText(QString::number( DEFAULT_INCREMENT_POSITION_XY,   'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_increment_y   ->setText(QString::number( DEFAULT_INCREMENT_POSITION_XY,   'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_increment_z   ->setText(QString::number( DEFAULT_INCREMENT_POSITION_Z,    'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_increment_yaw ->setText(QString::number( DEFAULT_INCREMENT_ANGLE_DEGREES, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    ui->lineEdit_increment_mass->setText(QString::number( DEFAULT_INCREMENT_MASS_GRAMS,    'f', DECIMAL_PLACES_MASS_GRAMS));

    // SET THE INITIAL CHECKBOX STATE FOR SMOOTHING
    ui->checkbox_goto_start->setChecked(true);
    ui->checkbox_attach    ->setChecked(true);
    ui->checkbox_lift_up   ->setChecked(true);
    ui->checkbox_goto_end  ->setChecked(true);
    ui->checkbox_put_down  ->setChecked(true);
    ui->checkbox_squat     ->setChecked(true);
    ui->checkbox_jump      ->setChecked(false);
    ui->checkbox_standby   ->setChecked(true);
    ui->checkbox_current   ->setChecked(false);

    // SET THE INITIAL CHECKBOX STATE FOR PUBLISHING EVERY CHANGED VALUE
    ui->checkbox_should_publish_value_changed->setChecked(true);

    // HIDE ALL THE "RED FRAMES"
    // > These indicate when an occuled measurement is recieved
    ui->frame_x_unavailable  ->setVisible(false);
    ui->frame_y_unavailable  ->setVisible(false);
    ui->frame_z_unavailable  ->setVisible(false);
    ui->frame_yaw_unavailable->setVisible(false);

    // SET DEFAULTS FOR ALL THE {x,y,z,yaw,mass} LineEdits
    // > For Goto Start
    ui->lineEdit_goto_start_x  ->setText(QString::number( PICKER_DEFAULT_X,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_goto_start_y  ->setText(QString::number( PICKER_DEFAULT_Y,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_goto_start_z  ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_goto_start_yaw->setText(QString::number( PICKER_DEFAULT_YAW_DEGREES, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // > For Attach
    ui->lineEdit_attach_z      ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    // > For Lift Up
    ui->lineEdit_lift_up_z     ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_lift_up_mass  ->setText(QString::number( PICKER_DEFAULT_MASS_GRAMS,  'f', DECIMAL_PLACES_MASS_GRAMS));
    // > For Goto End
    ui->lineEdit_goto_end_x    ->setText(QString::number( PICKER_DEFAULT_X,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_goto_end_y    ->setText(QString::number( PICKER_DEFAULT_Y,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_goto_end_yaw  ->setText(QString::number( PICKER_DEFAULT_YAW_DEGREES, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // > For Put Down
    ui->lineEdit_put_down_z    ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    // > For Squat
    ui->lineEdit_squat_z       ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    // > For Jump
    ui->lineEdit_jump_z        ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    // > For Standby
    ui->lineEdit_standby_x     ->setText(QString::number( PICKER_DEFAULT_X,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_standby_y     ->setText(QString::number( PICKER_DEFAULT_Y,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_standby_z     ->setText(QString::number( PICKER_DEFAULT_Z,           'f', DECIMAL_PLACES_POSITION));
    ui->lineEdit_standby_yaw   ->setText(QString::number( PICKER_DEFAULT_YAW_DEGREES, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    ui->lineEdit_standby_mass  ->setText(QString::number( PICKER_DEFAULT_MASS_GRAMS,  'f', DECIMAL_PLACES_MASS_GRAMS));





#ifdef CATKIN_MAKE

    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[PICKER CONTROLLER TAB GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[PICKER CONTROLLER TAB GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE THE REQUEST SETPOINT CHANGE PUBLISHER
    requestSetpointChangePublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::SetpointWithHeader>("PickerControllerService/RequestSetpointChange", 1);

    // SUBSCRIBE TO SETPOINT CHANGES
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        setpointChangedSubscriber = nodeHandle_for_this_gui.subscribe("PickerControllerService/SetpointChanged", 1, &PickerControllerTab::setpointChangedCallback, this);
    }

    // GET THE CURRENT SETPOINT
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = nodeHandle_for_this_gui.serviceClient<d_fall_pps::GetSetpointService>("PickerControllerService/GetCurrentSetpoint", false);
        d_fall_pps::GetSetpointService getSetpointCall;
        getSetpointCall.request.data = 0;
        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(2.0));
        if(getCurrentSetpointServiceClient.call(getSetpointCall))
        {
            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
        }
        else
        {
            // Inform the user
            ROS_INFO("[PICKER CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }
    }

#endif
}

PickerControllerTab::~PickerControllerTab()
{
    delete ui;
}






// ---------------------------------------------------------- //
// ---------------------------------------------------------- //
// PUBLISH CHANGES IN STATE AND SETPOINT
// ---------------------------------------------------------- //
// ---------------------------------------------------------- //

void PickerControllerTab::publish_setpoint_if_current_state_matches(QVector<int> state_to_match)
{
    m_current_picker_state_mutex.lock();
    if ( state_to_match.contains(m_current_picker_state) )
    {
        publish_request_setpoint_change_for_state(m_current_picker_state);
    }
    m_current_picker_state_mutex.unlock();
}

void PickerControllerTab::publish_request_setpoint_change_for_state(int state_to_publish)
{
    // Initiliase variables for the setpoing
    bool req_should_smooth;
    float req_x;
    float req_y;
    float req_z;
    float req_yaw;
    float req_mass;


    // Switch between the possible states and get the respectively
    // values for the setpoint
    switch (state_to_publish)
    {
    case PICKER_STATE_STANDBY:
    {
        req_should_smooth = ui->checkbox_standby->isChecked();
        req_x    = (ui->lineEdit_standby_x   ->text()).toFloat();
        req_y    = (ui->lineEdit_standby_y   ->text()).toFloat();
        req_z    = (ui->lineEdit_standby_z   ->text()).toFloat();
        req_yaw  = (ui->lineEdit_standby_yaw ->text()).toFloat();
        req_mass = (ui->lineEdit_standby_mass->text()).toFloat();
        break;
    }
    case PICKER_STATE_GOTO_START:
    {
        req_should_smooth = ui->checkbox_goto_start->isChecked();
        req_x    = (ui->lineEdit_goto_start_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_start_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_goto_start_z  ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_start_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_standby_mass  ->text()).toFloat();
        break;
    }
    case PICKER_STATE_ATTACH:
    {
        req_should_smooth = ui->checkbox_attach->isChecked();
        req_x    = (ui->lineEdit_goto_start_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_start_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_attach_z      ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_start_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_standby_mass  ->text()).toFloat();
        break;
    }
    case PICKER_STATE_LIFT_UP:
    {
        req_should_smooth = ui->checkbox_lift_up->isChecked();
        req_x    = (ui->lineEdit_goto_start_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_start_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_lift_up_z     ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_start_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_lift_up_mass  ->text()).toFloat();
        break;
    }
    case PICKER_STATE_GOTO_END:
    {
        req_should_smooth = ui->checkbox_goto_end->isChecked();
        req_x    = (ui->lineEdit_goto_end_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_end_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_lift_up_z   ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_end_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_lift_up_mass->text()).toFloat();
        break;
    }
    case PICKER_STATE_PUT_DOWN:
    {
        req_should_smooth = ui->checkbox_put_down->isChecked();
        req_x    = (ui->lineEdit_goto_end_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_end_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_put_down_z  ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_end_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_lift_up_mass->text()).toFloat();
        break;
    }
    case PICKER_STATE_SQUAT:
    {
        req_should_smooth = ui->checkbox_squat->isChecked();
        req_x    = (ui->lineEdit_goto_end_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_end_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_squat_z     ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_end_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_standby_mass->text()).toFloat();
        break;
    }
    case PICKER_STATE_JUMP:
    {
        req_should_smooth = ui->checkbox_jump->isChecked();
        req_x    = (ui->lineEdit_goto_end_x  ->text()).toFloat();
        req_y    = (ui->lineEdit_goto_end_y  ->text()).toFloat();
        req_z    = (ui->lineEdit_jump_z      ->text()).toFloat();
        req_yaw  = (ui->lineEdit_goto_end_yaw->text()).toFloat();
        req_mass = (ui->lineEdit_standby_mass->text()).toFloat();
        break;
    }
    default:
    {
        req_should_smooth = true;
        req_x    = PICKER_DEFAULT_X;
        req_y    = PICKER_DEFAULT_Y;
        req_z    = PICKER_DEFAULT_Z;
        req_yaw  = PICKER_DEFAULT_YAW_DEGREES;
        req_mass = PICKER_DEFAULT_MASS_GRAMS;
        break;
    }
    }

    // Publish a ROS message with the setpoint to be requested
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    d_fall_pps::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the (x,y,z,yaw,mass) values
    msg.x    = req_x;
    msg.y    = req_y;
    msg.z    = req_z;
    msg.yaw  = req_yaw * DEG2RAD;
    msg.mass = req_mass;

    // Fill in the "should smooth" value
    msg.isChecked = req_should_smooth;

    // Fill in the "picker state" value
    msg.buttonID = state_to_publish;

    // Publish the setpoint
    this->requestSetpointChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[PICKER CONTROLLER GUI] Published request for setpoint change to: [" << req_x << ", "<< req_y << ", "<< req_z << ", "<< req_yaw << "], with {mass,smooth,state} = " << req_mass << ", "<< req_should_smooth << ", "<< state_to_publish );
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[PICKER CONTROLLER GUI] would publish request for: [" << req_x << ", "<< req_y << ", "<< req_z << ", "<< req_yaw << "], with {mass,smooth,state} = " << req_mass << ", "<< req_should_smooth << ", "<< state_to_publish;
#endif
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
void PickerControllerTab::setpointChangedCallback(const d_fall_pps::SetpointWithHeader& newSetpoint)
{
    // INITIALISE A STRING VARIABLE FOR ADDING THE "+"
    QString qstr = "";

    // EXTRACT THE SETPOINT
    float x   = newSetpoint.x;
    float y   = newSetpoint.y;
    float z   = newSetpoint.z;
    float yaw = newSetpoint.yaw;

    float mass    = newSetpoint.yaw;
    bool  smooth  = newSetpoint.isChecked;
    int   state   = newSetpoint.buttonID;

    // UPDATE THE SETPOINT LineEdits
    if (x < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_current_x->setText(qstr + QString::number( x, 'f', DECIMAL_PLACES_POSITION));
    if (y < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_current_y->setText(qstr + QString::number( y, 'f', DECIMAL_PLACES_POSITION));
    if (z < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_current_z->setText(qstr + QString::number( z, 'f', DECIMAL_PLACES_POSITION));

    if (yaw < 0.0f) qstr = ""; else qstr = "+";
    ui->lineEdit_current_yaw->setText(qstr + QString::number( yaw * RAD2DEG, 'f', DECIMAL_PLACES_ANGLE_DEGREES));

    ui->lineEdit_current_mass->setText(qstr + QString::number( mass, 'f', DECIMAL_PLACES_MASS_GRAMS));

    ui->checkbox_current->setChecked(smooth);



    // MAKE THE "GREEN FRAME" VISIBLE ONLY FOR THE CURRENT STATE
    // Lock the mutex that we will access "m_current_picker_state"
    m_current_picker_state_mutex.lock();

    // Put things into more readable variables
    int new_state = state;
    int previous_state = m_current_picker_state;

    // Update the class variable for tracking the state
    if ( new_state != previous_state )
        m_current_picker_state = new_state;

    // Change the "GREEN FRAME" if the state changed
    if ( new_state != previous_state )
    {
        // Make the new one green
        switch (new_state)
        {
        case PICKER_STATE_STANDBY:
        {
            ui->frame_standby_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_GOTO_START:
        {
            ui->frame_goto_start_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_ATTACH:
        {
            ui->frame_attach_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_LIFT_UP:
        {
            ui->frame_lift_up_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_GOTO_END:
        {
            ui->frame_goto_end_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_PUT_DOWN:
        {
            ui->frame_put_down_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_SQUAT:
        {
            ui->frame_squat_active->setStyleSheet("background-color:green;");
            break;
        }
        case PICKER_STATE_JUMP:
        {
            ui->frame_jump_active->setStyleSheet("background-color:green;");
            break;
        }
        default:
        {
            break;
        }
        }

        // And the old one not white
        switch (previous_state)
        {
        case PICKER_STATE_STANDBY:
        {
            ui->frame_standby_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_GOTO_START:
        {
            ui->frame_goto_start_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_ATTACH:
        {
            ui->frame_attach_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_LIFT_UP:
        {
            ui->frame_lift_up_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_GOTO_END:
        {
            ui->frame_goto_end_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_PUT_DOWN:
        {
            ui->frame_put_down_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_SQUAT:
        {
            ui->frame_squat_active->setStyleSheet("background-color:white;");
            break;
        }
        case PICKER_STATE_JUMP:
        {
            ui->frame_jump_active->setStyleSheet("background-color:white;");
            break;
        }
        default:
        {
            break;
        }
        }
    }
    else
    {
        // NOTHING TO CHANGE
    }
    // Unlock the mutex for accessing "m_current_picker_state"
    m_current_picker_state_mutex.unlock();
}
#endif





//    ----------------------------------------------------------------------------------
//    PPPP    OOO    SSSS  EEEEE     DDDD     A    TTTTT    A
//    P   P  O   O  S      E         D   D   A A     T     A A
//    PPPP   O   O   SSS   EEE       D   D  A   A    T    A   A
//    P      O   O      S  E         D   D  AAAAA    T    AAAAA
//    P       OOO   SSSS   EEEEE     DDDD   A   A    T    A   A
//    ----------------------------------------------------------------------------------


void PickerControllerTab::setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool occluded)
{
    if (!occluded)
    {
        // INITIALISE A STRING VARIABLE FOR ADDING THE "+"
        QString qstr = "";
        // UPDATE THE MEASUREMENT COLUMN
        if (x < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_x->setText(qstr + QString::number( x, 'f', DECIMAL_PLACES_POSITION));
        if (y < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_y->setText(qstr + QString::number( y, 'f', DECIMAL_PLACES_POSITION));
        if (z < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_z->setText(qstr + QString::number( z, 'f', DECIMAL_PLACES_POSITION));

        if (yaw < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_measured_yaw->setText(qstr + QString::number( yaw   * RAD2DEG, 'f', DECIMAL_PLACES_ANGLE_DEGREES));

        // GET THE CURRENT SETPOINT
        float error_x   = x   - (ui->lineEdit_current_x->text()  ).toFloat();
        float error_y   = y   - (ui->lineEdit_current_y->text()  ).toFloat();
        float error_z   = z   - (ui->lineEdit_current_z->text()  ).toFloat();
        float error_yaw = yaw - (ui->lineEdit_current_yaw->text()).toFloat();

        // UPDATE THE ERROR COLUMN
        if (error_x < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_x->setText(qstr + QString::number( error_x, 'f', DECIMAL_PLACES_POSITION));
        if (error_y < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_y->setText(qstr + QString::number( error_y, 'f', DECIMAL_PLACES_POSITION));
        if (error_z < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_z->setText(qstr + QString::number( error_z, 'f', DECIMAL_PLACES_POSITION));

        if (error_yaw < 0.0f) qstr = ""; else qstr = "+";
        ui->lineEdit_error_yaw->setText(qstr + QString::number( error_yaw * RAD2DEG, 'f', DECIMAL_PLACES_ANGLE_DEGREES));

        // Ensure the red frames are not visible
        if ( ui->frame_x_unavailable->isVisible() )
        {
            ui->frame_x_unavailable  ->setVisible(false);
            ui->frame_y_unavailable  ->setVisible(false);
            ui->frame_z_unavailable  ->setVisible(false);
            ui->frame_yaw_unavailable->setVisible(false);
        }
    }
    else
    {
        // Make visible the red frames to indicate occluded
        if ( !(ui->frame_x_unavailable->isVisible()) )
        {
            ui->frame_x_unavailable  ->setVisible(true);
            ui->frame_y_unavailable  ->setVisible(true);
            ui->frame_z_unavailable  ->setVisible(true);
            ui->frame_yaw_unavailable->setVisible(true);
        }
    }
}


void PickerControllerTab::poseDataUnavailableSlot()
{
    ui->lineEdit_measured_x->setText("xx.xx");
    ui->lineEdit_measured_y->setText("xx.xx");
    ui->lineEdit_measured_z->setText("xx.xx");

    ui->lineEdit_measured_yaw->setText("xx.xx");

    ui->lineEdit_error_x->setText("xx.xx");
    ui->lineEdit_error_y->setText("xx.xx");
    ui->lineEdit_error_z->setText("xx.xx");
    ui->lineEdit_error_yaw->setText("xx.xx");
}





// ---------------------------------------------------------- //
// ---------------------------------------------------------- //
// STATE BUTTONS CLICKED
// ---------------------------------------------------------- //
// ---------------------------------------------------------- //

void PickerControllerTab::on_button_goto_start_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_GOTO_START);
}

void PickerControllerTab::on_button_attach_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_ATTACH);
}

void PickerControllerTab::on_button_lift_up_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_LIFT_UP);
}

void PickerControllerTab::on_button_goto_end_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_GOTO_END);
}

void PickerControllerTab::on_button_put_down_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_PUT_DOWN);
}

void PickerControllerTab::on_button_squat_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_SQUAT);
}

void PickerControllerTab::on_button_jump_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_JUMP);
}

void PickerControllerTab::on_button_standby_clicked()
{
    // Directly call the function that publishes state (and setpoint) changes
    publish_request_setpoint_change_for_state(PICKER_STATE_STANDBY);
}





// ---------------------------------------------------------- //
// ---------------------------------------------------------- //
// CHECK BOXES CLICKED
// ---------------------------------------------------------- //
// ---------------------------------------------------------- //

void PickerControllerTab::on_checkbox_goto_start_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_attach_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_lift_up_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_goto_end_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_put_down_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_squat_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_jump_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_checkbox_standby_clicked()
{
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}





// ---------------------------------------------------------- //
// ---------------------------------------------------------- //
// INCREMENT BUTTONS CLICKED
// ---------------------------------------------------------- //
// ---------------------------------------------------------- //

// >> FOR GOTO START

void PickerControllerTab::on_button_goto_start_inc_minus_x_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_x->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_x ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_start_x->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_plus_x_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_x->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_x ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_start_x->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_minus_y_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_y->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_y ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_start_y->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_plus_y_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_y->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_y ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_start_y->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_start_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_start_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_minus_yaw_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_yaw->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_yaw ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_start_yaw->setText(QString::number( new_value, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_start_inc_plus_yaw_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_start_yaw->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_yaw ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_start_yaw->setText(QString::number( new_value, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR ATTACH

void PickerControllerTab::on_button_attach_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_attach_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_attach_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_attach_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_attach_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_attach_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR LIFT UP

void PickerControllerTab::on_button_lift_up_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_lift_up_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_lift_up_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_lift_up_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_lift_up_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_lift_up_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_lift_up_inc_minus_mass_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_lift_up_mass->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_mass ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_lift_up_mass->setText(QString::number( new_value, 'f', DECIMAL_PLACES_MASS_GRAMS));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_lift_up_inc_plus_mass_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_lift_up_mass->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_mass ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_lift_up_mass->setText(QString::number( new_value, 'f', DECIMAL_PLACES_MASS_GRAMS));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR GOTO END

void PickerControllerTab::on_button_goto_end_inc_minus_x_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_end_x->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_x ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_end_x->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_end_inc_plus_x_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_end_x->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_x ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_end_x->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_end_inc_minus_y_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_end_y->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_y ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_end_y->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_end_inc_plus_y_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_end_y->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_y ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_end_y->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_end_inc_minus_yaw_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_end_yaw->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_yaw ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_goto_end_yaw->setText(QString::number( new_value, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_goto_end_inc_plus_yaw_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_goto_end_yaw->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_yaw ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_goto_end_yaw->setText(QString::number( new_value, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR PUT DOWN

void PickerControllerTab::on_button_put_down_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_put_down_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_put_down_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_put_down_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_put_down_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_put_down_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR SQUAT

void PickerControllerTab::on_button_squat_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_squat_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_squat_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_squat_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_squat_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_squat_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR JUMP

void PickerControllerTab::on_button_jump_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_jump_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_jump_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_jump_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_jump_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_jump_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR STANDBY

void PickerControllerTab::on_button_stanby_inc_minus_x_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_x->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_x ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_standby_x->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_plus_x_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_x->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_x ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_standby_x->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_minus_y_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_y->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_y ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_standby_y->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_plus_y_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_y->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_y ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_standby_y->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_minus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_standby_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_plus_z_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_z->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_z ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_standby_z->setText(QString::number( new_value, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_minus_yaw_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_yaw->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_yaw ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_standby_yaw->setText(QString::number( new_value, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_plus_yaw_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_yaw->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_yaw ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_standby_yaw->setText(QString::number( new_value, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_minus_mass_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_mass->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_mass ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value - current_inc;
    ui->lineEdit_standby_mass->setText(QString::number( new_value, 'f', DECIMAL_PLACES_MASS_GRAMS));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_button_stanby_inc_plus_mass_clicked()
{
    // Get the value from the respective line edit and increment
    float current_value = (ui->lineEdit_standby_mass->text()).toFloat();
    float current_inc   = (ui->lineEdit_increment_mass ->text()).toFloat();
    // Compute the new value and set it back
    float new_value = current_value + current_inc;
    ui->lineEdit_standby_mass->setText(QString::number( new_value, 'f', DECIMAL_PLACES_MASS_GRAMS));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}





// ---------------------------------------------------------- //
// ---------------------------------------------------------- //
// LINE EDITS FINISHED EDITING
// ---------------------------------------------------------- //
// ---------------------------------------------------------- //

// >> FOR GOTO START

void PickerControllerTab::on_lineEdit_goto_start_x_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_start_x->text()).toFloat();
    ui->lineEdit_goto_start_x->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_goto_start_y_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_start_y->text()).toFloat();
    ui->lineEdit_goto_start_y->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_goto_start_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_start_z->text()).toFloat();
    ui->lineEdit_goto_start_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_goto_start_yaw_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_start_yaw->text()).toFloat();
    ui->lineEdit_goto_start_yaw->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR ATTACH

void PickerControllerTab::on_lineEdit_attach_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_attach_z->text()).toFloat();
    ui->lineEdit_attach_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR LIFT UP

void PickerControllerTab::on_lineEdit_lift_up_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_lift_up_z->text()).toFloat();
    ui->lineEdit_lift_up_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_lift_up_mass_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_lift_up_mass->text()).toFloat();
    ui->lineEdit_lift_up_mass->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_MASS_GRAMS));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT_UP);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR GOTO END

void PickerControllerTab::on_lineEdit_goto_end_x_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_end_x->text()).toFloat();
    ui->lineEdit_goto_end_x->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_goto_end_y_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_end_y->text()).toFloat();
    ui->lineEdit_goto_end_y->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_goto_end_yaw_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_goto_end_yaw->text()).toFloat();
    ui->lineEdit_goto_end_yaw->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_END);
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR PUT DOWN

void PickerControllerTab::on_lineEdit_put_down_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_put_down_z->text()).toFloat();
    ui->lineEdit_put_down_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_PUT_DOWN);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR SQUAT

void PickerControllerTab::on_lineEdit_squat_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_squat_z->text()).toFloat();
    ui->lineEdit_squat_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR JUMP

void PickerControllerTab::on_lineEdit_jump_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_jump_z->text()).toFloat();
    ui->lineEdit_jump_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR  STANDBY

void PickerControllerTab::on_lineEdit_standby_x_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_standby_x->text()).toFloat();
    ui->lineEdit_standby_x->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_standby_y_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_standby_y->text()).toFloat();
    ui->lineEdit_standby_y->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_standby_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_standby_z->text()).toFloat();
    ui->lineEdit_standby_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_standby_yaw_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_standby_yaw->text()).toFloat();
    ui->lineEdit_standby_yaw->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

void PickerControllerTab::on_lineEdit_standby_mass_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_standby_mass->text()).toFloat();
    ui->lineEdit_standby_mass->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_MASS_GRAMS));
    // Call the function to publish the new setpoint if appropriate
    if (ui->checkbox_should_publish_value_changed->isChecked())
    {
        QVector<int> states_for_which_this_change_applies;
        states_for_which_this_change_applies.append(PICKER_STATE_STANDBY);
        states_for_which_this_change_applies.append(PICKER_STATE_GOTO_START);
        states_for_which_this_change_applies.append(PICKER_STATE_ATTACH);
        states_for_which_this_change_applies.append(PICKER_STATE_SQUAT);
        states_for_which_this_change_applies.append(PICKER_STATE_JUMP);
        publish_setpoint_if_current_state_matches(states_for_which_this_change_applies);
    }
}

// >> FOR INCREMENTS

void PickerControllerTab::on_lineEdit_increment_x_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_increment_x->text()).toFloat();
    ui->lineEdit_increment_x->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
}

void PickerControllerTab::on_lineEdit_increment_y_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_increment_y->text()).toFloat();
    ui->lineEdit_increment_y->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
}

void PickerControllerTab::on_lineEdit_increment_z_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_increment_z->text()).toFloat();
    ui->lineEdit_increment_z->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_POSITION));
}

void PickerControllerTab::on_lineEdit_increment_yaw_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_increment_yaw->text()).toFloat();
    ui->lineEdit_increment_yaw->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_ANGLE_DEGREES));
}

void PickerControllerTab::on_lineEdit_increment_mass_editingFinished()
{
    // Get the value and set it back with two decimal places
    float value_entered = (ui->lineEdit_increment_mass->text()).toFloat();
    ui->lineEdit_increment_mass->setText(QString::number( value_entered, 'f', DECIMAL_PLACES_MASS_GRAMS));
}


//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void PickerControllerTab::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
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
        ros::ServiceClient getCurrentSetpointServiceClient = agent_base_nodeHandle.serviceClient<d_fall_pps::GetSetpointService>("PickerControllerService/GetCurrentSetpoint", false);
        d_fall_pps::GetSetpointService getSetpointCall;
        getSetpointCall.request.data = 0;
        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(2.0));
        if(getCurrentSetpointServiceClient.call(getSetpointCall))
        {
            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
        }
        else
        {
            // Inform the user
            ROS_INFO("[PICKER CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }

        // SUBSCRIBERS
        // > For receiving message that the setpoint was changed
        setpointChangedSubscriber = agent_base_nodeHandle.subscribe("PickerControllerService/SetpointChanged", 1, &PickerControllerTab::setpointChangedCallback, this);
    }
    else
    {
        // Unsubscribe
        setpointChangedSubscriber.shutdown();

        // Set information back to the default
        ui->lineEdit_current_x   ->setText("xx.xx");
        ui->lineEdit_current_y   ->setText("xx.xx");
        ui->lineEdit_current_z   ->setText("xx.xx");
        ui->lineEdit_current_yaw ->setText("xx.xx");
        ui->lineEdit_current_mass->setText("xx.xx");

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
void PickerControllerTab::fillSetpointMessageHeader( d_fall_pps::SetpointWithHeader & msg )
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
            ROS_ERROR("[PICKER CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
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
bool PickerControllerTab::getTypeAndIDParameters()
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
        ROS_ERROR("[PICKER CONTROLLER TAB GUI] Failed to get type");
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
        ROS_ERROR("[PICKER CONTROLLER TAB GUI] The 'type' parameter retrieved was not recognised.");
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
                ROS_ERROR("[PICKER CONTROLLER TAB GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[PICKER CONTROLLER TAB GUI] Is of type AGENT with ID = " << m_ID);
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
                ROS_ERROR("[PICKER CONTROLLER TAB GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[PICKER CONTROLLER TAB GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[PICKER CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
