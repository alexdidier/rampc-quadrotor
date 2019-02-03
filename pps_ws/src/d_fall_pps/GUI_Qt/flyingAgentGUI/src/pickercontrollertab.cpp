#include "pickercontrollertab.h"
#include "ui_pickercontrollertab.h"

PickerControllerTab::PickerControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PickerControllerTab)
{
    ui->setupUi(this);


    // HIDE ALL THE "GREEN FRAMES"
    ui->frame_goto_start_active->setVisible(false);
    ui->frame_attach_active->setVisible(false);
    ui->frame_lift_up_active->setVisible(false);
    ui->frame_goto_end_active->setVisible(false);
    ui->frame_put_down_active->setVisible(false);
    ui->frame_squat_active->setVisible(false);
    ui->frame_jump_active->setVisible(false);
    ui->frame_standby_active->setVisible(false);

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
    if ( state_to_match.contains(current_picker_state) )
    {
        publish_request_setpoint_change_for_state(current_picker_state);
    }
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
    case PICKER_STATE_LIFT:
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
    publish_request_setpoint_change_for_state(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
        states_for_which_this_change_applies.append(PICKER_STATE_LIFT);
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
