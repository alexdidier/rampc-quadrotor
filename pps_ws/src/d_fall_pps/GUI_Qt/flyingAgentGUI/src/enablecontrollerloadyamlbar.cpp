#include "enablecontrollerloadyamlbar.h"
#include "ui_enablecontrollerloadyamlbar.h"

EnableControllerLoadYamlBar::EnableControllerLoadYamlBar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EnableControllerLoadYamlBar)
{
    ui->setupUi(this);


#ifdef CATKIN_MAKE
    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[ENABLE CONTROLLER LOAD YAML GUI BAR] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE THE COMMAND PUBLISHER
    commandPublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::IntWithHeader>("PPSClient/Command", 1);

    // CREATE THE REQUEST LOAD YAML FILE PUBLISHER
    // Get the node handle to this parameter service
    m_requestLoadYamlFilenamePublisher = nodeHandle_for_this_gui.advertise<d_fall_pps::StringWithHeader>("ParameterService/requestLoadYamlFilename", 1);
#endif

}

EnableControllerLoadYamlBar::~EnableControllerLoadYamlBar()
{
    delete ui;
}




void EnableControllerLoadYamlBar::showHideController_default_changed()
{
    ui->enable_default_button   ->setHidden( !(ui->enable_default_button->isHidden()) );
    ui->load_yaml_default_button->setHidden( !(ui->load_yaml_default_button->isHidden()) );
}

void EnableControllerLoadYamlBar::showHideController_student_changed()
{
    ui->enable_student_button   ->setHidden( !(ui->enable_student_button->isHidden()) );
    ui->load_yaml_student_button->setHidden( !(ui->load_yaml_student_button->isHidden()) );
}

void EnableControllerLoadYamlBar::showHideController_picker_changed()
{
    ui->enable_demo_button   ->setHidden(    !(ui->enable_demo_button->isHidden()) );
    ui->load_yaml_demo_button->setHidden( !(ui->load_yaml_demo_button->isHidden()) );
}

void EnableControllerLoadYamlBar::showHideController_safe_changed()
{
    ui->enable_safe_button   ->setHidden( !(ui->enable_safe_button->isHidden()) );
    ui->load_yaml_safe_button->setHidden( !(ui->load_yaml_safe_button->isHidden()) );
}






// ENABLE CONTROLLER BUTTONS ON-CLICK CALLBACK

void EnableControllerLoadYamlBar::on_enable_safe_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_USE_SAFE_CONTROLLER;
    this->commandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Enable Safe Controller");
#endif
}

void EnableControllerLoadYamlBar::on_enable_demo_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_USE_DEMO_CONTROLLER;
    this->commandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Enable Demo Controller");
#endif
}

void EnableControllerLoadYamlBar::on_enable_student_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    fillIntMessageHeader(msg);
    msg.data = CMD_USE_STUDENT_CONTROLLER;
    this->commandPublisher.publish(msg);
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Enable Student Controller");
#endif
}

void EnableControllerLoadYamlBar::on_enable_default_button_clicked()
{
#ifdef CATKIN_MAKE
    //d_fall_pps::IntWithHeader msg;
    //fillIntMessageHeader(msg);
    //msg.data = CMD_USE_STUDENT_CONTROLLER;
    //this->commandPublisher.publish(msg);
    //ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Enable Student Controller");
#endif
}


// LOAD YAML BUTTONS ON-CLICK CALLBACK

void EnableControllerLoadYamlBar::on_load_yaml_safe_button_clicked()
{
#ifdef CATKIN_MAKE

#endif
}

void EnableControllerLoadYamlBar::on_load_yaml_demo_button_clicked()
{
#ifdef CATKIN_MAKE
    // Create a local variable for the message
    d_fall_pps::StringWithHeader yaml_filename_msg;
    // Set for whom this applies to
    fillStringMessageHeader(yaml_filename_msg);
    // Specify the data
    yaml_filename_msg.data = "DemoController";
    // Send the message
    m_requestLoadYamlFilenamePublisher.publish(yaml_filename_msg);
    // Inform the user that the menu item was selected
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Load Demo Controller YAML was clicked.");
#endif
}

void EnableControllerLoadYamlBar::on_load_yaml_student_button_clicked()
{
#ifdef CATKIN_MAKE
    // Create a local variable for the message
    d_fall_pps::StringWithHeader yaml_filename_msg;
    // Set for whom this applies to
    fillStringMessageHeader(yaml_filename_msg);
    // Specify the data
    yaml_filename_msg.data = "StudentController";
    // Send the message
    m_requestLoadYamlFilenamePublisher.publish(yaml_filename_msg);
    // Inform the user that the menu item was selected
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Load Student Controller YAML was clicked.");
#endif
}

void EnableControllerLoadYamlBar::on_load_yaml_default_button_clicked()
{
    #ifdef CATKIN_MAKE
    // Create a local variable for the message
    d_fall_pps::StringWithHeader yaml_filename_msg;
    // Set for whom this applies to
    fillStringMessageHeader(yaml_filename_msg);
    // Specify the data
    yaml_filename_msg.data = "DefaultController";
    // Send the message
    m_requestLoadYamlFilenamePublisher.publish(yaml_filename_msg);
    // Inform the user that the menu item was selected
    ROS_INFO("[ENABLE CONTROLLER LOAD YAML GUI BAR] Load Default Controller YAML was clicked.");
#endif
}





//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void EnableControllerLoadYamlBar::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
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
void EnableControllerLoadYamlBar::fillIntMessageHeader( d_fall_pps::IntWithHeader & msg )
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
            msg.shouldCheckForID = true;
            ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] The 'm_type' variable was not recognised.");
            break;
        }
    } 
}
#endif





#ifdef CATKIN_MAKE
// Fill the head for a message
void EnableControllerLoadYamlBar::fillStringMessageHeader( d_fall_pps::StringWithHeader & msg )
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
bool EnableControllerLoadYamlBar::getTypeAndIDParameters()
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
        ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Failed to get type");
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
        ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] The 'type' parameter retrieved was not recognised.");
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
                ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[ENABLE CONTROLLER LOAD YAML GUI BAR] Is of type AGENT with ID = " << m_ID);
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
                ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[ENABLE CONTROLLER LOAD YAML GUI BAR] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[ENABLE CONTROLLER LOAD YAML GUI BAR] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
