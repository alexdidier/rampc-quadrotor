#include "controllertabs.h"
#include "ui_controllertabs.h"

ControllerTabs::ControllerTabs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerTabs)
{
    ui->setupUi(this);


    // Initialise the object name as blank
    m_object_name_for_emitting_pose_data = "";


    // CONNECT THE "MEASURED POST" SIGNAL TO EACH OF
    // THE TABS
    // i.e., connect the "measured pose value changed"
    // signal to the "set measured pose" slots
    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->default_controller_tab_widget , &DefaultControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->student_controller_tab_widget , &StudentControllerTab::setMeasuredPose
        );


    // CONNECT THE "MEASUREMENTS UNAVAILABLE" SIGNAL TO
    // EACH OF THE TABS
    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->default_controller_tab_widget , &DefaultControllerTab::poseDataUnavailableSlot
        );


    // CONNECT TO THE COORDINATOR SIGNAL TO BE ALWAYS UPDATED
    // WITH THE LIST OF AGENT IDs TO COORDINATE
    // This is passed from this "controller tabs widget" to
    // each of the controller tabs. The signal is simply
    // "passed through"
    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->default_controller_tab_widget , &DefaultControllerTab::setAgentIDsToCoordinate
        );

    



#ifdef CATKIN_MAKE

    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[CONTROLLER TABS GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[CONTROLLER TABS GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE A NODE HANDLE TO THE D-FaLL ROOT
    ros::NodeHandle nodeHandle_dfall_root("/dfall");

    // CREATE THE SUBSCRIBER TO THE MOTION CAPTURE DATA
    m_poseDataSubscriber = nodeHandle_dfall_root.subscribe("ViconDataPublisher/ViconData", 100, &ControllerTabs::poseDataReceivedCallback, this);

#endif

}

ControllerTabs::~ControllerTabs()
{
    delete ui;
}




void ControllerTabs::setObjectNameForDisplayingPoseData( QString object_name )
{
    if (object_name.isEmpty())
    {
        // Set the class variable accordingly
        m_object_name_for_emitting_pose_data = "";
        // Update the flag accordingly
        m_should_search_pose_data_for_object_name = false;
        // Emit a signal to let the tabs know
        emit poseDataUnavailableSignal();
        // Inform the user
        ROS_INFO("[CONTROLLER TABS GUI] No longer emitting pose data for any object.");
    }
    else
    {
        // Set the class variable accordingly
        m_object_name_for_emitting_pose_data = object_name.toStdString();
        // Update the flag accordingly
        m_should_search_pose_data_for_object_name = true;
        // Inform the user
        #ifdef CATKIN_MAKE
            ROS_INFO_STREAM("[CONTROLLER TABS GUI] now emitting data for object named: " << m_object_name_for_emitting_pose_data );
        #endif
    }
}


#ifdef CATKIN_MAKE
// > For the controller currently operating, received on
//   "controllerUsedSubscriber"
void ControllerTabs::poseDataReceivedCallback(const d_fall_pps::ViconData& viconData)
{
    if (m_should_search_pose_data_for_object_name)
    {
        for(std::vector<d_fall_pps::CrazyflieData>::const_iterator it = viconData.crazyflies.begin(); it != viconData.crazyflies.end(); ++it)
        {
            d_fall_pps::CrazyflieData pose_in_global_frame = *it;

            if(pose_in_global_frame.crazyflieName == m_object_name_for_emitting_pose_data)
            {
                emit measuredPoseValueChanged(
                        pose_in_global_frame.x,
                        pose_in_global_frame.y,
                        pose_in_global_frame.z,
                        pose_in_global_frame.roll,
                        pose_in_global_frame.pitch,
                        pose_in_global_frame.yaw,
                        pose_in_global_frame.occluded
                    );
            }
        }
    }
}
#endif





//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void ControllerTabs::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
{
    // Simply pass on the signal to the tabs
    emit agentIDsToCoordinateChanged( agentIDs , shouldCoordinateAll );
}





//    ----------------------------------------------------------------------------------
//    III  DDDD       &&&      TTTTT  Y   Y  PPPP   EEEEE
//     I   D   D     &           T     Y Y   P   P  E
//     I   D   D      &          T      Y    PPPP   EEE
//     I   D   D     & & &       T      Y    P      E
//    III  DDDD       &&&        T      Y    P      EEEEE
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
bool ControllerTabs::getTypeAndIDParameters()
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