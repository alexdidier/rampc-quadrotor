#include "controllertabs.h"
#include "ui_controllertabs.h"

ControllerTabs::ControllerTabs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerTabs)
{
    ui->setupUi(this);



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



#ifdef CATKIN_MAKE
// > For the controller currently operating, received on
//   "controllerUsedSubscriber"
void ControllerTabs::poseDataReceivedCallback(const d_fall_pps::ViconData& viconData)
{
    for(std::vector<d_fall_pps::CrazyflieData>::const_iterator it = viconData.crazyflies.begin(); it != viconData.crazyflies.end(); ++it)
    {
        d_fall_pps::CrazyflieData pose_in_global_frame = *it;

        if(pose_in_global_frame.crazyflieName == "CF05")
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


    // OLD STYLE    
    // // Initialise a Qvector to sending around
    // QVector<float> poseDataForSignal;
    // // Fill in the data
    // poseDataForSignal.push_back(msg.x);
    // poseDataForSignal.push_back(msg.y);
    // poseDataForSignal.push_back(msg.z);
    // poseDataForSignal.push_back(msg.roll);
    // poseDataForSignal.push_back(msg.pitch);
    // poseDataForSignal.push_back(msg.yaw);
    // // Emit the signal
    // emit measuredPoseValueChanged(poseDataForSignal);

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