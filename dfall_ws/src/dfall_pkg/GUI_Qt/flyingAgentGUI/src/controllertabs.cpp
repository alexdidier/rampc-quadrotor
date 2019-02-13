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
//    The tab wdiget that contains and manages the individual tabs used
//    to interface with each separate controller.
//
//    ----------------------------------------------------------------------------------





#include "controllertabs.h"
#include "ui_controllertabs.h"

ControllerTabs::ControllerTabs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerTabs)
{
    ui->setupUi(this);


    // Specify the color for normal and highlighted tabs
    m_tab_text_colour_normal = Qt::black;
    m_tab_text_colour_highlight = QColor(0,200,0);


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

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->picker_controller_tab_widget , &PickerControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->tuning_controller_tab_widget , &TuningControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->template_controller_tab_widget , &TemplateControllerTab::setMeasuredPose
        );



    // CONNECT THE "MEASUREMENTS UNAVAILABLE" SIGNAL TO
    // EACH OF THE TABS
    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->default_controller_tab_widget , &DefaultControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->student_controller_tab_widget , &StudentControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->picker_controller_tab_widget , &PickerControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->tuning_controller_tab_widget , &TuningControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->template_controller_tab_widget , &TemplateControllerTab::poseDataUnavailableSlot
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

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->student_controller_tab_widget , &StudentControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->picker_controller_tab_widget , &PickerControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->tuning_controller_tab_widget , &TuningControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->template_controller_tab_widget , &TemplateControllerTab::setAgentIDsToCoordinate
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

    // CREATE THE SUBSCRIBER TO THE CONTROLLER THAT IS CURRENTLY OPERATING
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        controllerUsedSubscriber = nodeHandle_for_this_gui.subscribe("FlyingAgentClient/controllerUsed", 1, &ControllerTabs::controllerUsedChangedCallback, this);
    }



    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    // > Publisher for the emergency stop button
    //emergencyStopPublisher = dfall_root_nodeHandle.advertise<dfall_pkg::IntWithHeader>("emergencyStop", 1);

    // > For changes in the database that defines {agentID,cfID,flying zone} links
    //databaseChangedSubscriber = dfall_root_nodeHandle.subscribe("CentralManagerService/DBChanged", 1, &TopBanner::databaseChangedCallback, this);;
    centralManagerDatabaseService = dfall_root_nodeHandle.serviceClient<dfall_pkg::CMQuery>("CentralManagerService/Query", false);


#endif

}

ControllerTabs::~ControllerTabs()
{
    delete ui;
}



void ControllerTabs::showHideController_toggle(QString qstr_label, QWidget * tab_widget_to_toggle)
{
    // Get the current index of the tab
    // > Note the this returns -1 if the tab is not found
    int current_index_of_tab = ui->controller_tabs_widget->indexOf(tab_widget_to_toggle);

    // Switch depending on whether the tab was found
    if (current_index_of_tab < 0)
    {
        // Insert the tab
        ui->controller_tabs_widget->addTab(tab_widget_to_toggle,qstr_label);
    }
    else
    {
        // Remove the tab
        ui->controller_tabs_widget->removeTab(current_index_of_tab);
    }
}



void ControllerTabs::showHideController_default_changed()
{
    showHideController_toggle("Default",ui->default_tab);
}

void ControllerTabs::showHideController_student_changed()
{
    showHideController_toggle("Student",ui->student_tab);
}

void ControllerTabs::showHideController_picker_changed()
{
    showHideController_toggle("Picker",ui->picker_tab);
}

void ControllerTabs::showHideController_tuning_changed()
{
    showHideController_toggle("Tuning",ui->tuning_tab);
}

void ControllerTabs::showHideController_template_changed()
{
    showHideController_toggle("Template",ui->template_tab);
}






void ControllerTabs::setObjectNameForDisplayingPoseData( QString object_name )
{
    m_should_search_pose_data_for_object_name_mutex.lock();
    if (object_name.isEmpty())
    {
        // Set the class variable accordingly
        m_object_name_for_emitting_pose_data = "";
        // Update the flag accordingly
        m_should_search_pose_data_for_object_name = false;
        // Emit a signal to let the tabs know
        emit poseDataUnavailableSignal();
        // Inform the user
        #ifdef CATKIN_MAKE
            ROS_INFO("[CONTROLLER TABS GUI] No longer emitting pose data for any object.");
        #endif
    }
    else
    {
        // Set the class variable accordingly
        m_object_name_for_emitting_pose_data = object_name.toStdString();
        // Update the flag accordingly
        m_should_search_pose_data_for_object_name = true;
        // Inform the user
        #ifdef CATKIN_MAKE
            ROS_INFO_STREAM("[CONTROLLER TABS GUI] now emitting data for object named: " << m_object_name_for_emitting_pose_data << ", with ID = " << m_ID );
        #endif


        #ifdef CATKIN_MAKE
        // Get also the context
        dfall_pkg::CMQuery contextCall;
        contextCall.request.studentID = m_ID;

        centralManagerDatabaseService.waitForExistence(ros::Duration(-1));

        if(centralManagerDatabaseService.call(contextCall))
        {
            m_context = contextCall.response.crazyflieContext;
            m_area = m_context.localArea;
            ROS_INFO_STREAM("[CONTROLLER TABS GUI] AreaBounds:\n" << m_area);

            //qstr_crazyflie_name.append(QString::fromStdString(m_context.crazyflieName));

            //m_object_name_for_emitting_pose_data = QString::fromStdString(my_context.crazyflieName);

        }
        else
        {
            ROS_ERROR_STREAM("[CONTROLLER TABS GUI] Failed to load context for agentID = " << m_ID);
        }
        #endif

    }
    m_should_search_pose_data_for_object_name_mutex.unlock();
}


#ifdef CATKIN_MAKE
// > For the controller currently operating, received on
//   "controllerUsedSubscriber"
void ControllerTabs::poseDataReceivedCallback(const dfall_pkg::ViconData& viconData)
{
    m_should_search_pose_data_for_object_name_mutex.lock();
    if (m_should_search_pose_data_for_object_name)
    {
        for(std::vector<dfall_pkg::CrazyflieData>::const_iterator it = viconData.crazyflies.begin(); it != viconData.crazyflies.end(); ++it)
        {
            dfall_pkg::CrazyflieData pose_in_global_frame = *it;

            if(pose_in_global_frame.crazyflieName == m_object_name_for_emitting_pose_data)
            {

                // Convert it into the local frame
                float originX = (m_area.xmin + m_area.xmax) / 2.0;
                float originY = (m_area.ymin + m_area.ymax) / 2.0;
                
                pose_in_global_frame.x -= originX;
                pose_in_global_frame.y -= originY;

                // change Z origin to zero, i.e., to the table height, zero of global coordinates, instead of middle of the box
                //float originZ = 0.0;
                // float originZ = (area.zmin + area.zmax) / 2.0;
                //pose_in_global_frame.z -= originZ;

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
    m_should_search_pose_data_for_object_name_mutex.unlock();
}
#endif






//    ----------------------------------------------------------------------------------
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//
//    EEEEE  N   N    A    BBBB   L      EEEEE  DDDD
//    E      NN  N   A A   B   B  L      E      D   D
//    EEE    N N N  A   A  BBBB   L      EEE    D   D
//    E      N  NN  AAAAA  B   B  L      E      D   D
//    EEEEE  N   N  A   A  BBBB   LLLLL  EEEEE  DDDD
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// > For the controller currently operating, received on "controllerUsedSubscriber"
void ControllerTabs::controllerUsedChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Controller Used Changed Callback called for agentID = " << m_agentID);
    setControllerEnabled(msg.data);
}
#endif


void ControllerTabs::setControllerEnabled(int new_controller)
{
    // First set everything back to normal colouring
    setAllTabLabelsToNormalColouring();

    // Now switch to highlight the tab corresponding to
    // the enable controller
    switch(new_controller)
    {
       case DEFAULT_CONTROLLER:
       {
           setTextColourOfTabLabel( m_tab_text_colour_highlight , ui->default_tab );
           break;
       }
        case DEMO_CONTROLLER:
        {
            //ui->controller_enabled_label->setText("Demo");
            break;
        }
        case STUDENT_CONTROLLER:
        {
            setTextColourOfTabLabel( m_tab_text_colour_highlight , ui->student_tab );
            break;
        }
        case MPC_CONTROLLER:
        {
            //ui->controller_enabled_label->setText("MPC");
            break;
        }
        case REMOTE_CONTROLLER:
        {
            //ui->controller_enabled_label->setText("Remote");
            break;
        }
        case TUNING_CONTROLLER:
        {
            setTextColourOfTabLabel( m_tab_text_colour_highlight , ui->tuning_tab );
            break;
        }
        case PICKER_CONTROLLER:
        {
            setTextColourOfTabLabel( m_tab_text_colour_highlight , ui->picker_tab );
            break;
        }
        case TEMPLATE_CONTROLLER:
        {
            setTextColourOfTabLabel( m_tab_text_colour_highlight , ui->template_tab );
            break;
        }
        default:
        {
            //ui->controller_enabled_label->setText("Unknown");
            break;
        }
    }
}


void ControllerTabs::setAllTabLabelsToNormalColouring()
{
    setTextColourOfTabLabel( m_tab_text_colour_normal , ui->default_tab );
    setTextColourOfTabLabel( m_tab_text_colour_normal , ui->student_tab );
    setTextColourOfTabLabel( m_tab_text_colour_normal , ui->picker_tab );
    setTextColourOfTabLabel( m_tab_text_colour_normal , ui->tuning_tab );
    setTextColourOfTabLabel( m_tab_text_colour_normal , ui->template_tab );
}

void ControllerTabs::setTextColourOfTabLabel(QColor color , QWidget * tab_widget)
{
    // Get the current index of the tab
    int current_index_of_tab = ui->controller_tabs_widget->indexOf(tab_widget);
    // Onlz apply the colour is the tab is found
    if (current_index_of_tab >= 0)
    {
        ui->controller_tabs_widget->tabBar()->setTabTextColor(current_index_of_tab, color);
    }
}



//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void ControllerTabs::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
{
    // Pass on the signal to the tabs
    emit agentIDsToCoordinateChanged( agentIDs , shouldCoordinateAll );


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

        
        // > Request the current instant controller
        ros::ServiceClient getInstantControllerService = agent_base_nodeHandle.serviceClient<dfall_pkg::IntIntService>("FlyingAgentClient/getInstantController", false);
        dfall_pkg::IntIntService getInstantControllerCall;
        getInstantControllerCall.request.data = 0;
        getInstantControllerService.waitForExistence(ros::Duration(2.0));
        if(getInstantControllerService.call(getInstantControllerCall))
        {
            setControllerEnabled(getInstantControllerCall.response.data);
        }
        else
        {
            //setControllerEnabled(CONTROLLER_UNKNOWN);
        }

        // SUBSCRIBERS
        // > For receiving message that the instant controller was changed
        controllerUsedSubscriber = agent_base_nodeHandle.subscribe("FlyingAgentClient/controllerUsed", 1, &ControllerTabs::controllerUsedChangedCallback, this);
    }
    else
    {
        // Unsubscribe
        controllerUsedSubscriber.shutdown();

        // Set all tabs to be normal colours
        setAllTabLabelsToNormalColouring();

    }
#endif

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
