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
//    The title displayed at the top of the GUI
//
//    ----------------------------------------------------------------------------------





#include "topbanner.h"
#include "ui_topbanner.h"

#ifdef CATKIN_MAKE
#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

#endif



TopBanner::TopBanner(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TopBanner)
{
    ui->setupUi(this);
    (":/images/rf_connected.png");
    QPixmap pixmap(":/images/emergency_stop.png");
    QIcon ButtonIcon(pixmap);
    ui->emergency_stop_button->setText("");
    ui->emergency_stop_button->setIcon(ButtonIcon);
    ui->emergency_stop_button->setIconSize(QSize(50,50));
    //ui->emergency_stop_button->setIconSize(pixmap.rect().size());


    m_object_name_for_emitting_pose_data = "";


#ifdef CATKIN_MAKE
    // Get the namespace of this node
    std::string base_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[TOP BANNER GUI] ros::this_node::getNamespace() =  " << base_namespace);


    // Get the type and ID of this parameter service
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the TYPE and ID are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[TOP BANNER GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }
#else
    // Default as a coordinator when compiling with QtCreator
    m_type = TYPE_COORDINATOR;
    m_ID = 1;
#endif



#ifdef CATKIN_MAKE
    // CREATE A NODE HANDLE TO THE BASE NAMESPACE
    //ros::NodeHandle base_nodeHandle(base_namespace);

    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    // > Publisher for the emergency stop button
    emergencyStopPublisher = dfall_root_nodeHandle.advertise<d_fall_pps::IntWithHeader>("emergencyStop", 1);

	// > For changes in the database that defines {agentID,cfID,flying zone} links
	databaseChangedSubscriber = dfall_root_nodeHandle.subscribe("CentralManagerService/DBChanged", 1, &TopBanner::databaseChangedCallback, this);;
	centralManagerDatabaseService = dfall_root_nodeHandle.serviceClient<d_fall_pps::CMQuery>("CentralManagerService/Query", false);
#endif


    // FURTHER INITILIASATIONS NEED TO OCCUR AFTER THE ROS RELATED
    // INITIALISATIONS ARE COMPLETE
    if (m_type == TYPE_AGENT)
    {
    	loadCrazyflieContext(m_ID,1000);
    }
    else if (m_type == TYPE_COORDINATOR)
    {
		// Set the label appropriate for a cooridnator
		QString qstr_title = "Flying Agent GUI: for COORDINATOR ID ";
		qstr_title.append( QString::number(m_ID) );
		ui->top_banner_label->setText(qstr_title);
	}
	else
	{
		// Set the label to inform the user of the error
		QString qstr_title = "Flying Agent GUI: for UNKNOWN NODE TYPE";
		ui->top_banner_label->setText(qstr_title);
    }

}

TopBanner::~TopBanner()
{
    delete ui;
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
void TopBanner::databaseChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STEAM("[TOP BANNER GUI] Database Changed Callback called for agentID = " << m_agentID);
    loadCrazyflieContext(m_ID,0);
}
#endif



void TopBanner::emitObjectNameForDisplayingPoseDataValueChanged()
{
    emit objectNameForDisplayingPoseDataValueChanged( m_object_name_for_emitting_pose_data );
#ifdef CATKIN_MAKE
    ROS_INFO_STREAM("[TOP BANNER GUI] Object name \"" << m_object_name_for_emitting_pose_data.toStdString() << "\" emitted for the controller tabs.");
#endif
}


// > For loading the "context" for this agent, i.e., the {agentID,cfID,flying zone} tuple
void TopBanner::loadCrazyflieContext(int ID_to_request_from_database , int emit_after_milliseconds)
{

    QString qstr_crazyflie_name = "";

#ifdef CATKIN_MAKE
	d_fall_pps::CMQuery contextCall;
	contextCall.request.studentID = ID_to_request_from_database;
	//ROS_INFO_STREAM("StudentID:" << m_agentID);

	centralManagerDatabaseService.waitForExistence(ros::Duration(-1));

	if(centralManagerDatabaseService.call(contextCall))
	{
		my_context = contextCall.response.crazyflieContext;
		ROS_INFO_STREAM("[TOP BANNER GUI] CrazyflieContext:\n" << my_context);

		qstr_crazyflie_name.append(QString::fromStdString(my_context.crazyflieName));

        m_object_name_for_emitting_pose_data = QString::fromStdString(my_context.crazyflieName);

        // Emit the crazyflie name
        // > This signal is connected to the "controller tabs" widget
        //   and is used for filtering with motion capture data to
        //   display in the tabs for each controller
        if (emit_after_milliseconds == 0)
        {
            emit objectNameForDisplayingPoseDataValueChanged( QString::fromStdString(my_context.crazyflieName) );
            ROS_INFO_STREAM("[TOP BANNER GUI] Object name \"" << my_context.crazyflieName << "\" emitted for the controller tabs.");
        }
        else
        {
            QTimer::singleShot(emit_after_milliseconds, this, SLOT( emitObjectNameForDisplayingPoseDataValueChanged() ) );
        }
	}
	else
	{
		ROS_ERROR_STREAM("[TOP BANNER GUI] Failed to load context for agentID = " << m_ID);

        m_object_name_for_emitting_pose_data = "";

        if (emit_after_milliseconds == 0)
        {
            QString qstr = "";
            emit objectNameForDisplayingPoseDataValueChanged( qstr );
            ROS_INFO_STREAM("[TOP BANNER GUI] emitted for the controller tabs that no object name is available.");
        }
        else
        {
            QTimer::singleShot(emit_after_milliseconds, this, SLOT( emitObjectNameForDisplayingPoseDataValueChanged() ) );
        }
	}
	// This updating of the radio only needs to be done by the actual agent's node
	//ros::NodeHandle nh("CrazyRadio");
	//nh.setParam("crazyFlieAddress", m_context.crazyflieAddress);
#else
	// Set the Crazyflie Name String to be a question mark
	qstr_crazyflie_name.append("?");
#endif

	// Construct and set the string for the checkbox label
	QString qstr_title = "Flying Agent GUI: for AGENT ID ";
	qstr_title.append( QString::number(m_ID) );
	qstr_title.append(", connected to ");
	qstr_title.append(qstr_crazyflie_name);
	ui->top_banner_label->setText(qstr_title);
}




//    ----------------------------------------------------------------------------------
//    BBBB   U   U  TTTTT  TTTTT   OOO   N   N   SSSS
//    B   B  U   U    T      T    O   O  NN  N  S
//    BBBB   U   U    T      T    O   O  N N N   SSS
//    B   B  U   U    T      T    O   O  N  NN      S
//    BBBB    UUU     T      T     OOO   N   N  SSSS
//    ----------------------------------------------------------------------------------



void TopBanner::on_emergency_stop_button_clicked()
{
#ifdef CATKIN_MAKE
    d_fall_pps::IntWithHeader msg;
    msg.shouldCheckForID = false;
    msg.data = CMD_CRAZYFLY_MOTORS_OFF;
    this->emergencyStopPublisher.publish(msg);
    ROS_INFO("[TOP BANNER GUI] EMERGENCY STOP button clicked");
#endif
}





//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void TopBanner::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
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

    // If there is only one agent to coordinate,
    // then load the context for that agent
    if (agentIDs.length() == 1)
    {
        loadCrazyflieContext(agentIDs[0],0);
    }
    else
    {
        // Set the label appropriate for a cooridnator
        QString qstr_title = "Flying Agent GUI: for COORDINATOR ID ";
        qstr_title.append( QString::number(m_ID) );
        ui->top_banner_label->setText(qstr_title);

        // Update the class variable for the name
        m_object_name_for_emitting_pose_data = "";
        // Emit the empty name as a signal
        QString qstr = "";
        emit objectNameForDisplayingPoseDataValueChanged( qstr );
    }
}






//    ----------------------------------------------------------------------------------
//    III  DDDD       &&&      TTTTT  Y   Y  PPPP   EEEEE
//     I   D   D     &           T     Y Y   P   P  E
//     I   D   D      &          T      Y    PPPP   EEE
//     I   D   D     & & &       T      Y    P      E
//    III  DDDD       &&&        T      Y    P      EEEEE
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
bool TopBanner::getTypeAndIDParameters()
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
        ROS_ERROR("[TOP BANNER GUI] Failed to get type");
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
        ROS_ERROR("[TOP BANNER GUI] The 'type' parameter retrieved was not recognised.");
    }


    // Construct the string to the namespace of this Paramater Service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            // Get the value of the "agentID" parameter into the class variable "m_ID"
            if(!nodeHandle.getParam("agentID", m_ID))
            {
                // Throw an error if the agent ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[TOP BANNER GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[TOP BANNER GUI] Is of type AGENT with ID = " << m_ID);
            }
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            // Get the value of the "coordID" parameter into the class variable "m_ID"
            if(!nodeHandle.getParam("coordID", m_ID))
            {
                // Throw an error if the coord ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[TOP BANNER GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[TOP BANNER GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[TOP BANNER GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
