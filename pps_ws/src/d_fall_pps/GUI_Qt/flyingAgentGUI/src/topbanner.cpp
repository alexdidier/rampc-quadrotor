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





TopBanner::TopBanner(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TopBanner)
{
    ui->setupUi(this);


#ifdef CATKIN_MAKE
    // Get the namespace of this node
    std::string base_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[CONNECT START STOP GUI BAR] ros::this_node::getNamespace() =  " << base_namespace);


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



#ifdef CATKIN_MAKE
    // CREATE A NODE HANDLE TO THE BASE NAMESPACE
    //ros::NodeHandle base_nodeHandle(base_namespace);

    // CREATE A NODE HANDLE TO THE ROOT OF THE D-FaLL SYSTEM
    ros::NodeHandle dfall_root_nodeHandle("/dfall");

    // SUBSCRIBER AND SERVICE:
    if (m_type == TYPE_AGENT)
    {
    	// > For changes in the database that defines {agentID,cfID,flying zone} links
    	databaseChangedSubscriber = dfall_root_nodeHandle.subscribe("CentralManagerService/DBChanged", 1, &TopBanner::databaseChangedCallback, this);;
    	centralManagerDatabaseService = dfall_root_nodeHandle.serviceClient<d_fall_pps::CMQuery>("CentralManagerService/Query", false);
    }
#endif


    // FURTHER INITILIASATIONS NEED TO OCCUR AFTER THE ROS RELATED
    // INITIALISATIONS ARE COMPLETE
    if (m_type == TYPE_AGENT)
    {
    	loadCrazyflieContext();
    }
    else if (m_type == TYPE_COORDINATOR)
    {
		// Set the label appropriate for a cooridnator
		QString qstr_title = "Flying Device GUI: for COORDINATOR ID ";
		qstr_title.append( QString::number(m_ID) );
		ui->top_banner_label->setText(qstr_title);
	}
	else
	{
		// Set the label to inform the user of the error
		QString qstr_title = "Flying Device GUI: for UNKNOWN NODE TYPE";
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
    //ROS_INFO_STEAM("[COORDINATOR ROW GUI] Database Changed Callback called for agentID = " << m_agentID);
    loadCrazyflieContext();
}
#endif

// > For loading the "context" for this agent, i.e., the {agentID,cfID,flying zone} tuple
void TopBanner::loadCrazyflieContext()
{

    QString qstr_crazyflie_name = "";

	if (m_type == TYPE_AGENT)
	{

#ifdef CATKIN_MAKE
		d_fall_pps::CMQuery contextCall;
		contextCall.request.studentID = m_ID;
		//ROS_INFO_STREAM("StudentID:" << m_agentID);

		centralManagerDatabaseService.waitForExistence(ros::Duration(-1));

		if(centralManagerDatabaseService.call(contextCall))
		{
			my_context = contextCall.response.crazyflieContext;
			ROS_INFO_STREAM("[TOP BANNER GUI] CrazyflieContext:\n" << my_context);

			qstr_crazyflie_name.append(QString::fromStdString(my_context.crazyflieName));
		}
		else
		{
			ROS_ERROR_STREAM("[TOP BANNER GUI] Failed to load context for agentID = " << m_ID);
		}
		// This updating of the radio only needs to be done by the actual agent's node
		//ros::NodeHandle nh("CrazyRadio");
		//nh.setParam("crazyFlieAddress", m_context.crazyflieAddress);
#else
		// Set the Crazyflie Name String to be a question mark
		qstr_crazyflie_name.append("?");
#endif

		// Construct and set the string for the checkbox label
		QString qstr_title = "Flying Device GUI: for AGENT ID ";
		qstr_title.append( QString::number(m_ID) );
		qstr_title.append(", connected to ");
		qstr_title.append(qstr_crazyflie_name);
		ui->top_banner_label->setText(qstr_title);

	}
	else
	{
#ifdef CATKIN_MAKE
		ROS_ERROR("[TOP BANNER GUI] loadCrazyflieContext called by a node that is NOT of type AGENT.");
#endif
		qstr_crazyflie_name.append("??");
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
            // Get the value of the "agentID" parameter into the class variable "m_ID"
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
            // Get the value of the "coordID" parameter into the class variable "m_ID"
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