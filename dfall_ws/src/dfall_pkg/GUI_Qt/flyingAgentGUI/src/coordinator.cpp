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
//    The coordinator part of the Flying Agent GUI that allows multiple
//    flying agents to be controller from one place, and keeps track
//    of which subset of flying agents to control.
//
//    ----------------------------------------------------------------------------------





#include "coordinator.h"
#include "ui_coordinator.h"

Coordinator::Coordinator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Coordinator)
{
    ui->setupUi(this);

    ui->verticalLayout_for_coordinatedAgentsScrollArea->setAlignment(Qt::AlignTop);
}

Coordinator::~Coordinator()
{
    delete ui;
}

void Coordinator::on_refresh_button_clicked()
{
    // DELETE ALL EXISTING ROWS
    remove_all_entries_from_vector_of_coordinatorRows();

    // Get the state of the "coordinate all" is check box
    bool shouldCoordinateAll = ui->coordinate_all_checkBox->isChecked();


#ifdef CATKIN_MAKE
    // USE A REGULAR EXPRESSION TO IDENTIFY NODES THAT EXIST
    // > The regular expression we use is: \/agent(\d\d\d)\/FlyingAgentClient
    //   with the different that the escaped character is \\ instead of \ only

    // GET A "V_string" OF ALL THE NODES CURRENTLY IN EXISTENCE
    ros::V_string v_str;
    ros::master::getNodes(v_str);

    // ITERATE THROUGH THIS "V_string" OF ALL NODE IN EXISTENCE
    for(int i = 0; i < v_str.size(); i++)
    {
        // TEST THE NAME OF THIS NODE AGAINST THE REGULAR EXPRESSION
        std::string s = v_str[i];
        std::smatch m;
        std::regex e ("\\/agent(\\d\\d\\d)\\/FlyingAgentClient");

        if(std::regex_search(s, m, e))
        {
            // GET THE MATCHED PART OF THE NODE NAME INTO A LOCAL VARIABLE
            // > Note: we use m[1] because we are interested ONLY in the first match
            // > Thus "found_string" should the the agentID, as a string with zero padding
            std::string found_string = m[1].str();


            // LET THE USER KNOW THAT WE FOUND A MATCH
            ROS_INFO_STREAM("[Coordinator Row GUI] Found an agent with ID = " << found_string.c_str() );
            
            // CONVERT THE STRING TO AN INTEGER
            int this_agentID = stoi(found_string);

            // ADD A ROW TO THE COORDINATOR FOR THE AGENT WITH THIS ID FOUND
            CoordinatorRow *temp_coordinatorRow = new CoordinatorRow(this,this_agentID);

            // Check the box if "coordinate all" is checked
            temp_coordinatorRow->setShouldCoordinate(shouldCoordinateAll);

            // Connect the "should coordinate value changed" signal to
            // the respective slot
            QObject::connect(
                    temp_coordinatorRow , &CoordinatorRow::shouldCoordinateThisAgentValueChanged ,
                    this , &Coordinator::setShouldCoordinateThisAgent
                    );

            // Add to the vector of coordinator rows
            vector_of_coordinatorRows.append(temp_coordinatorRow);
            vector_of_shouldCoordinate_perRow.append(shouldCoordinateAll);
            vector_of_agentID_perRow.append(this_agentID);

            ui->coordinated_agents_scrollAreaWidgetContents->layout()->addWidget(temp_coordinatorRow);
        }
    }
#else


    for ( int i_agent = 1 ; i_agent < 9 ; i_agent++ )
    {
        //ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);

        CoordinatorRow *temp_coordinatorRow = new CoordinatorRow(this,i_agent);

        // Check the box if "coordinate all" is checked
        temp_coordinatorRow->setShouldCoordinate(shouldCoordinateAll);

        // Connect the "should coordinate value changed" signal to
        // the respective slot
        QObject::connect(
                temp_coordinatorRow , &CoordinatorRow::shouldCoordinateThisAgentValueChanged ,
                this , &Coordinator::setShouldCoordinateThisAgent
                );

        // Add to the vector of coordinator rows
        vector_of_coordinatorRows.append(temp_coordinatorRow);
        vector_of_shouldCoordinate_perRow.append(shouldCoordinateAll);
        vector_of_agentID_perRow.append(i_agent);

        ui->coordinated_agents_scrollAreaWidgetContents->layout()->addWidget(temp_coordinatorRow);
    }
#endif

    // Send out a signal with the current IDs to coordinate
    emit_signal_with_agentIDs_toCoordinate();
    
    // Call the function that applies this level
    // of detail to all the entries
    apply_level_of_detail_to_all_entries(level_of_detail_to_display);
}


void Coordinator::on_toggle_details_button_clicked()
{
    // Toggle the level of detail to display
    switch (level_of_detail_to_display)
    {
    case 0:
    {
        level_of_detail_to_display = 1;
        break;
    }
    case 1:
    {
        level_of_detail_to_display = 0;
        break;
    }
    default:
    {
        level_of_detail_to_display = 0;
        break;
    }
    }
    // Call the function that applies this level
    // of detail to all the entries
    apply_level_of_detail_to_all_entries(level_of_detail_to_display);
}

void Coordinator::on_delete_button_clicked()
{
    // Call the function that performs the task requested
    remove_all_entries_from_vector_of_coordinatorRows();

    // Send out a signal with the current IDs to coordinate
    emit_signal_with_agentIDs_toCoordinate();
}

void Coordinator::remove_all_entries_from_vector_of_coordinatorRows()
{
    // Iterate through and delete all entries
    foreach ( CoordinatorRow* temp_coordinatorRow, vector_of_coordinatorRows) {
        delete( temp_coordinatorRow );
    }
    // Clear the vector
    vector_of_coordinatorRows.clear();
    vector_of_shouldCoordinate_perRow.clear();
    vector_of_agentID_perRow.clear();
}

void Coordinator::apply_level_of_detail_to_all_entries(int level)
{
    // Apply this to all the rows
    for ( int irow = 0 ; irow < vector_of_coordinatorRows.length() ; irow++ )
    {
        vector_of_coordinatorRows[irow]->setLevelOfDetailToDisplay( level );
    }
}

void Coordinator::on_coordinate_all_checkBox_clicked()
{
    // Get the state of the "coordinate all" is check box
    bool shouldCoordinateAll = ui->coordinate_all_checkBox->isChecked();

    // Apply this to all the rows
    for ( int irow = 0 ; irow < vector_of_coordinatorRows.length() ; irow++ )
    {
        vector_of_coordinatorRows[irow]->setShouldCoordinate( shouldCoordinateAll );
        vector_of_shouldCoordinate_perRow[irow] = shouldCoordinateAll;
    }

    // Send out a signal with the current IDs to coordinate
    emit_signal_with_agentIDs_toCoordinate();
}


void Coordinator::setShouldCoordinateThisAgent(int agentID , bool shouldCoordinate)
{
    // Find the row with the matching ID, and update the "shouldCoordinate" vector
    for ( int irow = 0 ; irow < vector_of_agentID_perRow.length() ; irow++ )
    {
        if (vector_of_agentID_perRow[irow] == agentID)
        {
            vector_of_shouldCoordinate_perRow[irow] = shouldCoordinate;
            break;
        }
    }

    // Update the "Coordinate All Check Box" as appropriate
    bool shouldCoordinateAll = true;
    if (!shouldCoordinate)
    {
        shouldCoordinateAll = false;
    }
    else
    {
        for ( int irow = 0 ; irow < vector_of_shouldCoordinate_perRow.length() ; irow++ )
        {
            if (!(vector_of_shouldCoordinate_perRow[irow]))
            {
                shouldCoordinateAll = false;
                break;
            }
        }
    }
    ui->coordinate_all_checkBox->setChecked(shouldCoordinateAll);

    // Send out a signal with the current IDs to coordinate
    emit_signal_with_agentIDs_toCoordinate();
}

void Coordinator::emit_signal_with_agentIDs_toCoordinate()
{
    // Initilise a boolean for whether to coordinate all
    bool shouldCoordinateAll = true;
    // Send out a signal with the current IDs to coordinate
    QVector<int> agentIDsToCoordinate;
    for ( int irow = 0 ; irow < vector_of_shouldCoordinate_perRow.length() ; irow++ )
    {
        if (vector_of_shouldCoordinate_perRow[irow])
        {
            agentIDsToCoordinate.append(vector_of_agentID_perRow[irow]);
        }
        else
        {
            shouldCoordinateAll = false;
        }
    }
    emit agentIDsToCoordinateChanged( agentIDsToCoordinate , shouldCoordinateAll );


#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[COORDINATOR] is coordinating agentIDs:";
    for ( int irow = 0 ; irow < vector_of_shouldCoordinate_perRow.length() ; irow++ )
    {
        if (vector_of_shouldCoordinate_perRow[irow])
        {
            QTextStream(stdout) << " " << vector_of_agentID_perRow[irow];
        }
    }
    QTextStream(stdout) << " " << endl;
#endif
}
