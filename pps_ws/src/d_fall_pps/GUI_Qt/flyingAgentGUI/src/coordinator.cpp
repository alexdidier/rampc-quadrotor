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


#ifdef CATKIN_MAKE
    // USE A REGULAR EXPRESSION TO IDENTIFY NODES THAT EXIST
    // > The regular expression we use is: \/agent(\d\d\d)\/PPSClient
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
        std::regex e ("\\/agent(\\d\\d\\d)\\/PPSClient");

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
            if (ui->coordinate_all_checkBox->isChecked())
            {
                temp_coordinatorRow->setShouldCoordinate(true);
            }
            else
            {
                temp_coordinatorRow->setShouldCoordinate(false);
            }

            // Add to the vector of coordinator rows
            vector_of_coordinatorRows.append(temp_coordinatorRow);

            ui->coordinated_agents_scrollAreaWidgetContents->layout()->addWidget(temp_coordinatorRow);
        }
    }
#else


    for ( int i_agent = 1 ; i_agent < 9 ; i_agent++ )
    {
        //ui->scrollAreaWidgetContents->setLayout(new QVBoxLayout);

        CoordinatorRow *temp_coordinatorRow = new CoordinatorRow(this,i_agent);

        // Check the box if "coordinate all" is checked
        if (ui->coordinate_all_checkBox->isChecked())
        {
            temp_coordinatorRow->setShouldCoordinate(true);
        }
        else
        {
            temp_coordinatorRow->setShouldCoordinate(false);
        }

        // Add to the vector of coordinator rows
        vector_of_coordinatorRows.append(temp_coordinatorRow);

        ui->coordinated_agents_scrollAreaWidgetContents->layout()->addWidget(temp_coordinatorRow);

        // Call the function that applies this level
        // of detail to all the entries
        apply_level_of_detail_to_all_entries(level_of_detail_to_display);
    }
#endif
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
}

void Coordinator::remove_all_entries_from_vector_of_coordinatorRows()
{
    // Iterate through and delete all entries
    foreach ( CoordinatorRow* temp_coordinatorRow, vector_of_coordinatorRows) {
        delete( temp_coordinatorRow );
    }
    // Clear the vector
    vector_of_coordinatorRows.clear();
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
    }
}
