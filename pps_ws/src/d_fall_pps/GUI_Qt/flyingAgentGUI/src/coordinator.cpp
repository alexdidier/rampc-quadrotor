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


    for ( int i_agent = 1 ; i_agent < 11 ; i_agent++ )
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
    }
}

void Coordinator::on_delete_button_clicked()
{

    foreach ( CoordinatorRow* temp_coordinatorRow, vector_of_coordinatorRows) {
        delete( temp_coordinatorRow );
    }

    vector_of_coordinatorRows.clear();

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
