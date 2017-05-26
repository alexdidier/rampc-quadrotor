#include "mainguiwindow.h"
#include "ui_mainguiwindow.h"
#include "crazyFlyZoneTab.h"
#include "myGraphicsScene.h"
#include "myGraphicsView.h"

#include <QObject>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QString>
#include <QMetaType>
#include <QDir>
#include <regex>

#ifdef CATKIN_MAKE
#include "d_fall_pps/UnlabeledMarker.h"
#endif

#include <string>

#define N_MAX_CRAZYFLIES           20 // protection number

#ifdef CATKIN_MAKE
using namespace d_fall_pps;
#endif

MainGUIWindow::MainGUIWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainGUIWindow)
{
    #ifdef CATKIN_MAKE
    _rosNodeThread = new rosNodeThread(argc, argv, "/ViconDataPublisher/ViconData");
    #endif
    ui->setupUi(this);
    _init();
}


MainGUIWindow::~MainGUIWindow()
{
    delete ui;
}

int MainGUIWindow::getTabIndexFromName(QString name)
{
    int found_name = -1;
    for(int i = 0; i < ui->tabWidget->count(); i++)
    {
        qDebug("name: %s", name.toStdString().c_str());
        qDebug("tabText: %s", ui->tabWidget->tabText(i).toStdString().c_str());
        if(name == ui->tabWidget->tabText(i))
        {
            found_name = i;
        }
    }
    return found_name;
}

void MainGUIWindow::doNumCrazyFlyZonesChanged(int n)
{
    // tabs number management, maybe do it in a different way so we dont have to remove and add everything?
    // first check if size of tabs is greater than size of vector or viceversa. Have we removed or added a zone?
    qDebug("tabWidgetCount : %d", ui->tabWidget->count());
    if(ui->tabWidget->count() > scene->crazyfly_zones.size())
    {
        // we removed one crazyfly_zone, n means index of the one we removed. Look for that index tab and remove it
        QString qstr = "CrazyFly ";
        qstr.append(QString::number(n+1));
        if(scene->crazyfly_zones.size() == 0)
        {
            ui->tabWidget->clear();
        }
        int found_index = getTabIndexFromName(qstr);
        if(found_index != -1)
        {
            ui->tabWidget->removeTab(found_index);
        }

        //  now unlink it from table also:
        if(cf_linker->isCFZoneLinked(n))
        {
            cf_linker->unlink_cf_zone(n);
        }
    }
    else if(ui->tabWidget->count() < scene->crazyfly_zones.size())
    {
        // we added one crazyfly_zone, n means index of the new one. New tab will be labeld index + 1
        QString qstr = "CrazyFly ";
        qstr.append(QString::number(n+1));
        crazyFlyZoneTab* widget = new crazyFlyZoneTab(n);
        ui->tabWidget->insertTab(n, widget, qstr);
        connect(widget, SIGNAL(centerButtonClickedSignal(int)), this, SLOT(centerViewIndex(int)));
    }
    // for (int i = 0; i < n; i++)
    // {
    //     QString qstr = "CrazyFly ";
    //     qstr.append(QString::number(i+1));
    //     crazyFlyZoneTab* widget = new crazyFlyZoneTab(i);
    //     ui->tabWidget->addTab(widget, qstr);
    //     connect(widget, SIGNAL(centerButtonClickedSignal(int)), this, SLOT(centerViewIndex(int)));
    // }

    updateComboBoxesCFZones();
}

void MainGUIWindow::_init()
{
    // initialize checkboxes, spinboxes,....
    ui->scaleSpinBox->setRange(0.1, 100);
    ui->scaleSpinBox->setSingleStep(0.1);
    ui->scaleSpinBox->setValue(1);

    ui->checkBox_vicon_crazyflies->setChecked(false);
    ui->scaleSpinBox->setEnabled(false);


    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    // error messages
    // ui->err_message_cf->hide();
    // ui->err_message_cf_zone->hide();
    // ui->err_message_student_id->hide();

   ui->err_message_cf->setStyleSheet("QLabel { color : red; }");
   ui->err_message_cf_zone->setStyleSheet("QLabel { color : red; }");
   ui->err_message_student_id->setStyleSheet("QLabel { color : red; }");

   ui->err_message_cf->clear();
   ui->err_message_cf_zone->clear();
   ui->err_message_student_id->clear();

    // initialize table_links
    ui->table_links->setColumnCount(3);

    QFont fnt;
    fnt.setPointSize(7);
    ui->table_links->horizontalHeader()->setFont(fnt);

    ui->table_links->horizontalHeader()->setDefaultSectionSize(90);
    ui->table_links->verticalHeader()->setDefaultSectionSize(20);

    const int rowCount = ui->table_links->rowCount();
    const int columnCount = ui->table_links->columnCount();
    for(int i = 0; i < rowCount; ++i)
    {
    	for(int j = 0; j < columnCount; ++j)
        {
    		QTableWidgetItem* selectedItem = ui->table_links->item(i, j);
    		selectedItem->setFont(fnt);
    	}
    }
    ui->table_links->setSelectionBehavior(QAbstractItemView::SelectRows);
    QStringList horizontal_header;
    horizontal_header << "Student ID" << "CrazyFly" << "CrazyFly Zone";
    ui->table_links->setHorizontalHeaderLabels(horizontal_header);

    // scene
    scene = new myGraphicsScene(ui->frame_drawing);
    scene->setSceneRect(-100 * FROM_METERS_TO_UNITS, -100 * FROM_METERS_TO_UNITS, 200 * FROM_METERS_TO_UNITS, 200 * FROM_METERS_TO_UNITS);

    ui->graphicsView->setScene(scene);

    // after scene is created, create CFlinker
    #ifdef CATKIN_MAKE
    cf_linker = new CFLinker(ui, &crazyflies_vector, &scene->crazyfly_zones);
    #endif
    // connections
    QObject::connect(ui->tabWidget, SIGNAL(tabCloseRequested(int)), this, SLOT(doTabClosed(int)));
    QObject::connect(scene, SIGNAL(numCrazyFlyZonesChanged(int)), this, SLOT(doNumCrazyFlyZonesChanged(int)));
    QObject::connect(scene, SIGNAL(crazyFlyZoneSelected(int)), this, SLOT(setTabIndex(int)));
    QObject::connect(scene, SIGNAL(modeChanged(int)), this, SLOT(transitionToMode(int)));
    QObject::connect(scene, SIGNAL(numTablePiecesChanged(int)), this, SLOT(handleTablePiecesNumChanged(int)));

    ui->checkBox_vicon_highlight_markers->setEnabled(false);

    #ifdef CATKIN_MAKE
    _rosNodeThread->init();
    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));
    QObject::connect(cf_linker, SIGNAL(updateComboBoxes()), this, SLOT(updateComboBoxes()));
    #endif
}

void MainGUIWindow::doTabClosed(int tab_index)
{
    QString name = ui->tabWidget->tabText(tab_index);
    int cf_index = cf_linker->getCFZoneIndexFromName(name);
    scene->removeCrazyFlyZone(cf_index);
}

void MainGUIWindow::setTabIndex(int index)
{
    QString qstr = "CrazyFly ";
    qstr.append(QString::number(index + 1));
    ui->tabWidget->setCurrentIndex(getTabIndexFromName(qstr));
}

void MainGUIWindow::updateComboBoxes()
{
    updateComboBoxesCFs();
    updateComboBoxesCFZones();
}
void MainGUIWindow::updateComboBoxesCFs()
{
    #ifdef CATKIN_MAKE
    ui->comboBoxCFs->clear();
    for(int i = 0; i < crazyflies_vector.size(); i++)
    {
        if(!cf_linker->isCFLinked(crazyflies_vector[i]->getName()))
        {
            QString qstr = QString::fromStdString(crazyflies_vector[i]->getName());
            ui->comboBoxCFs->addItem(qstr);
        }
    }
    #endif
}

void MainGUIWindow::updateComboBoxesCFZones()
{
    ui->comboBoxCFZones->clear();
    #ifdef CATKIN_MAKE
    for(int i = 0; i < scene->crazyfly_zones.size(); i++)
    {
        if(!cf_linker->isCFZoneLinked(scene->crazyfly_zones[i]->getIndex()))
        {
            int cf_zone_index = scene->crazyfly_zones[i]->getIndex();
            QString qstr = "CrazyFlyZone ";
            qstr.append(QString::number(cf_zone_index + 1));
            ui->comboBoxCFZones->addItem(qstr);
        }
    }
    #endif
}


#ifdef CATKIN_MAKE
void MainGUIWindow::updateNewViconData(const ptrToMessage& p_msg) //connected to newViconData, from node
{

    // update Markers

    if(p_msg->markers.size() < markers_vector.size()) // some markers have dissapeared, received stuff is smaller than what we have
    {
        for(int i = p_msg->markers.size(); i < markers_vector.size(); i++)
        {
            scene->removeItem(markers_vector[i]); // remove objects from scene
            // ROS_INFO_STREAM("element index: " << i << " removed");
        }
        markers_vector.erase(markers_vector.begin() + p_msg->markers.size(), markers_vector.end()); //delete them
    }

    // ROS_INFO_STREAM("markers.size: " << p_msg->markers.size());

    for(int i = 0; i < p_msg->markers.size(); i++) // here, or new markers message is equal to current messages, or greater (some new markers)
    {
        if(i >= markers_vector.size()) //some new markers coming
        {
            // ROS_INFO_STREAM("element index: " << i << " added");
            Marker* tmp_p_marker = new Marker(&(p_msg->markers[i]));
            markers_vector.push_back(tmp_p_marker); // what happens with the new indexes? check if this is correct

            if(ui->checkBox_vicon_markers->checkState() == Qt::Checked) //only if markers checkbox info is checked..
            {
                scene->addItem(markers_vector[i]);
                if(ui->checkBox_vicon_highlight_markers->checkState() == Qt::Checked)
                {
                    markers_vector[i]->setHighlighted();
                }
            }
        }
        else
        {
            // ROS_INFO_STREAM("element index: " << i << " moved, already existed");
            markers_vector[i]->updateMarker(&(p_msg->markers[i]));
        }
    }

    // update Crazyflies
    // also: what happens if we dont go through one of the names? we need to remove that crazyfly
    int crazyfly_vector_size_before = crazyflies_vector.size(); //initial size of vector
    // in this loop, add new ones and update old ones
    for(int i = 0; i < p_msg->crazyflies.size(); i++)
    {
        bool name_found = false; // for each iteration, name_found starts in false
        int index_name_found;
        for(int j = 0; j < crazyfly_vector_size_before; j++)
        {
            if(crazyflies_vector[j]->getName() == p_msg->crazyflies[i].crazyflieName)
            {
                name_found = true; // name found. This can only happen once per i-iteration, names are unique
                index_name_found = j; // index in already existing vector, to update it later (really needed?)
            }
        }

        if(name_found)
        {
            crazyflies_vector[index_name_found]->updateCF(&(p_msg->crazyflies[i]));
        }
        else                    //name not found, newly arrived, add it to the vector
        {
            crazyFly* tmp_p_crazyfly = new crazyFly(&(p_msg->crazyflies[i]));
            crazyflies_vector.push_back(tmp_p_crazyfly);
        }

        if(ui->checkBox_vicon_crazyflies->checkState() == Qt::Checked)
        {
            for(int i = 0; i < crazyflies_vector.size(); i++) //check for occlussion
            {
                if(crazyflies_vector[i]->isOccluded())
                {
                    ROS_INFO("===================OCCLUDED");
                    if(crazyflies_vector[i]->isAddedToScene())
                    {
                        scene->removeItem(crazyflies_vector[i]);
                        crazyflies_vector[i]->setAddedToScene(false);
                    }
                }
                else
                {
                    if(!crazyflies_vector[i]->isAddedToScene())
                    {
                        scene->addItem(crazyflies_vector[i]);
                        crazyflies_vector[i]->setAddedToScene(true);
                    }
                }
            }
        }
    }

    // in this loop, clean the ones that are not present anymore. UPDATE: this will apparently only happen when we tick and untick in Vicon
    int crazyfly_vector_size_after = crazyflies_vector.size();

    for(int j = 0; j < crazyfly_vector_size_after; j++)
    {
        bool name_found = false;
        for(int i = 0; i < p_msg->crazyflies.size(); i++)
        {
            if(crazyflies_vector[j]->getName() == p_msg->crazyflies[i].crazyflieName)
            {
                name_found = true;
            }
        }
        if(!name_found)
        {
            scene->removeItem(crazyflies_vector[j]);
            crazyflies_vector.erase(crazyflies_vector.begin() + j);
        }
    }
}
#endif


void MainGUIWindow::on_removeTable_clicked()
{
    if(scene->getMode() == myGraphicsScene::mode_table)
    {
        scene->removeTable();
    }
}

void MainGUIWindow::transitionToMode(int mode)
{
    switch(mode)
    {
        case myGraphicsScene::mode_table:
        {
            ui->removeTable->setDisabled(false);
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            ui->removeTable->setDisabled(true);
            break;
        }
    }
}

void MainGUIWindow::on_radioButton_table_mode_toggled(bool checked)
{
    switch(scene->getMode())
    {
        case myGraphicsScene::mode_table:
        {
            // already in the mode we want, do nothing
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            scene->setMode(myGraphicsScene::mode_table);
            break;
        }
        case myGraphicsScene::mode_locked:
        {
            scene->setMode(myGraphicsScene::mode_table);
            break;
        }
    }

}


void MainGUIWindow::on_radioButton_crazyfly_zones_mode_toggled(bool checked)
{
    switch(scene->getMode())
    {
        case myGraphicsScene::mode_table:
        {
            scene->setMode(myGraphicsScene::mode_crazyfly_zones);
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            // already in the mode we want, do nothing
            break;
        }
        case myGraphicsScene::mode_locked:
        {
            scene->setMode(myGraphicsScene::mode_crazyfly_zones);
            break;
        }
    }
}

void MainGUIWindow::handleTablePiecesNumChanged(int newNum)
{

}

void MainGUIWindow::on_radioButton_lock_mode_toggled(bool checked)
{
    switch(scene->getMode())
    {
        case myGraphicsScene::mode_table:
        {
            scene->setMode(myGraphicsScene::mode_locked);
            break;
        }
        case myGraphicsScene::mode_crazyfly_zones:
        {
            scene->setMode(myGraphicsScene::mode_locked);
            break;
        }
        case myGraphicsScene::mode_locked:
        {
            break;
        }
    }
}

void MainGUIWindow::on_checkBox_grid_toggled(bool checked)
{
    scene->setGrid(checked);
}

void MainGUIWindow::on_checkBox_table_toggled(bool checked)
{
    if(checked)
    {
        scene->showTable();
    }
    else
    {
        scene->hideTable();
    }
}

void MainGUIWindow::on_checkBox_crazyfly_zones_toggled(bool checked)
{
    if(checked)
    {
        scene->showCrazyFlyZones();
    }
    else
    {
        scene->hideCrazyFlyZones();
    }
}

void MainGUIWindow::on_tabWidget_currentChanged(int index)
{
    // this index is tab index. Need to go to cf index
    QString name = ui->tabWidget->tabText(index);
    int cf_index = cf_linker->getCFZoneIndexFromName(name);
    scene->setSelectedCrazyFlyZone(cf_index);
}

void MainGUIWindow::centerViewIndex(int index)
{
    ui->graphicsView->fitInView(scene->getRectFCrazyFlyZone(index), Qt::KeepAspectRatio);
    ui->graphicsView->scale(0.95, 0.95); // A bit back zoom, so we can see everything better
}


void MainGUIWindow::on_pushButton_fitAll_clicked()
{
    ui->graphicsView->fitInView(scene->itemsBoundingRect(), Qt::KeepAspectRatio);
    ui->graphicsView->scale(0.95, 0.95); // A bit back zoom, so we can see everything better
}

void MainGUIWindow::on_checkBox_vicon_markers_toggled(bool checked)
{
    if(checked)
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            scene->addItem(markers_vector[i]);
        }
        #endif
        ui->checkBox_vicon_highlight_markers->setCheckable(true);
        ui->checkBox_vicon_highlight_markers->setEnabled(true);
    }
    else
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            scene->removeItem(markers_vector[i]);
        }
        #endif
        ui->checkBox_vicon_highlight_markers->setChecked(false);
        ui->checkBox_vicon_highlight_markers->setCheckable(false);
        ui->checkBox_vicon_highlight_markers->setEnabled(false);
    }
}

void MainGUIWindow::on_checkBox_vicon_highlight_markers_toggled(bool checked)
{
    if(checked)
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            markers_vector[i]->setHighlighted();
        }
        #endif
    }
    else
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < markers_vector.size(); i++)
        {
            markers_vector[i]->clearHighlighted();
        }
        #endif
    }
}

void MainGUIWindow::on_checkBox_vicon_crazyflies_toggled(bool checked)
{
    if(checked)
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < crazyflies_vector.size(); i++)
        {
            if(!crazyflies_vector[i]->isAddedToScene())
            {
                scene->addItem(crazyflies_vector[i]);
                crazyflies_vector[i]->setAddedToScene(true);
            }
        }
        #endif
        ui->scaleSpinBox->setEnabled(true);
    }
    else
    {
        #ifdef CATKIN_MAKE
        for(int i = 0; i < crazyflies_vector.size(); i++)
        {
            if(crazyflies_vector[i]->isAddedToScene())
            {
                scene->removeItem(crazyflies_vector[i]);
                crazyflies_vector[i]->setAddedToScene(false);
            }
        }
        #endif
        ui->scaleSpinBox->setEnabled(false);
    }
}

void MainGUIWindow::on_scaleSpinBox_valueChanged(double arg1)
{
    #ifdef CATKIN_MAKE
    for(int i = 0; i < crazyflies_vector.size(); i++)
    {
        crazyflies_vector[i]->setScaleCFs(arg1);
    }
    #endif
}

void MainGUIWindow::on_refresh_cfs_button_clicked()
{
    updateComboBoxesCFs();
}

void MainGUIWindow::on_refresh_student_ids_button_clicked()
{
    #ifdef CATKIN_MAKE
    ui->list_discovered_student_ids->clear();

    // \/(\d)\/PPSClient
    ros::V_string v_str;
    ros::master::getNodes(v_str);
    for(int i = 0; i < v_str.size(); i++)
    {
        std::string s = v_str[i];
        std::smatch m;
        std::regex e ("\\/(\\d)\\/PPSClient");

        // std::regex e("\\/PPSClien(.)");

        // while(std::regex_search(s, m, e))
        // {
        //     for (int i = 0; i < m.size(); i++)
        //     {
        //         ROS_INFO("FOUND: %s", m[i].str().c_str());
        //         // std::cout << "FOUND" << m[i] << "\n";
        //     }
        //     s = m.suffix().str();
        // }

        if(std::regex_search(s, m, e))
        {
            // ROS_INFO("===============================================FOUND: %s", m[1].str().c_str()); // one because we are interested ONLY in the first match
            std::string found_string = m[1].str();
            ui->list_discovered_student_ids->addItem(found_string.c_str());
        }
    }
    #endif
}



void MainGUIWindow::on_link_button_clicked()
{
    #ifdef CATKIN_MAKE

    bool error = false;
    if(ui->comboBoxCFs->count() == 0)
    {
        // plot error message
        ui->err_message_cf->setText("CF box is empty");
        error = true;
    }
    else
    {
        ui->err_message_cf->clear();
    }
    if(ui->comboBoxCFZones->count() == 0)
    {
        // plot error message
        ui->err_message_cf_zone->setText("CFZone box is empty");
        error = true;
    }
    else
    {
        ui->err_message_cf_zone->clear();
    }

    if(cf_linker->isStudentIDLinked(ui->spinBox_student_ids->value()))
    {
        // plot error message
        ui->err_message_student_id->setText("This StudentID has already been linked");
        error = true;
    }
    else
    {
        ui->err_message_student_id->clear();
    }

    if(!error)
    {
        cf_linker->link();
    }
    #endif
}

void MainGUIWindow::on_unlink_button_clicked()
{
    #ifdef CATKIN_MAKE
    cf_linker->unlink_selection();
    #endif
}
