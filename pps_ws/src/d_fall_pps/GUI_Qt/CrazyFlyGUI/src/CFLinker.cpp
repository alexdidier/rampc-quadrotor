//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero
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
//    Links the crazyflies with the student ids and with the crazyflie zones
//
//    ----------------------------------------------------------------------------------


#include "CFLinker.h"

#include <QHeaderView>

CFLinker::CFLinker(Ui::MainGUIWindow* ui, std::vector<crazyFly*> *crazyflies_vector, std::vector<crazyFlyZone*> *crazyfly_zones)
    : QObject(0)
{
    m_ui = ui;
    m_crazyflies_vector = crazyflies_vector;
    m_crazyfly_zones = crazyfly_zones;
}

CFLinker::~CFLinker()
{
}

int CFLinker::getCFZoneIndexFromName(QString name)
{
    return name.split(" ")[1].toInt() - 1;
}

int CFLinker::getCFIndexFromName(std::string name)
{
    for(int i = 0; (*m_crazyflies_vector).size(); i++)
    {
        if(name == (*m_crazyflies_vector)[i]->getName())
        {
            return i;
        }
    }
}

bool CFLinker::isStudentIDLinked(int student_id)
{
    bool is_linked = false;
    for(int i = 0; i < links.size(); i++)
    {
        if(links[i].student_id == student_id)
        {
            is_linked = true;
        }
    }

    return is_linked;
}

bool CFLinker::isRadioAddressLinked(std::string radio_address)
{
    bool is_linked = false;
    for(int i = 0; i < links.size(); i++)
    {
        if(links[i].radio_address == radio_address)
        {
            is_linked = true;
        }
    }
    return is_linked;
}

bool CFLinker::isCFZoneLinked(int cf_zone_index)
{
    bool is_linked = false;
    for(int i = 0; i < links.size(); i++)
    {
        if(links[i].cf_zone_index == cf_zone_index)
        {
            is_linked = true;
        }
    }

    return is_linked;
}

bool CFLinker::isCFLinked(std::string cf_name)
{
    bool is_linked = false;
    for(int i = 0; i < links.size(); i++)
    {
        if(links[i].cf_name == cf_name)
        {
            is_linked = true;
        }
    }
    return is_linked;
}


void CFLinker::addNewRow(int student_id, std::string crazyfly_name, int cf_zone_index, std::string radio_address)
{
    m_ui->table_links->insertRow(m_ui->table_links->rowCount());
    QString str_id = QString::number(student_id);
    QTableWidgetItem *item_id = new QTableWidgetItem(str_id);
    item_id->setFlags(item_id->flags() & ~Qt::ItemIsEditable);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 0, item_id);

    QString str_cf_name = QString::fromStdString(crazyfly_name);
    QTableWidgetItem *item_cf = new QTableWidgetItem(str_cf_name);
    item_cf->setFlags(item_cf->flags() & ~Qt::ItemIsEditable);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 1, item_cf);

    QString str_cf_zone_index = QString("CrazyFlyZone ").append(QString::number(cf_zone_index + 1));
    QTableWidgetItem *item_cf_zone = new QTableWidgetItem(str_cf_zone_index);
    item_cf_zone->setFlags(item_cf_zone->flags() & ~Qt::ItemIsEditable);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 2, item_cf_zone);

    QString str_radio_address = QString::fromStdString(radio_address);
    QTableWidgetItem *item_radio_address = new QTableWidgetItem(str_radio_address);
    item_cf->setFlags(item_radio_address->flags() & ~Qt::ItemIsEditable);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 3, item_radio_address);
}

void CFLinker::link(int student_id, int cf_zone_index, std::string cf_name, std::string radio_address)
{
    m_ui->table_links->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    struct link tmp_link;

    tmp_link.student_id = student_id;
    tmp_link.cf_zone_index = cf_zone_index;
    tmp_link.cf_name = cf_name;
    tmp_link.radio_address = radio_address;

    ROS_INFO("tmp_link.student_id %d", tmp_link.student_id);
    ROS_INFO("tmp_link.cf_zone_index %d", tmp_link.cf_zone_index);
    ROS_INFO("tmp_link.cf_name %s", tmp_link.cf_name.c_str());
    ROS_INFO("tmp_link.radio_address %s", tmp_link.radio_address.c_str());

    // (*m_crazyfly_zones)[tmp_link.cf_zone_index]->linkCF(tmp_link.cf_name);
    // (*m_crazyflies_vector)[getCFIndexFromName(tmp_link.cf_name)]->assignCFZone(tmp_link.cf_zone_index);

    addNewRow(tmp_link.student_id, tmp_link.cf_name, tmp_link.cf_zone_index, tmp_link.radio_address);

    links.push_back(tmp_link);
    // TODO: remove options linked from available ones

    // Get the index of the currently selected CF
    int index = m_ui->comboBoxCFs->currentIndex();
    // Remove item from the CF Combo Box
    m_ui->comboBoxCFs->removeItem(index);


    // THIS WAS COMMENTED OUT TO ALLOW ASSIGNING MULTIPLE CRAYZFLIES TO ONE ZONE
    // Get the index of the currently selected CF
    //index = m_ui->comboBoxCFZones->currentIndex();
    // Remove item from the CF Combo Box
    //m_ui->comboBoxCFZones->removeItem(index);

    // disable item
    // m_ui->comboBoxCFs->setItemData(index, 0, Qt::UserRole - 1);
    // enable item
    // ui->comboBox->setItemData(index, 33, Qt::UserRole - 1);
}

void CFLinker::clear_all_links()
{
    links.clear();
    m_ui->table_links->setRowCount(0);
    emit updateComboBoxes();
}

void CFLinker::unlink_cf_zone(int cf_zone_index)
{
    for(int i = 0; i < links.size(); i++)
    {
        if(links[i].cf_zone_index == cf_zone_index)
        {
            links.erase(links.begin() + i);
            break;
        }
    }
    // remove them graphically
    for(int i = 0; i < m_ui->table_links->rowCount(); i++)
    {
        QString name = m_ui->table_links->item(i, 2)->text(); //2 because cf zone
        if(getCFZoneIndexFromName(name) == cf_zone_index)
        {
            m_ui->table_links->removeRow(i);
            break;
        }
    }
    // update combo boxes
    emit updateComboBoxes();
}

void CFLinker::unlink_selection()
{
    QModelIndexList selection = m_ui->table_links->selectionModel()->selectedRows();


    // first, get an ordered from greater to lesser vector of indexes of selected rows
    std::vector<int> ordered_row_indexes;

    for(int i = selection.count() - 1; i >= 0; i--) // fill vector first
    {
        QModelIndex index = selection.at(i);
        ordered_row_indexes.push_back(index.row());
    }

    // sort using a standard library compare function object, greater to lesser
    std::sort(ordered_row_indexes.begin(), ordered_row_indexes.end(), std::greater<int>());

    // now, unlink them in data structure and graphically removing rows from the table

    for(int i = 0; i < ordered_row_indexes.size(); i++)
    {
        // TODO: unset linked status in its corresponding objects
        // remove them from links vector. We will look for student as unique key.. Maybe in a future the CF is the unique key?
        int student_id = m_ui->table_links->item(ordered_row_indexes[i], 0)->text().toInt(); // 0: student ID
        for(int i = 0; i < links.size(); i++)
        {
            if(links[i].student_id == student_id)
            {
                links.erase(links.begin() + i);
                break;
            }
        }
        // remove them graphically
        m_ui->table_links->removeRow(ordered_row_indexes[i]);
    }

    // update combo boxes
    emit updateComboBoxes();
}
