#include "CFLinker.h"

#include <QHeaderView>

CFLinker::CFLinker(Ui::MainGUIWindow* ui, std::vector<crazyFly*> *crazyflies_vector, std::vector<crazyFlyZone*> *crazyfly_zones)
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

void CFLinker::addNewRow(int student_id, std::string crazyfly_name, int cf_zone_index)
{
    m_ui->table_links->insertRow(m_ui->table_links->rowCount());
    QString str_id = QString::number(student_id);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 0, new QTableWidgetItem(str_id));
    QString str_cf_name = QString::fromStdString(crazyfly_name);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 1, new QTableWidgetItem(str_cf_name));
    QString str_cf_zone_index = QString("CrazyFlyZone ").append(QString::number(cf_zone_index + 1));
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 2, new QTableWidgetItem(str_cf_zone_index));
}

void CFLinker::link()
{
    m_ui->table_links->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    struct link tmp_link;

    tmp_link.cf_zone_index = getCFZoneIndexFromName(m_ui->comboBoxCFZones->currentText());
    tmp_link.cf_name = m_ui->comboBoxCFs->currentText().toStdString();

    ROS_INFO("tmp_link.cf_zone_index %d", tmp_link.cf_zone_index);
    ROS_INFO("tmp_link.cf_name %s", tmp_link.cf_name.c_str());

    (*m_crazyfly_zones)[tmp_link.cf_zone_index]->linkCF(tmp_link.cf_name);
    (*m_crazyflies_vector)[getCFIndexFromName(tmp_link.cf_name)]->assignCFZone(tmp_link.cf_zone_index);

    addNewRow(m_ui->spinBox_student_ids->value(), tmp_link.cf_name, tmp_link.cf_zone_index);

    links.push_back(tmp_link);
    // TODO: remove options linked from available ones
}

void CFLinker::unlink()
{
    // bool found = false;
    // int index_found;
    // for(int i = 0; i < links.size(); i++)
    // {
    //     if(links[i].cf_zone_index == crazyfly_zone->getIndex())
    //     {
    //         if(crazyfly->getName() == links[i].cf_name)
    //         {
    //             found = true;
    //             index_found = i;
    //         }
    //     }
    // }

    // if(found)
    // {
    //     crazyfly_zone->removeLink();
    //     crazyfly->removeAssigned();
    //     links.erase(links.begin() + index_found);
    // }
}
