#include "CFLinker.h"


CFLinker::CFLinker(Ui::MainGUIWindow* ui)
{
    m_ui = ui;
}

CFLinker::~CFLinker()
{
}

void CFLinker::link(int student_id, crazyFly* crazyfly, crazyFlyZone* crazyfly_zone)
{
    struct link tmp_link;

    tmp_link.cf_zone_index = crazyfly_zone->getIndex();
    tmp_link.cf_name = crazyfly->getName();

    crazyfly_zone->linkCF(tmp_link.cf_name);
    crazyfly->assignCFZone(tmp_link.cf_zone_index);

    ROS_INFO("tmp_link.cf_name = %s", tmp_link.cf_name.c_str());
    int row_count = m_ui->table_links->rowCount();
    ROS_INFO("row_count %d", row_count);
    m_ui->table_links->insertRow(row_count);
    QString str_id = QString::number(student_id);
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 0, new QTableWidgetItem(str_id));
    QString str_cf_name = QString::fromStdString(crazyfly->getName());
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 1, new QTableWidgetItem(str_cf_name));
    QString str_cf_zone_index = QString("CrazyFlyZone ").append(QString::number(crazyfly_zone->getIndex() + 1));
    m_ui->table_links->setItem(m_ui->table_links->rowCount() - 1, 2, new QTableWidgetItem(str_cf_zone_index));

    links.push_back(tmp_link);
}

void CFLinker::unlink(crazyFly* crazyfly, crazyFlyZone* crazyfly_zone)
{
    bool found = false;
    int index_found;
    for(int i = 0; i < links.size(); i++)
    {
        if(links[i].cf_zone_index == crazyfly_zone->getIndex())
        {
            if(crazyfly->getName() == links[i].cf_name)
            {
                found = true;
                index_found = i;
            }
        }
    }

    if(found)
    {
        crazyfly_zone->removeLink();
        crazyfly->removeAssigned();
        links.erase(links.begin() + index_found);
    }
}
