#include "CFLinker.h"


CFLinker::CFLinker(QTableWidget* p_table)
{
    m_p_table = p_table;
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

    m_p_table->insertRow(m_p_table->rowCount());
    QString str_id = QString::number(student_id);
    m_p_table->insertItem(m_p_table->rowCount() - 1, 0, new QTableWidgetItem(str_id));
    QString str_cf_name = QString::fromStdString(crazyfly->getName());
    m_p_table->insertItem(m_p_table->rowCount() - 1, 1, new QTableWidgetItem(str_cf_name));
    QString str_cf_zone_index = QString::number(crazyfly_zone->getIndex());
    m_p_table->insertItem(m_p_table->rowCount() - 1, 2, new QTableWidgetItem(str_cf_zone_index));

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
