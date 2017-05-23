#include "CFLinker.h"


CFLinker::CFLinker()
{
}

CFLinker::~CFLinker()
{
}

void CFLinker::link(crazyFly* crazyfly, crazyFlyZone* crazyfly_zone)
{
    struct link tmp_link;

    tmp_link.cf_zone_index = crazyfly_zone->getIndex();
    tmp_link.cf_name = crazyfly->getName();

    crazyfly_zone->linkCF(tmp_link.cf_name);
    crazyfly->assignCFZone(tmp_link.cf_zone_index);

    links.push_back(tmp_link);
}

void CFLinker::link(crazyFlyZone* crazyfly_zone, crazyFly* crazyfly)
{
    link(crazyfly, crazyfly_zone);
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
