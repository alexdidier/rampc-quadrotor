#ifndef CFLINKER_H
#define CFLINKER_H

#include "globalDefinitions.h"
#include "crazyFly.h"
#include "crazyFlyZone.h"

class CFLinker
{
public:
    explicit CFLinker();
    ~CFLinker();

    void link(crazyFly* crazyfly, crazyFlyZone* crazyfly_zone);
    void link(crazyFlyZone* crazyfly_zone, crazyFly* crazyfly);

    void unlink(crazyFly* crazyfly,  crazyFlyZone* crazyfly_zone);

private:

    struct link {
        int cf_zone_index;
        std::string cf_name;
    };

    std::vector<struct link> links;
};


#endif
