#ifndef CFLINKER_H
#define CFLINKER_H

#include "globalDefinitions.h"
#include "crazyFly.h"
#include "crazyFlyZone.h"

#include <QTableWidget>

class CFLinker
{
public:
    explicit CFLinker(QTableWidget* p_table);
    ~CFLinker();

    void link(int student_id, crazyFly* crazyfly, crazyFlyZone* crazyfly_zone);

    void unlink(crazyFly* crazyfly,  crazyFlyZone* crazyfly_zone);

private:

    struct link {
        int cf_zone_index;
        std::string cf_name;
    };

    std::vector<struct link> links;

    // table
    QTableWidget* m_p_table;
};


#endif
