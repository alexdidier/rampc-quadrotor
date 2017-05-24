#ifndef CFLINKER_H
#define CFLINKER_H

#include "globalDefinitions.h"
#include "crazyFly.h"
#include "crazyFlyZone.h"
#include "ui_mainguiwindow.h"

#include "rosNodeThread.h"


class CFLinker
{
public:
    explicit CFLinker(Ui::MainGUIWindow* ui);
    ~CFLinker();

    void link(int student_id, crazyFly* crazyfly, crazyFlyZone* crazyfly_zone);

    void unlink(crazyFly* crazyfly,  crazyFlyZone* crazyfly_zone);

private:

    struct link {
        int cf_zone_index;
        std::string cf_name;
    };

    std::vector<struct link> links;

    // QTableWidget m_p_table;

    Ui::MainGUIWindow* m_ui;
};


#endif
