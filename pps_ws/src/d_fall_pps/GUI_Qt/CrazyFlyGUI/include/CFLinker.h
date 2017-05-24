#ifndef CFLINKER_H
#define CFLINKER_H

#include "globalDefinitions.h"
#include "crazyFly.h"
#include "crazyFlyZone.h"
#include "ui_mainguiwindow.h"

#include "rosNodeThread.h"

#include <QObject>


class CFLinker : public QObject
{
    Q_OBJECT
public:

    struct link {
        int student_id;
        int cf_zone_index;
        std::string cf_name;
    };

    explicit CFLinker(Ui::MainGUIWindow* ui, std::vector<crazyFly*> *crazyflies_vector, std::vector<crazyFlyZone*> *crazyfly_zones);
    ~CFLinker();

    void link();
    void unlink();

    std::vector<struct link> links;

    bool isStudentIDLinked(int student_id);
    bool isCFZoneLinked(int cf_zone_index);
    bool isCFLinked(std::string cf_name);

signals:
    void updateComboBoxes();

private:

    // QTableWidget m_p_table;

    Ui::MainGUIWindow* m_ui;
    std::vector<crazyFly*>* m_crazyflies_vector;
    std::vector<crazyFlyZone*>* m_crazyfly_zones;

    int getCFZoneIndexFromName(QString name);
    int getCFIndexFromName(std::string name);
    void addNewRow(int student_id, std::string crazyfly_name, int cf_zone_index);
};


#endif
