#include <crazyFlyZoneTab.h>

#include <QLayout>

crazyFlyZoneTab::crazyFlyZoneTab(int index, QWidget *parent)
    : QWidget(parent)
{
    _index = index;
    _num_rows = 3;
    _num_columns = 3;
    center_button = new QPushButton("Fit view");
    QGridLayout *mainLayout = new QGridLayout;
    // mainLayout->setRowMinimumHeight(1, 25);
    // mainLayout->setRowMinimumHeight(2, 25);
    // mainLayout->setRowStretch(1);
    // mainLayout->setColumnMinimumWidth(5);
    for(int i = 0; i < _num_rows; i++)
    {
        mainLayout->setRowStretch(i, 1);
    }

    for(int i = 0; i < _num_columns; i++)
    {
        mainLayout->setColumnStretch(i, 1);
    }

    mainLayout->addWidget(center_button, _num_rows - 1, _num_columns - 1);
    setLayout(mainLayout);
    QObject::connect(center_button, SIGNAL(clicked()), this, SLOT(centerButtonClicked()));
    qDebug("tab widget created, index: %d", _index);
}

void crazyFlyZoneTab::centerButtonClicked()
{
    qDebug("index clicked: %d", _index);
    emit centerButtonClickedSignal(_index);
}
