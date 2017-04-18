#include <crazyFlyZoneTab.h>

#include <QLayout>

crazyFlyZoneTab::crazyFlyZoneTab(int index, QWidget *parent)
    : QWidget(parent)
{
    _index = index;
    center_button = new QPushButton("Fit view");
    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(center_button);
    setLayout(mainLayout);
    QObject::connect(center_button, SIGNAL(clicked()), this, SLOT(centerButtonClicked()));
    qDebug("tab widget created, index: %d", _index);
}

void crazyFlyZoneTab::centerButtonClicked()
{
    qDebug("index clicked: %d", _index);
    emit centerButtonClickedSignal(_index);
}
