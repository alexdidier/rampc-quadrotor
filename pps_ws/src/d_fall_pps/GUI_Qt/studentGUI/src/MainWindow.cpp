#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_rosNodeThread = new rosNodeThread(argc, argv, "student_GUI");
    ui->setupUi(this);
    m_rosNodeThread->init();
    qRegisterMetaType<ptrToMessage>("ptrToMessage");
    QObject::connect(m_rosNodeThread, SIGNAL(newViconData(const ptrToMessage&)), this, SLOT(updateNewViconData(const ptrToMessage&)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateNewViconData(const ptrToMessage& p_msg) //connected to newViconData, from node
{
}
