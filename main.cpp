#include "uav.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    uav w;
    w.show();
    w.setMainWindowTitle();
    return a.exec();
}
