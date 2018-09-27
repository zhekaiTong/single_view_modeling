#include "single_view_modeling.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    single_view_modeling w;
    w.show();

    return a.exec();
}
