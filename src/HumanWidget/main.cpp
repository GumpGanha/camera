#include <QApplication>
#include <QTextCodec>
//#include <QLocale>
//#include <QTranslator>
#include "HumanWidget.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));

    HumanWidget   w;
    w.show();
    return a.exec();
}
