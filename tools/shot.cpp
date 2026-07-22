// Render MainWindow tabs at given sizes to PNG. Headless via offscreen platform.
#include <QApplication>
#include <QPixmap>
#include <QTabWidget>
#include "mainwindow.h"
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    { MainWindow w; w.setMinimumSize(0,0); w.show(); QApplication::processEvents();
      printf("REQUIRED minimumSizeHint = %dx%d\n",
             w.minimumSizeHint().width(), w.minimumSizeHint().height()); }
    for(int i=2;i+1<argc;i+=2){
        for(int tab=0; tab<2; tab++){
            MainWindow w; w.resize(QString(argv[i]).toInt(), QString(argv[i+1]).toInt());
            if(auto *tw = w.findChild<QTabWidget*>("tabWidget")) tw->setCurrentIndex(tab);
            w.show(); QApplication::processEvents();
            QString f = QString("%1-tab%2-%3x%4.png").arg(argv[1]).arg(tab).arg(w.width()).arg(w.height());
            w.grab().save(f);
            printf("wrote %s\n", qPrintable(f));
        }
    }
    return 0;
}
