#include <QApplication>
#include <QComboBox>
#include <QThread>
#include <cmath>
#include "jointplotwidget.h"
// Verifies: add position data, switch to speed, switch BACK to position ->
// the position history must still be there (not lost on signal switch).
int main(int argc, char **argv){
    QApplication a(argc, argv);
    JointPlotWidget p; p.resize(1000, 480);
    p.setJoints({{1,"HLS3955"},{2,"HLS3915"},{4,"HLS2915"}});
    p.show(); QApplication::processEvents();

    auto feed=[&](int base){
        for(int k=0;k<40;k++){
            double ph=k*0.25;
            p.addSample(1, base + 1500*std::sin(ph));
            p.addSample(2, base + 1200*std::sin(ph+1.0));
            p.addSample(4, base +  900*std::sin(ph+2.0));
            QThread::msleep(15); QApplication::processEvents();
        }
    };
    feed(2048);                                   // position data
    auto *combo = p.findChild<QComboBox*>();
    combo->setCurrentText("speed");               // switch away
    feed(0);
    combo->setCurrentText("position");            // switch BACK
    QApplication::processEvents();
    p.grab().save("plot.png");
    printf("wrote plot.png (position history should be visible after round-trip)\n");
    return 0;
}
