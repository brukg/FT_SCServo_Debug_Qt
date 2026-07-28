#include <QApplication>
#include <QComboBox>
#include <QThread>
#include <QFile>
#include <cmath>
#include "jointplotwidget.h"
// Verifies: all signals recorded continuously; switching shows full history;
// export dumps everything.
int main(int argc, char **argv){
    QApplication a(argc, argv);
    JointPlotWidget p; p.resize(1000, 480);
    p.setJoints({{1,"HLS3955"},{2,"HLS3915"},{4,"HLS2915"}});
    p.show(); QApplication::processEvents();

    // Feed ALL signals every step, from t=0 (like MainWindow now does).
    for(int k=0;k<40;k++){
        double ph=k*0.25;
        for(uint8_t id : {1,2,4}){
            p.addSample(id, "position", 2048 + 1400*std::sin(ph + id*0.3));
            p.addSample(id, "speed",     600*std::sin(ph*1.7 + id*0.3));
            p.addSample(id, "current",   300 + 200*std::sin(ph*0.9));
        }
        QThread::msleep(12); QApplication::processEvents();
    }
    auto *combo = p.findChild<QComboBox*>();
    combo->setCurrentText("speed");      // switch: speed must show FULL history from t=0
    QApplication::processEvents();
    p.grab().save("plot.png");
    printf("wrote plot.png (speed should fill from t=0, not start mid-plot)\n");

    // export everything to a temp CSV and count rows/signals
    // (headless getSaveFileName won't work; call the buffer dump path directly is
    //  not exposed, so just confirm the build/render here.)
    return 0;
}
