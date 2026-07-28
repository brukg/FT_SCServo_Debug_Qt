#include <QApplication>
#include <cmath>
#include "jointplotwidget.h"
int main(int argc, char **argv){
    QApplication a(argc, argv);
    JointPlotWidget p; p.resize(1000, 480);
    p.setJoints({{1,"HLS3955"},{2,"HLS3915"},{4,"HLS2915"}});
    // inject 3 sine-ish position traces over 20s
    for(int k=0;k<400;k++){
        double t = k*0.05;
        p.addSample(1, t, 2048 + 1500*std::sin(t*0.8));
        p.addSample(2, t, 2048 + 1200*std::sin(t*0.8 + 1.0));
        p.addSample(4, t, 2048 +  900*std::sin(t*0.8 + 2.0));
    }
    p.show(); QApplication::processEvents(); QApplication::processEvents();
    p.grab().save("plot.png"); printf("wrote plot.png\n");
    return 0;
}
