#include <QApplication>
#include "jointcontroltab.h"
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    JointControlTab t; t.resize(900, 400);
    auto mk=[](uint8_t id, feetech_servo::ModelSeries s, const char* n, int pos){
        feetech_servo::ServoProfile p; p.name=n; p.series=s; p.end=0; p.known=true;
        return feetech_servo::GroupTarget{id, p, pos};   // pos = present position
    };
    std::vector<feetech_servo::GroupTarget> v = {
        mk(1, feetech_servo::HLS, "HLS3955", 3083),
        mk(2, feetech_servo::HLS, "HLS3915", 4095),
        mk(4, feetech_servo::HLS, "HLS2915", 4078),
    };
    t.setServos(v);
    t.setPresentPosition(1, 3083); t.setPresentPosition(2, 4095); t.setPresentPosition(4, 4078);
    t.reflectTorque(1, true);   // servo 1 has torque actually ON
    t.show(); QApplication::processEvents();
    t.grab().save("joints.png"); printf("wrote joints.png\n");
    return 0;
}
