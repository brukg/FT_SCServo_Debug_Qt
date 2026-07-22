// Does HLS honour speed=0 the way SMS/STS does (max speed), or as literal zero?
#include <QCoreApplication>
#include <QSerialPort>
#include <QTextStream>
#include <QThread>
#include "servo/scserial.h"
int main(int argc, char *argv[]){
    QCoreApplication app(argc,argv); QTextStream out(stdout);
    QSerialPort s; s.setPortName(argv[1]); s.setBaudRate(QString(argv[2]).toInt());
    if(!s.open(QIODevice::ReadWrite)){ out<<"open failed\n"; return 1; }
    feetech_servo::SCSerial scs(&s); scs.set_timeout(30);
    feetech_servo::HLSCL hls(&scs);
    int id = QString(argv[3]).toInt();

    for(int speed : {0, 30})
    {
        hls.enable_torque(id,1); QThread::msleep(50);
        hls.servo_mode(id);      QThread::msleep(50);
        int start = hls.read_pos(id);
        int target = start - 150;
        out << "\n--- speed=" << speed << "  " << start << " -> " << target << " ---\n";
        hls.write_pos_ex(id, target, speed, 20, 150);
        for(int i=0;i<8;i++){ QThread::msleep(250);
            out << "  t=" << (i+1)*250 << "ms pos=" << hls.read_pos(id) << "\n"; out.flush(); }
        int fin = hls.read_pos(id);
        out << "  RESULT moved " << (fin-start) << " counts, error " << (fin-target) << "\n";
    }
    hls.enable_torque(id,0);
    out << "\ntorque disabled\n";
    s.close(); return 0;
}
