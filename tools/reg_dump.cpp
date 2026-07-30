// READ-ONLY: dump the stiffness/compliance-relevant registers for each servo.
// Sends no writes and no motion commands -- safe with servos mounted.
#include <QCoreApplication>
#include <QSerialPort>
#include <QTextStream>
#include "servo/scserial.h"
int main(int argc, char *argv[]){
    QCoreApplication app(argc, argv); QTextStream out(stdout);
    QSerialPort s; s.setPortName(argv[1]); s.setBaudRate(QString(argv[2]).toInt());
    if(!s.open(QIODevice::ReadWrite)){ out << "open failed\n"; return 1; }
    feetech_servo::SCSerial scs(&s); scs.set_timeout(30); scs.set_end(0);
    for(int id=0; id<=0xfd; id++){
        if(scs.ping(id) <= 0) continue;
        out << "\nID " << id << "\n"
            << "  mode(33)=" << scs.read_byte(id,33)
            << "  torque_en(40)=" << scs.read_byte(id,40)
            << "  present_pos(56)=" << scs.read_word(id,56) << "\n"
            << "  Pos P gain(21)=" << scs.read_byte(id,21)
            << "  Pos D gain(22)=" << scs.read_byte(id,22)
            << "  Pos I gain(23)=" << scs.read_byte(id,23) << "\n"
            << "  SRAM Kp(50)=" << scs.read_byte(id,50)
            << "  SRAM Kd(51)=" << scs.read_byte(id,51)
            << "  SRAM Ki(52)=" << scs.read_byte(id,52) << "\n"
            << "  Torque Limit(48)=" << scs.read_word(id,48)
            << "  Goal Current(44)=" << scs.read_word(id,44)
            << "  Cur P(34)=" << scs.read_byte(id,34)
            << "  Vel P(37)=" << scs.read_byte(id,37) << "\n";
        out.flush();
    }
    s.close(); return 0;
}
