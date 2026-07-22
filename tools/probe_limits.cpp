// Read-only: dump the safety-relevant registers for one servo.
#include <QCoreApplication>
#include <QSerialPort>
#include <QTextStream>
#include "servo/scserial.h"
int main(int argc, char *argv[]){
    QCoreApplication app(argc, argv); QTextStream out(stdout);
    QSerialPort s; s.setPortName(argv[1]); s.setBaudRate(QString(argv[2]).toInt());
    if(!s.open(QIODevice::ReadWrite)){ out<<"open failed\n"; return 1; }
    feetech_servo::SCSerial scs(&s); scs.set_timeout(30);
    int id = QString(argv[3]).toInt();
    out << "ID " << id << "\n"
        << "  min_angle_limit(9)  = " << scs.read_word(id, 9)  << "\n"
        << "  max_angle_limit(11) = " << scs.read_word(id, 11) << "\n"
        << "  max_temp(13)        = " << scs.read_byte(id, 13) << "\n"
        << "  max_torque_limit(16)= " << scs.read_word(id, 16) << "\n"
        << "  overload_current(28)= " << scs.read_word(id, 28) << "\n"
        << "  work_mode(33)       = " << scs.read_byte(id, 33) << "\n"
        << "  torque_enable(40)   = " << scs.read_byte(id, 40) << "\n"
        << "  goal_torque(44)     = " << scs.read_word(id, 44) << "\n"
        << "  torque_limit(48)    = " << scs.read_word(id, 48) << "\n"
        << "  present_pos(56)     = " << scs.read_word(id, 56) << "\n"
        << "  present_temp(63)    = " << scs.read_byte(id, 63) << "\n";
    s.close(); return 0;
}
