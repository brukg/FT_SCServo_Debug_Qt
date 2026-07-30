// Read-then-set a stiffness-candidate register so we can FEEL which one makes an
// HLS servo compliant. Writes the position P gain (reg 21, EPROM) and/or the SRAM
// Kp (reg 50), with an EPROM unlock. Then you push the joint by hand.
//   usage: compliance_probe <port> <baud> <id> <reg> <value>
//   e.g.:  compliance_probe /dev/ttyACM0 1000000 1 21 2     # very soft P gain
//          compliance_probe /dev/ttyACM0 1000000 1 21 32    # restore default
#include <QCoreApplication>
#include <QSerialPort>
#include <QTextStream>
#include <QThread>
#include "servo/scserial.h"
int main(int argc, char *argv[]){
    QCoreApplication app(argc, argv); QTextStream out(stdout);
    if(argc < 6){ out << "usage: compliance_probe <port> <baud> <id> <reg> <value>\n"; return 2; }
    QSerialPort s; s.setPortName(argv[1]); s.setBaudRate(QString(argv[2]).toInt());
    if(!s.open(QIODevice::ReadWrite)){ out << "open failed\n"; return 1; }
    feetech_servo::SCSerial scs(&s); scs.set_timeout(30); scs.set_end(0);
    int id = QString(argv[3]).toInt(), reg = QString(argv[4]).toInt(), val = QString(argv[5]).toInt();

    out << "before: reg " << reg << " = " << scs.read_byte(id, reg)
        << "  (torque_en=" << scs.read_byte(id,40) << ", mode=" << scs.read_byte(id,33) << ")\n";
    scs.write_byte(id, 55, 0);            // unlock EPROM (HLS lock = reg 55)
    QThread::msleep(20);
    scs.write_byte(id, reg, val);          // set the candidate stiffness register
    QThread::msleep(20);
    scs.write_byte(id, 55, 1);            // re-lock
    out << "after:  reg " << reg << " = " << scs.read_byte(id, reg) << "\n";
    out << "Now push the joint by hand and see if it is softer.\n";
    s.close(); return 0;
}
