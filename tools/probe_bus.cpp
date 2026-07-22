// Read-only bus probe: pings every ID, reads model number + firmware version,
// and reports what resolveServo() makes of each. Sends no write of any kind.
//
// Build:  qmake tools/probe.pro && make
// Run:    ./probe_bus /dev/ttyACM0 1000000

#include <QCoreApplication>
#include <QSerialPort>
#include <QTextStream>
#include "servo/scserial.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream out(stdout);

    if(argc < 3)
    {
        out << "usage: probe_bus <port> <baud>\n";
        return 2;
    }

    QSerialPort serial;
    serial.setPortName(argv[1]);
    serial.setBaudRate(QString(argv[2]).toInt());
    serial.setParity(QSerialPort::NoParity);
    serial.setDataBits(QSerialPort::Data8);
    serial.setStopBits(QSerialPort::OneStop);

    if(!serial.open(QIODevice::ReadWrite))
    {
        out << "failed to open " << argv[1] << ": " << serial.errorString() << "\n";
        return 1;
    }

    feetech_servo::SCSerial scs(&serial);
    scs.set_timeout(30);

    out << "scanning " << argv[1] << " @ " << argv[2] << " baud (read-only)\n";
    out.flush();

    int found = 0;
    for(int id = 0; id <= 0xfd; id++)
    {
        int ret = scs.ping(id);
        if(ret <= 0)
            continue;

        int mid = scs.read_model_number(ret);
        int fw  = scs.read_firmware_version(ret);
        auto p  = feetech_servo::resolveServo(mid, fw);

        static const char *series_name[] =
            { "SMCL", "SMBL", "STS", "SCS", "HLS", "SCS2", "UNKNOWN" };

        out << "\nID " << ret << "\n"
            << "  model    = 0x" << QString::number(mid, 16).rightJustified(4, '0')
            << "  (major " << (mid & 0xff) << ", minor " << ((mid >> 8) & 0xff) << ")\n"
            << "  firmware = 0x" << QString::number(fw, 16).rightJustified(4, '0')
            << "  (major " << (fw & 0xff) << ", minor " << ((fw >> 8) & 0xff) << ")\n"
            << "  name     = " << p.name << "\n"
            << "  series   = " << series_name[p.series] << "\n"
            << "  endian   = " << p.end << "\n"
            << "  known    = " << (p.known ? "yes" : "no  <-- control disabled") << "\n";

        // Read-only sanity of the live SRAM block.
        out << "  pos=" << scs.read_word(ret, 56)
            << " volt=" << scs.read_byte(ret, 62)
            << " temp=" << scs.read_byte(ret, 63)
            << " mode=" << scs.read_byte(ret, 33)
            << " torque_en=" << scs.read_byte(ret, 40)
            << " goal_torque(44)=" << scs.read_word(ret, 44)
            << "\n";
        out.flush();
        found++;
    }

    out << "\n" << found << " servo(s) found\n";
    serial.close();
    return found > 0 ? 0 : 1;
}
