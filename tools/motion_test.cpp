// Guarded first-motion test for an HLS servo.
//
// Sequence:
//   1. read and print pre-flight state
//   2. test the e-stop analogue (torque disable) and confirm it takes effect
//   3. re-enable torque and command ONE small bounded move at reduced torque/speed
//   4. log position throughout
//   5. leave the servo in a safe state (torque disabled)
//
// Motion is bounded in code because HLS servos with min==max==0 angle limits are
// in multi-turn mode and have no travel stops of their own.

#include <QCoreApplication>
#include <QSerialPort>
#include <QTextStream>
#include <QThread>
#include <QElapsedTimer>
#include "servo/scserial.h"

static const int DELTA_COUNTS = -200;  // ~17.6 deg at 4096 counts/rev
static const int TORQUE       = 150;   // x6.5mA ~= 975mA
static const int SPEED        = 30;
static const int ACC          = 20;
static const int SETTLE_MS    = 2000;

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream out(stdout);

    if(argc < 4)
    {
        out << "usage: motion_test <port> <baud> <id>\n";
        return 2;
    }

    QSerialPort serial;
    serial.setPortName(argv[1]);
    serial.setBaudRate(QString(argv[2]).toInt());
    if(!serial.open(QIODevice::ReadWrite))
    {
        out << "failed to open " << argv[1] << "\n";
        return 1;
    }

    feetech_servo::SCSerial scs(&serial);
    scs.set_timeout(30);
    feetech_servo::HLSCL hls(&scs);
    const int id = QString(argv[3]).toInt();

    // --- 1. pre-flight ---------------------------------------------------
    int start_pos = scs.read_word(id, 56);
    out << "== pre-flight ==\n"
        << "  present_pos   = " << start_pos << "\n"
        << "  work_mode     = " << scs.read_byte(id, 33) << "\n"
        << "  torque_enable = " << scs.read_byte(id, 40) << "\n"
        << "  goal_torque   = " << scs.read_word(id, 44) << "\n";
    out.flush();

    if(start_pos < 0)
    {
        out << "ABORT: cannot read position\n";
        return 1;
    }

    // --- 2. e-stop analogue: torque disable must work BEFORE we move ------
    out << "\n== e-stop test (torque disable) ==\n";
    hls.enable_torque(id, 0);
    QThread::msleep(100);
    int te = scs.read_byte(id, 40);
    out << "  torque_enable after disable = " << te
        << (te == 0 ? "  OK\n" : "  FAILED\n");
    out.flush();
    if(te != 0)
    {
        out << "ABORT: torque disable did not take effect. No motion commanded.\n";
        return 1;
    }

    // --- 3. bounded move -------------------------------------------------
    const int target = start_pos + DELTA_COUNTS;
    out << "\n== motion ==\n"
        << "  " << start_pos << " -> " << target
        << "  (delta " << DELTA_COUNTS << ", torque " << TORQUE
        << ", speed " << SPEED << ", acc " << ACC << ")\n";
    out.flush();

    hls.enable_torque(id, 1);
    QThread::msleep(50);
    hls.servo_mode(id);
    QThread::msleep(50);

    int rc = hls.write_pos_ex(id, target, SPEED, ACC, TORQUE);
    out << "  write_pos_ex returned " << rc << "\n";
    out << "  goal_torque now reads " << scs.read_word(id, 44) << "\n";
    out.flush();

    // --- 4. log the response ---------------------------------------------
    QElapsedTimer t;
    t.start();
    int last = start_pos;
    while(t.elapsed() < SETTLE_MS)
    {
        int p = scs.read_word(id, 56);
        if(p >= 0)
        {
            out << "  t=" << t.elapsed() << "ms pos=" << p
                << " (moved " << (p - start_pos) << ")\n";
            out.flush();
            last = p;
        }
        QThread::msleep(200);
    }

    // --- 5. safe state ----------------------------------------------------
    hls.enable_torque(id, 0);
    out << "\n== result ==\n"
        << "  start  = " << start_pos << "\n"
        << "  target = " << target << "\n"
        << "  final  = " << last << "\n"
        << "  moved  = " << (last - start_pos) << " counts\n"
        << "  error  = " << (last - target) << " counts\n"
        << "  torque disabled, servo left in safe state\n";

    serial.close();
    return 0;
}
