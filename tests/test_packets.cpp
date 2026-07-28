#include <QtTest>
#include <QDir>
#include <QFile>
#include <vector>
#include "servo/scserial.h"
#include "servo/servo_dispatch.h"
#include "csvrecorder.h"

// Test double: captures every byte the servo layer would put on the wire.
// Passing nullptr for the QSerialPort is safe because all IO is overridden.
// level_ = 0 makes SCSerial::ask() return without attempting a read.
class CapturingSerial : public feetech_servo::SCSerial
{
public:
    CapturingSerial() : SCSerial(nullptr) { level_ = 0; }

    std::vector<uint8_t> tx;

    int write(uint8_t *n_dat, int n_len) override
    {
        tx.insert(tx.end(), n_dat, n_dat + n_len);
        return n_len;
    }

    int write(uint8_t b_dat) override
    {
        tx.push_back(b_dat);
        return 1;
    }

    int read(uint8_t *, int) override { return 0; }
};

// Extract the payload of a WRITE packet: ff ff id len inst addr <payload...> checksum
static std::vector<uint8_t> payload_of(const std::vector<uint8_t> &tx)
{
    if (tx.size() < 7) return {};
    return std::vector<uint8_t>(tx.begin() + 6, tx.end() - 1);
}

class TestPackets : public QObject
{
    Q_OBJECT

private slots:
    void sms_sts_write_pos_ex_emits_zero_at_44_45();
    void sms_sts_packet_framing_is_wellformed();

    void resolver_identifies_hls();
    void resolver_distinguishes_hts_from_hls_same_major();
    void resolver_uses_endianness_to_break_firmware_overlap();
    void resolver_fails_closed_on_unknown_model();
    void resolver_controls_unknown_model_with_known_firmware();

    void hls_write_pos_ex_puts_torque_at_44_45();
    void hls_write_pos_ex_encodes_negative_position_as_sign_bit();
    void hls_zero_speed_is_replaced_with_a_moving_default();

    void dispatch_write_goal_routes_by_series();
    void dispatch_enable_torque_routes_by_series();
    void sync_group_all_hls_emits_one_packet_with_torque();
    void sync_group_mixed_series_emits_one_packet_per_series();

    void csv_recorder_writes_header_and_rows();
};

// Build a known ServoProfile for a given series.
static feetech_servo::ServoProfile prof(feetech_servo::ModelSeries s, uint8_t end = 0)
{
    feetech_servo::ServoProfile p;
    p.name = "test"; p.series = s; p.end = end; p.known = true;
    return p;
}

// Count SYNC WRITE packets (instruction byte 0x83 at offset 4 of each frame).
static int count_sync_packets(const std::vector<uint8_t> &tx)
{
    int n = 0;
    for (size_t i = 0; i + 4 < tx.size(); i++)
        if (tx[i] == 0xff && tx[i+1] == 0xff && tx[i+4] == 0x83)
            n++;
    return n;
}

// Model and firmware numbers are packed (minor << 8 | major).
static uint16_t mk(uint8_t major, uint8_t minor) { return (uint16_t)((minor << 8) | major); }

// Characterization: documents CURRENT behaviour. On SMS/STS addresses 44/45 are
// Goal Time / PWM open-loop speed, and zero there is correct and harmless.
void TestPackets::sms_sts_write_pos_ex_emits_zero_at_44_45()
{
    CapturingSerial serial;
    feetech_servo::SMS_STS sms(&serial);

    sms.write_pos_ex(1, 2048, 300, 50);

    auto p = payload_of(serial.tx);
    QCOMPARE(p.size(), size_t(7));
    QCOMPARE(p[0], uint8_t(50));            // ACC
    QCOMPARE(p[1], uint8_t(2048 & 0xff));   // Position L
    QCOMPARE(p[2], uint8_t(2048 >> 8));     // Position H
    QCOMPARE(p[3], uint8_t(0));             // addr 44
    QCOMPARE(p[4], uint8_t(0));             // addr 45
    QCOMPARE(p[5], uint8_t(300 & 0xff));    // Speed L
    QCOMPARE(p[6], uint8_t(300 >> 8));      // Speed H
}

void TestPackets::sms_sts_packet_framing_is_wellformed()
{
    CapturingSerial serial;
    feetech_servo::SMS_STS sms(&serial);

    sms.write_pos_ex(1, 2048, 300, 50);

    QCOMPARE(serial.tx[0], uint8_t(0xff));
    QCOMPARE(serial.tx[1], uint8_t(0xff));
    QCOMPARE(serial.tx[2], uint8_t(1));     // id
    QCOMPARE(serial.tx[3], uint8_t(10));    // length = 7 payload + addr + inst + 1
    QCOMPARE(serial.tx[4], uint8_t(0x03));  // INST_WRITE
    QCOMPARE(serial.tx[5], uint8_t(41));    // start address

    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < serial.tx.size(); i++)
        sum += serial.tx[i];
    QCOMPARE(serial.tx.back(), uint8_t(~sum));
}

void TestPackets::resolver_identifies_hls()
{
    auto p = feetech_servo::resolveServo(mk(10, 13), mk(3, 42));
    QCOMPARE(p.name, QString("HLS3625"));
    QCOMPARE(p.series, feetech_servo::HLS);
    QCOMPARE(p.end, uint8_t(0));
    QVERIFY(p.known);
}

// Model major 10 spans two register maps. Firmware, not name, decides.
void TestPackets::resolver_distinguishes_hts_from_hls_same_major()
{
    auto hts = feetech_servo::resolveServo(mk(10, 1), mk(3, 10));
    QCOMPARE(hts.name, QString("HTS3235"));
    QCOMPARE(hts.series, feetech_servo::STS);

    auto hls = feetech_servo::resolveServo(mk(10, 13), mk(3, 42));
    QCOMPARE(hls.series, feetech_servo::HLS);
}

// Firmware 3.20-3.39 matches both STS (flag 0) and SCSXX-2 (flag 1).
void TestPackets::resolver_uses_endianness_to_break_firmware_overlap()
{
    auto scs2 = feetech_servo::resolveServo(mk(9, 15), mk(3, 25));
    QCOMPARE(scs2.name, QString("SCS15-2"));
    QCOMPARE(scs2.series, feetech_servo::SCS2);
    QCOMPARE(scs2.end, uint8_t(1));

    auto sts = feetech_servo::resolveServo(mk(9, 3), mk(3, 25));
    QCOMPARE(sts.series, feetech_servo::STS);
    QCOMPARE(sts.end, uint8_t(0));
}

void TestPackets::resolver_fails_closed_on_unknown_model()
{
    auto p = feetech_servo::resolveServo(mk(99, 99), mk(99, 99));
    QCOMPARE(p.name, QString("Unknown"));
    QCOMPARE(p.series, feetech_servo::UNKNOWN);
    QVERIFY(!p.known);
}

// A servo newer than the model table is still safely controllable, because the
// register map comes from firmware.
void TestPackets::resolver_controls_unknown_model_with_known_firmware()
{
    auto p = feetech_servo::resolveServo(mk(10, 99), mk(3, 42));
    QCOMPARE(p.series, feetech_servo::HLS);
    QVERIFY(p.known);
    QVERIFY(p.name.startsWith("Unknown"));
}

// THE regression lock. The original bug: the app sent 0 to addresses 44/45,
// which on HLS is Goal Torque (max torque current, 6.5mA units). A zero current
// limit means the servo cannot move. It must carry the requested torque.
void TestPackets::hls_write_pos_ex_puts_torque_at_44_45()
{
    CapturingSerial serial;
    feetech_servo::HLSCL hls(&serial);

    hls.write_pos_ex(1, 4095, 60, 50, 500);

    auto p = payload_of(serial.tx);
    QCOMPARE(p.size(), size_t(7));
    QCOMPARE(p[0], uint8_t(50));            // ACC           @41
    QCOMPARE(p[1], uint8_t(4095 & 0xff));   // Position L    @42
    QCOMPARE(p[2], uint8_t(4095 >> 8));     // Position H    @43
    QCOMPARE(p[3], uint8_t(500 & 0xff));    // Goal Torque L @44
    QCOMPARE(p[4], uint8_t(500 >> 8));      // Goal Torque H @45
    QCOMPARE(p[5], uint8_t(60));            // Speed L       @46
    QCOMPARE(p[6], uint8_t(0));             // Speed H       @47

    QVERIFY2(!(p[3] == 0 && p[4] == 0),
             "Goal Torque must never be zero-filled: a 0mA current limit "
             "immobilises the servo. This was the original defect.");
}

void TestPackets::hls_write_pos_ex_encodes_negative_position_as_sign_bit()
{
    CapturingSerial serial;
    feetech_servo::HLSCL hls(&serial);

    hls.write_pos_ex(1, -1000, 60, 0, 500);

    auto p = payload_of(serial.tx);
    const uint16_t expected = 1000 | (1 << 15);
    QCOMPARE(p[1], uint8_t(expected & 0xff));
    QCOMPARE(p[2], uint8_t(expected >> 8));
}

// Sweep, step and the goal slider all command speed 0, which on SMS/STS means
// "no speed limit". HLS takes it literally: measured on an HLS3955, speed 0
// produced 0 counts of motion over 2s, while speed 30 reached target exactly.
// So a zero must be replaced with a usable default rather than passed through.
void TestPackets::hls_zero_speed_is_replaced_with_a_moving_default()
{
    CapturingSerial serial;
    feetech_servo::HLSCL hls(&serial);

    hls.write_pos_ex(1, 2000, 0, 0, 500);

    auto p = payload_of(serial.tx);
    const uint16_t speed = uint16_t(p[5]) | (uint16_t(p[6]) << 8);
    QVERIFY2(speed != 0,
             "HLS Goal Velocity 0 means no motion. Sweep/step/slider pass 0, so "
             "the control class must substitute a non-zero default.");
    QCOMPARE(speed, uint16_t(feetech_servo::HLSCL::kDefaultSpeed));
}

// write_goal_for must call the class matching the servo's series.
void TestPackets::dispatch_write_goal_routes_by_series()
{
    using namespace feetech_servo;
    CapturingSerial serial;
    SCSCL scs(&serial); SMS_STS sms(&serial); HLSCL hls(&serial);

    // HLS -> torque carried at bytes 44/45 (7-byte payload at addr 41)
    serial.tx.clear();
    write_goal_for(&scs, &sms, &hls, 1, prof(HLS), 4095, 60, 50, 500);
    {
        // last WRITE frame is the position write (servo_mode writes a byte first)
        // find the 7-byte-payload frame at addr 41
        bool found = false;
        for (size_t i = 0; i + 6 < serial.tx.size(); i++)
            if (serial.tx[i] == 0xff && serial.tx[i+1] == 0xff &&
                serial.tx[i+3] == 10 && serial.tx[i+4] == 0x03 && serial.tx[i+5] == 41)
            {
                QCOMPARE(serial.tx[i+6+3], uint8_t(500 & 0xff)); // torque L @44
                QCOMPARE(serial.tx[i+6+4], uint8_t(500 >> 8));   // torque H @45
                found = true;
            }
        QVERIFY2(found, "HLS position write frame at addr 41 not found");
    }

    // SCS -> write at addr 42, range semantics differ; just confirm a frame at 42
    serial.tx.clear();
    write_goal_for(&scs, &sms, &hls, 2, prof(SCS, 1), 512, 60, 0, 0);
    {
        bool at42 = false;
        for (size_t i = 0; i + 5 < serial.tx.size(); i++)
            if (serial.tx[i] == 0xff && serial.tx[i+1] == 0xff &&
                serial.tx[i+4] == 0x03 && serial.tx[i+5] == 42)
                at42 = true;
        QVERIFY2(at42, "SCS position write should target addr 42");
    }
}

void TestPackets::dispatch_enable_torque_routes_by_series()
{
    using namespace feetech_servo;
    CapturingSerial serial;
    SCSCL scs(&serial); SMS_STS sms(&serial); HLSCL hls(&serial);

    serial.tx.clear();
    enable_torque_for(&scs, &sms, &hls, 5, prof(HLS), true);
    // torque enable is a 1-byte write at addr 40
    bool at40 = false;
    for (size_t i = 0; i + 6 < serial.tx.size(); i++)
        if (serial.tx[i] == 0xff && serial.tx[i+1] == 0xff &&
            serial.tx[i+2] == 5 && serial.tx[i+4] == 0x03 && serial.tx[i+5] == 40)
        {
            QCOMPARE(serial.tx[i+6], uint8_t(1)); // enable = 1
            at40 = true;
        }
    QVERIFY2(at40, "torque enable should be a write to addr 40");

    // fail-closed servo: no bytes emitted
    serial.tx.clear();
    ServoProfile unknown; unknown.known = false; unknown.series = UNKNOWN;
    enable_torque_for(&scs, &sms, &hls, 9, unknown, true);
    QVERIFY2(serial.tx.empty(), "unknown servo must not be written to");
}

// All-HLS group -> exactly one sync packet, torque present.
void TestPackets::sync_group_all_hls_emits_one_packet_with_torque()
{
    using namespace feetech_servo;
    CapturingSerial serial;
    SCSCL scs(&serial); SMS_STS sms(&serial); HLSCL hls(&serial);

    std::vector<GroupTarget> g = {
        {1, prof(HLS), 1000},
        {2, prof(HLS), 2000},
        {3, prof(HLS), 3000},
    };
    sync_write_group(&scs, &sms, &hls, g, 60, 50, 500);

    QCOMPARE(count_sync_packets(serial.tx), 1);
    // torque bytes (500) must appear in the sync payload; a zero-torque bug would not
    bool torque_seen = false;
    for (size_t i = 0; i + 1 < serial.tx.size(); i++)
        if (serial.tx[i] == uint8_t(500 & 0xff) && serial.tx[i+1] == uint8_t(500 >> 8))
            torque_seen = true;
    QVERIFY2(torque_seen, "HLS sync write must carry the group torque");
}

// Mixed HLS + STS -> one packet per series (two total).
void TestPackets::sync_group_mixed_series_emits_one_packet_per_series()
{
    using namespace feetech_servo;
    CapturingSerial serial;
    SCSCL scs(&serial); SMS_STS sms(&serial); HLSCL hls(&serial);

    std::vector<GroupTarget> g = {
        {1, prof(HLS), 1000},
        {2, prof(STS), 2000},
        {3, prof(HLS), 3000},
        {4, prof(STS), 1500},
    };
    sync_write_group(&scs, &sms, &hls, g, 60, 50, 500);

    // one HLS packet + one STS packet
    QCOMPARE(count_sync_packets(serial.tx), 2);
}

void TestPackets::csv_recorder_writes_header_and_rows()
{
    const QString path = QDir::tempPath() + "/ft_csv_test.csv";
    {
        CsvRecorder rec;
        QVERIFY(rec.start(path, "position"));
        QVERIFY(rec.isRecording());
        rec.write(0.100, 1, "HLS3955", 3083);
        rec.write(0.200, 2, "HLS3915", 4095);
        rec.stop();
        QVERIFY(!rec.isRecording());
    }
    QFile f(path);
    QVERIFY(f.open(QIODevice::ReadOnly | QIODevice::Text));
    const QString content = QString::fromUtf8(f.readAll());
    f.close();

    const QStringList lines = content.split('\n', Qt::SkipEmptyParts);
    QCOMPARE(lines.size(), 3);
    QCOMPARE(lines[0], QString("time_s,id,name,signal,value"));
    QCOMPARE(lines[1], QString("0.100,1,HLS3955,position,3083"));
    QCOMPARE(lines[2], QString("0.200,2,HLS3915,position,4095"));
    QFile::remove(path);
}

QTEST_MAIN(TestPackets)
#include "test_packets.moc"
