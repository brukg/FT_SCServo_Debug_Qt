#include <QtTest>
#include <vector>
#include "servo/scserial.h"

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
};

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

QTEST_MAIN(TestPackets)
#include "test_packets.moc"
