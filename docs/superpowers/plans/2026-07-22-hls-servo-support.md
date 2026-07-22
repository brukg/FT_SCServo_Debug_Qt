# HLS Servo Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make HLS-series FeeTech servos identifiable and controllable from the Qt Linux debug tool, matching FD 1.9.8.5 behaviour.

**Architecture:** Replace display-name-based register-map inference with FD's two-key resolver (model number → name + endianness; firmware version + endianness → register map). Generate all six register tables from FD's `setup.log` via a checked-in script rather than hand transcription. Port FeeTech's `HLSCL` control class into the Qt app's header-only `servo/` style. Collapse nine duplicated two-way control branches into one series-aware helper.

**Tech Stack:** C++17, Qt 5.15 (qmake), QtTest, Python 3 (table generator only)

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-22-hls-servo-support-design.md`
- Source of truth for all register data: `../FD1.9.8.5(250729)/FD1985-250729/ft_setup_bat/setup.log`, encoding **GB18030**
- **CN (`setup.log`) is authoritative** for structure, size, direction bits, ranges, defaults. **EN (`setup_en.log`) supplies display names only.** Where an EN name contradicts CN semantics, CN wins and the English name is written by hand.
- Build: `qmake && make` using Qt 5.15 (`/usr/bin/qmake`). Baseline is confirmed to build clean on this machine.
- Unknown servos **fail closed** — never fall back to a guessed register map.
- Existing code style: header-only servo classes in `servo/`, `snake_case` methods, `feetech_servo` namespace, 4-space indent, opening brace on its own line.
- Do not commit the spec or this plan in task commits. Stage by explicit path only — never `git add -A` or `git add .`.

---

## File Structure

| File | Responsibility |
|---|---|
| `tools/gen_servo_tables.py` | **New.** Parses FD's `setup.log`/`setup_en.log`, emits the C++ register tables and model table. Provenance + regeneration for future FD releases. |
| `servo/servo_types.h` | Register tables (6 after this change). Generated content. |
| `servo/scserial.h` | `ModelSeries` enum, `ServoProfile`, resolver declarations, `SCSerial` with virtual write seam. |
| `servo/scserial.cpp` | Model table, firmware profile table, resolver implementation, `read_firmware_version`. |
| `servo/hlscl.h` | **New.** HLS control class, header-only, ported from `FTServo_Linux/src/HLSCL.cpp`. |
| `mainwindow.h` / `mainwindow.cpp` | `writeGoal` helper, fail-closed control gating, `timeLineEdit` relabelling. |
| `tests/tests.pro`, `tests/test_packets.cpp` | **New.** QtTest binary. Packet-level regression lock on the addr 44/45 defect. |
| `FT_SCServo_Debug_Qt.pro` | Add `servo/hlscl.h` to `HEADERS`. |

---

### Task 1: Test harness and characterization of current packet behaviour

Locks in what the wire bytes are *today* before anything changes, so Task 4 and Task 5 can prove they didn't break SCS/STS. Requires one production change: a virtual write seam.

**Files:**
- Modify: `servo/scserial.h:50-52` (make `write` overloads virtual, add virtual destructor)
- Create: `tests/tests.pro`
- Create: `tests/test_packets.cpp`

**Interfaces:**
- Consumes: nothing
- Produces: `CapturingSerial` test double (subclass of `feetech_servo::SCSerial` capturing all TX bytes in `std::vector<uint8_t> tx`), used by Tasks 4 and 5.

- [ ] **Step 1: Add the virtual write seam**

In `servo/scserial.h`, change the three IO declarations and add a virtual destructor. `SCSerial` currently has no virtual destructor; adding one is required now that it is subclassed.

```cpp
    SCSerial(QSerialPort *serial);
    virtual ~SCSerial() = default;
```

and:

```cpp
	virtual int write(uint8_t *n_dat, int n_len);
	virtual int read(uint8_t *n_dat, int n_len);
	virtual int write(uint8_t b_dat);
```

Note `read` becomes virtual too — `ask()` calls it, and Task 4's HLS tests need to stub a servo ACK.

- [ ] **Step 2: Create the test project file**

`tests/tests.pro`:

```
QT       += core serialport testlib
QT       -= gui

CONFIG   += c++17 console testcase
CONFIG   -= app_bundle

TARGET = test_packets

INCLUDEPATH += ..

SOURCES += \
    test_packets.cpp \
    ../servo/scserial.cpp

HEADERS += \
    ../servo/scserial.h \
    ../servo/servo_types.h \
    ../servo/sms_sts.h \
    ../servo/scscl.h
```

- [ ] **Step 3: Write the characterization test**

`tests/test_packets.cpp`. `CapturingSerial` passes `nullptr` for the `QSerialPort*` — safe because every IO path is overridden. `level_ = 0` makes `ask()` return without reading.

```cpp
#include <QtTest>
#include <vector>
#include "servo/scserial.h"

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
```

- [ ] **Step 4: Build and run the tests**

```bash
mkdir -p /tmp/hls-build/tests && cd /tmp/hls-build/tests
qmake /home/phoenix/signalbotics/feetech/FT_SCServo_Debug_Qt/tests/tests.pro && make -j$(nproc)
./test_packets
```

Expected: `Totals: 2 passed, 0 failed`. These pass immediately — they document existing behaviour, they do not drive new code.

- [ ] **Step 5: Verify the main app still builds**

```bash
mkdir -p /tmp/hls-build/app && cd /tmp/hls-build/app
qmake /home/phoenix/signalbotics/feetech/FT_SCServo_Debug_Qt/FT_SCServo_Debug_Qt.pro && make -j$(nproc)
```

Expected: links `FT_SCServo_Debug_Qt` with no errors. Warnings about `layoutWidget` from the `.ui` file are pre-existing and expected.

- [ ] **Step 6: Commit**

```bash
git add servo/scserial.h tests/tests.pro tests/test_packets.cpp
git commit -m "test: add packet-level test harness and characterize current write path"
```

---

### Task 2: Register table generator

Six tables of ~50 rows plus a 77-entry model table is 400+ hand-transcribed values across two disagreeing source files. A generator removes transcription error entirely and documents provenance.

**Files:**
- Create: `tools/gen_servo_tables.py`
- Modify: `servo/servo_types.h` (fully regenerated below the header guard)
- Modify: `servo/scserial.cpp:7-77` (model table regenerated)

**Interfaces:**
- Consumes: nothing
- Produces: `feetech_servo::HLSMemConfig`, `feetech_servo::SCS2MemConfig` (both `const std::vector<MemoryConfig>`), plus regenerated `SMCLMemConfig`/`SMBLMemConfig`/`STSMemConfig`/`SCSMemConfig`. `MemoryConfig` struct shape is unchanged.

- [ ] **Step 1: Write the generator**

`tools/gen_servo_tables.py`. Reads both config files, applies the CN-authoritative policy, emits C++.

```python
#!/usr/bin/env python3
"""Generate FT_SCServo_Debug_Qt register tables from FD's setup.log.

CN (setup.log) is authoritative for structure/sizes/dir bits/ranges/defaults.
EN (setup_en.log) supplies display names only. Where EN's name contradicts CN
semantics, CN wins and the English name is overridden below.

Usage:
  tools/gen_servo_tables.py <path-to-ft_setup_bat-dir>
"""
import sys, pathlib

# Firmware key -> (C++ table identifier, ModelSeries enumerator)
TABLES = {
    '1,0,19,0':  ('SMCLMemConfig', 'SMCL'),
    '2,40,69,0': ('SMBLMemConfig', 'SMBL'),
    '3,0,39,0':  ('STSMemConfig',  'STS'),
    '0,0,39,1':  ('SCSMemConfig',  'SCS'),
    '3,40,59,0': ('HLSMemConfig',  'HLS'),
    '3,20,39,1': ('SCS2MemConfig', 'SCS2'),
}

# EN name is wrong or missing; CN semantics win. Keyed (firmware_key, address).
NAME_OVERRIDES = {
    ('3,40,59,0', 44): 'Goal Torque',            # EN says "Goal PWM" (copy-paste from STS)
    ('0,0,39,1',  18): 'Phase',                  # absent from EN
    ('3,20,39,1', 34): 'Position Offset Value',  # absent from EN
    ('3,20,39,1', 44): 'Goal PWM',               # EN says "Running Time"
}

AREA_IS_EPROM = {'EPROM': 'true', 'SRAM': 'false', 'DEFAULT': 'false'}
RW_IS_READONLY = {'只读': 'true', '读写': 'false', '默认': 'true'}


def parse(path):
    """Return {firmware_key: {address: [name, size, default, dir, area, rw, min, max]}}."""
    lines = path.read_bytes().decode('gb18030', errors='replace').splitlines()
    out, i = {}, 0
    while i < len(lines):
        if lines[i].strip() == '[内存]':
            key = lines[i + 1].strip()
            rows, j = {}, i + 2
            while j < len(lines) and lines[j].strip() != '[内存结束]':
                parts = [p.strip() for p in lines[j].split(',')]
                if len(parts) >= 9 and parts[0].isdigit():
                    rows[int(parts[0])] = parts[1:9]
                j += 1
            out[key] = rows
            i = j
        i += 1
    return out


def parse_models(path):
    """Return sorted [(major, minor, endian_flag, name)] from the [型号] section."""
    lines = path.read_bytes().decode('gb18030', errors='replace').splitlines()
    out, inside = [], False
    for line in lines:
        s = line.strip()
        if s == '[型号]':
            inside = True
            continue
        if s == '[型号结束]':
            break
        if inside and s and s[0].isdigit():
            p = [x.strip() for x in s.split(',')]
            if len(p) >= 5:
                major, lo, hi, flag, name = int(p[0]), int(p[1]), int(p[2]), int(p[3]), p[4]
                for minor in range(lo, hi + 1):
                    out.append((major, minor, flag, name))
    # Later rows win on duplicates, matching FD's last-wins parse order.
    dedup = {}
    for major, minor, flag, name in out:
        dedup[(major, minor)] = (flag, name)
    return [(k[0], k[1], v[0], v[1]) for k, v in sorted(dedup.items())]


def emit_table(ident, rows_cn, rows_en, key):
    lines = [f'const std::vector<MemoryConfig> {ident} =', '{']
    for addr in sorted(rows_cn):
        cn = rows_cn[addr]
        # DEFAULT-area rows are FD-internal tuning values, not servo registers.
        if cn[3] == 'DEFAULT':
            continue
        name = NAME_OVERRIDES.get((key, addr))
        if name is None:
            en = rows_en.get(addr)
            if en is None:
                raise SystemExit(
                    f'ERROR: {ident} addr {addr} ({cn[0]}) has no English name '
                    f'and no override. Add it to NAME_OVERRIDES.')
            name = en[0]
        size, default, dirbit = cn[1], cn[2], cn[3 - 1]
        size, default, dirbit, area, rw, lo, hi = cn[1], cn[2], cn[3], cn[4], cn[5], cn[6], cn[7]
        default = default.strip() or '0'
        lines.append(
            f'    {{{addr}, "{name}", {size}, {default}, {dirbit}, '
            f'{AREA_IS_EPROM[area]}, {RW_IS_READONLY[rw]}, {lo}, {hi}}},')
    lines.append('};')
    return '\n'.join(lines)


def main():
    d = pathlib.Path(sys.argv[1])
    cn = parse(d / 'setup.log')
    en = parse(d / 'setup_en.log')

    print('// GENERATED by tools/gen_servo_tables.py from FD 1.9.8.5 setup.log.')
    print('// Do not edit by hand. See docs for the CN/EN reconciliation policy.')
    print()
    for key, (ident, _series) in TABLES.items():
        if key not in cn:
            raise SystemExit(f'ERROR: firmware key {key} missing from setup.log')
        print(emit_table(ident, cn[key], en.get(key, {}), key))
        print()

    print('// ---- model table (paste into servo/scserial.cpp) ----')
    for major, minor, flag, name in parse_models(d / 'setup.log'):
        print(f'        SERVO_MODEL({major}, {minor}, "{name}", {flag}),')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Run the generator and inspect output**

```bash
cd /home/phoenix/signalbotics/feetech/FT_SCServo_Debug_Qt
python3 tools/gen_servo_tables.py "../FD1.9.8.5(250729)/FD1985-250729/ft_setup_bat" > /tmp/hls-build/tables.txt
head -60 /tmp/hls-build/tables.txt
```

Expected: six `const std::vector<MemoryConfig>` blocks then the model list. If it exits with `ERROR: ... has no English name`, add the address to `NAME_OVERRIDES` with a name derived from the CN term — do not guess silently.

- [ ] **Step 3: Spot-check the three highest-risk values**

```bash
grep -A100 'HLSMemConfig' /tmp/hls-build/tables.txt | grep -E '^\s+\{(44|31|33|50),'
grep -A100 'SMCLMemConfig' /tmp/hls-build/tables.txt | grep -E '^\s+\{33,'
grep -c 'SERVO_MODEL' /tmp/hls-build/tables.txt
```

Expected, and each is a spec decision being verified:
- HLS addr 44 → `{44, "Goal Torque", 2, 0, 15, false, false, -2048, 2047},` — CN semantics, not EN's `Goal PWM`
- HLS addr 33 → max `4` (not STS's 3)
- HLS addr 50 → `Kp` present
- SMCL addr 33 → `dir_bit` of **11**, not 15 — this is the pre-existing sign-decode bug being corrected
- Model count → **77**

- [ ] **Step 4: Replace the tables in `servo/servo_types.h`**

Keep the header guard, `#include`s, `namespace feetech_servo`, and the `MemoryConfig` struct definition exactly as they are. Replace everything from `// SMXX` down to the closing `}` of the namespace with the six generated tables.

- [ ] **Step 5: Verify it compiles**

```bash
cd /tmp/hls-build/app && make -j$(nproc)
```

Expected: compiles clean. `SCS2MemConfig` and `HLSMemConfig` are unreferenced so far; that is fine, they are `const` at namespace scope.

- [ ] **Step 6: Commit**

```bash
git add tools/gen_servo_tables.py servo/servo_types.h
git commit -m "feat: generate register tables from FD setup.log, add HLS and SCS2

CN setup.log is authoritative for structure; EN supplies names.
Corrects SMCL position-offset dir_bit from 15 to 11 (11-bit signed range)."
```

---

### Task 3: Two-key servo profile resolver

Replaces `getModelSeries(QString)` name matching with FD's firmware-keyed lookup.

**Files:**
- Modify: `servo/scserial.h:21-31` (enum, `ServoProfile`, declarations)
- Modify: `servo/scserial.cpp:6-97` (model table, firmware table, resolver, `read_firmware_version`)
- Modify: `tests/tests.pro` (no change needed — already compiles `scserial.cpp`)
- Modify: `tests/test_packets.cpp` (add resolver tests)

**Interfaces:**
- Consumes: `HLSMemConfig`, `SCS2MemConfig` from Task 2
- Produces:
  - `enum ModelSeries { SMCL, SMBL, STS, SCS, HLS, SCS2, UNKNOWN }`
  - `struct ServoProfile { QString name; ModelSeries series; uint8_t end; bool known; }`
  - `ServoProfile resolveServo(uint16_t model_number, uint16_t firmware_version)`
  - `int SCSerial::read_firmware_version(int id)`

- [ ] **Step 1: Write the failing resolver tests**

Append to `tests/test_packets.cpp`, and add the slots to the `private slots:` block.

```cpp
    void resolver_identifies_hls();
    void resolver_distinguishes_hts_from_hls_same_major();
    void resolver_uses_endianness_to_break_firmware_overlap();
    void resolver_fails_closed_on_unknown_model();
    void resolver_controls_unknown_model_with_known_firmware();
```

```cpp
// Model numbers are packed (minor << 8 | major); firmware likewise.
static uint16_t mk(uint8_t major, uint8_t minor) { return (minor << 8) | major; }

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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /tmp/hls-build/tests && qmake /home/phoenix/signalbotics/feetech/FT_SCServo_Debug_Qt/tests/tests.pro && make -j$(nproc)
```

Expected: FAIL to compile — `resolveServo` is not declared, `feetech_servo::HLS` does not exist.

- [ ] **Step 3: Declare the types in `servo/scserial.h`**

Replace the existing `getModelType` / `ModelSeries` / `getModelSeries` block (lines 21-31):

```cpp
enum ModelSeries
{
	SMCL,
	SMBL,
	STS,
	SCS,
	HLS,
	SCS2,
	UNKNOWN
};

struct ServoProfile
{
    QString     name;            // display name, e.g. "HLS3625"
    ModelSeries series;          // which register map and control class to use
    uint8_t     end;             // endianness for host<->servo word packing
    bool        known;           // false => fail closed, disable control
};

// Resolves a servo the way FD 1.9.8.5 does: the model number (addresses 3,4)
// gives the display name and endianness; the firmware version (addresses 0,1)
// plus that endianness gives the register map. Model major 10 covers both HTS
// (STS map) and HLS (HLS map), so the name alone cannot decide.
ServoProfile resolveServo(uint16_t model_number, uint16_t firmware_version);
```

Add to the `SCSerial` public section, next to `read_model_number`:

```cpp
    int read_firmware_version(int id);
```

- [ ] **Step 4: Implement in `servo/scserial.cpp`**

Replace the `SERVO_MODEL` block and both old functions. The macro gains an endianness field.

```cpp
namespace {

struct ModelEntry
{
    QString name;
    uint8_t end;
};

struct FirmwareProfile
{
    uint8_t major;
    uint8_t minor_start;
    uint8_t minor_end;
    uint8_t end;
    feetech_servo::ModelSeries series;
};

// From setup.log [调试]. Firmware 3.20-3.39 appears twice, disambiguated by `end`.
const std::vector<FirmwareProfile> firmware_profiles =
{
    { 0,  0, 39, 1, feetech_servo::SCS  },
    { 1,  0, 19, 0, feetech_servo::SMCL },
    { 1, 20, 39, 0, feetech_servo::SMCL },
    { 2, 40, 69, 0, feetech_servo::SMBL },
    { 3,  0, 39, 0, feetech_servo::STS  },
    { 3, 20, 39, 1, feetech_servo::SCS2 },
    { 3, 40, 59, 0, feetech_servo::HLS  },
};

}
```

Then the model table (paste the generated `SERVO_MODEL` lines from Task 2 Step 2 output):

```cpp
#define SERVO_MODEL(major, minor, name, end) { (uint16_t)((minor)<<8 | (major)), ModelEntry{name, end} }
static const std::map<uint16_t, ModelEntry> &model_list()
{
    static const std::map<uint16_t, ModelEntry> list =
    {
        // ... 77 generated entries ...
    };
    return list;
}
#undef SERVO_MODEL
```

And the resolver:

```cpp
ServoProfile resolveServo(uint16_t model_number, uint16_t firmware_version)
{
    ServoProfile p;
    p.name   = "Unknown";
    p.series = UNKNOWN;
    p.end    = 0;
    p.known  = false;

    const auto &models = model_list();
    auto it = models.find(model_number);
    if(it != models.end())
    {
        p.name = it->second.name;
        p.end  = it->second.end;
    }

    const uint8_t fw_major = firmware_version & 0xff;
    const uint8_t fw_minor = (firmware_version >> 8) & 0xff;

    for(const auto &fp : firmware_profiles)
    {
        if(fp.major != fw_major)
            continue;
        if(fw_minor < fp.minor_start || fp.minor_end < fw_minor)
            continue;
        // When the model is unknown its endianness is unknown too, so accept the
        // first firmware match. When the model IS known, the flag disambiguates
        // the overlapping 3.20-3.39 range.
        if(it != models.end() && fp.end != p.end)
            continue;

        p.series = fp.series;
        p.end    = fp.end;
        p.known  = true;
        if(it == models.end())
            p.name = QString("Unknown (fw %1.%2)").arg(fw_major).arg(fw_minor);
        break;
    }

    return p;
}

int SCSerial::read_firmware_version(int id)
{
    error_ = 0;
    int major = read_byte(id, 0);
    if(major == -1)
    {
        error_ = 1;
        return -1;
    }
    int minor = read_byte(id, 1);
    if(minor == -1)
    {
        error_ = 1;
        return -1;
    }
    return (minor << 8) | major;
}
```

Delete `getModelType` and `getModelSeries` entirely.

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /tmp/hls-build/tests && make -j$(nproc) && ./test_packets
```

Expected: `Totals: 7 passed, 0 failed`.

- [ ] **Step 6: Commit**

The app will not build yet — `mainwindow.cpp` still calls the deleted functions. That is Task 5. Commit the library layer with the tests green.

```bash
git add servo/scserial.h servo/scserial.cpp tests/test_packets.cpp
git commit -m "feat: resolve servo series from firmware version, not display name

Model major 10 spans two register maps (HTS uses STS, HLS uses HLS), so
name-prefix matching cannot be correct. Unknown servos now fail closed."
```

---

### Task 4: HLS control class

**Files:**
- Create: `servo/hlscl.h`
- Modify: `servo/scserial.h:79-80` (include it)
- Modify: `FT_SCServo_Debug_Qt.pro:20-26` (add to `HEADERS`)
- Modify: `tests/tests.pro` (add `../servo/hlscl.h` to `HEADERS`)
- Modify: `tests/test_packets.cpp` (add the regression-lock test)

**Interfaces:**
- Consumes: `CapturingSerial` (Task 1), `feetech_servo::HLS` (Task 3)
- Produces: `feetech_servo::HLSCL` with `write_pos_ex(id, pos, speed, acc, torque)`, `reg_write_pos_ex(...)`, `sync_write_pos_ex(...)`, `write_spe`, `write_ele`, `enable_torque`, `servo_mode`, `wheel_mode`, `ele_mode`

- [ ] **Step 1: Write the failing test — the regression lock on the original defect**

Add slots and bodies to `tests/test_packets.cpp`:

```cpp
    void hls_write_pos_ex_puts_torque_at_44_45();
    void hls_write_pos_ex_encodes_negative_position_as_sign_bit();
```

```cpp
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
    QCOMPARE(p[0], uint8_t(50));            // ACC          @41
    QCOMPARE(p[1], uint8_t(4095 & 0xff));   // Position L   @42
    QCOMPARE(p[2], uint8_t(4095 >> 8));     // Position H   @43
    QCOMPARE(p[3], uint8_t(500 & 0xff));    // Goal Torque L@44
    QCOMPARE(p[4], uint8_t(500 >> 8));      // Goal Torque H@45
    QCOMPARE(p[5], uint8_t(60));            // Speed L      @46
    QCOMPARE(p[6], uint8_t(0));             // Speed H      @47

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
```

- [ ] **Step 2: Run to verify failure**

```bash
cd /tmp/hls-build/tests && make -j$(nproc) 2>&1 | tail -5
```

Expected: FAIL to compile — `feetech_servo::HLSCL` does not exist.

- [ ] **Step 3: Create `servo/hlscl.h`**

Ported from `FTServo_Linux/src/HLSCL.cpp`, converted to the Qt app's header-only, `snake_case`, `scserial_`-delegating style (mirror `servo/sms_sts.h`).

```cpp
#ifndef HLSCL_H
#define HLSCL_H

//-------EPROM(read only)--------
#define HLSCL_MODEL_L 3
#define HLSCL_MODEL_H 4

//-------EPROM(read write)--------
#define HLSCL_ID 5
#define HLSCL_BAUD_RATE 6
#define HLSCL_SECOND_ID 7
#define HLSCL_MIN_ANGLE_LIMIT_L 9
#define HLSCL_MIN_ANGLE_LIMIT_H 10
#define HLSCL_MAX_ANGLE_LIMIT_L 11
#define HLSCL_MAX_ANGLE_LIMIT_H 12
#define HLSCL_CW_DEAD 26
#define HLSCL_CCW_DEAD 27
#define HLSCL_OFS_L 31
#define HLSCL_OFS_H 32
#define HLSCL_MODE 33

//-------SRAM(read write)--------
#define HLSCL_TORQUE_ENABLE 40
#define HLSCL_ACC 41
#define HLSCL_GOAL_POSITION_L 42
#define HLSCL_GOAL_POSITION_H 43
#define HLSCL_GOAL_TORQUE_L 44
#define HLSCL_GOAL_TORQUE_H 45
#define HLSCL_GOAL_SPEED_L 46
#define HLSCL_GOAL_SPEED_H 47
#define HLSCL_TORQUE_LIMIT_L 48
#define HLSCL_TORQUE_LIMIT_H 49
#define HLSCL_LOCK 55

//-------SRAM(read only)--------
#define HLSCL_PRESENT_POSITION_L 56
#define HLSCL_PRESENT_POSITION_H 57
#define HLSCL_PRESENT_SPEED_L 58
#define HLSCL_PRESENT_SPEED_H 59
#define HLSCL_PRESENT_LOAD_L 60
#define HLSCL_PRESENT_LOAD_H 61
#define HLSCL_PRESENT_VOLTAGE 62
#define HLSCL_PRESENT_TEMPERATURE 63
#define HLSCL_MOVING 66
#define HLSCL_PRESENT_CURRENT_L 69
#define HLSCL_PRESENT_CURRENT_H 70

namespace feetech_servo
{

class SCSerial;

// HLS-series servos. Unlike SMS/STS, addresses 44/45 hold Goal Torque -- the
// maximum torque current for the move, in units of 6.5mA. Writing 0 there
// commands a 0mA limit and the servo will not move.
class HLSCL
{
public:
    HLSCL(SCSerial *scserial) : scserial_(scserial) {}

    int write_pos_ex(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0, uint16_t Torque = 0)
    {
        uint8_t bBuf[7];
        pack_pos(bBuf, Position, Speed, ACC, Torque);
        return scserial_->gen_write(ID, HLSCL_ACC, bBuf, 7);
    }

    int reg_write_pos_ex(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0, uint16_t Torque = 0)
    {
        uint8_t bBuf[7];
        pack_pos(bBuf, Position, Speed, ACC, Torque);
        return scserial_->reg_write(ID, HLSCL_ACC, bBuf, 7);
    }

    void sync_write_pos_ex(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[], uint16_t Torque[])
    {
        std::vector<uint8_t> offbuf(7 * IDN);
        for(uint8_t i = 0; i < IDN; i++)
        {
            pack_pos(offbuf.data() + i * 7,
                     Position[i],
                     Speed ? Speed[i] : 0,
                     ACC ? ACC[i] : 0,
                     Torque ? Torque[i] : 0);
        }
        scserial_->sync_write(ID, IDN, HLSCL_ACC, offbuf.data(), 7);
    }

    int write_spe(uint8_t ID, int16_t Speed, uint8_t ACC = 0, uint16_t Torque = 0)
    {
        if(Speed < 0)
        {
            Speed = -Speed;
            Speed |= (1 << 15);
        }
        uint8_t bBuf[7];
        bBuf[0] = ACC;
        scserial_->host_2_scs(bBuf + 1, bBuf + 2, 0);
        scserial_->host_2_scs(bBuf + 3, bBuf + 4, Torque);
        scserial_->host_2_scs(bBuf + 5, bBuf + 6, Speed);
        return scserial_->gen_write(ID, HLSCL_ACC, bBuf, 7);
    }

    int write_ele(uint8_t ID, int16_t Torque)
    {
        if(Torque < 0)
        {
            Torque = -Torque;
            Torque |= (1 << 15);
        }
        return scserial_->write_word(ID, HLSCL_GOAL_TORQUE_L, Torque);
    }

    int servo_mode(uint8_t ID)  { return scserial_->write_byte(ID, HLSCL_MODE, 0); }
    int wheel_mode(uint8_t ID)  { return scserial_->write_byte(ID, HLSCL_MODE, 1); }
    int ele_mode(uint8_t ID)    { return scserial_->write_byte(ID, HLSCL_MODE, 2); }

    int enable_torque(uint8_t ID, uint8_t Enable)
    {
        return scserial_->write_byte(ID, HLSCL_TORQUE_ENABLE, Enable);
    }

    int unlock_eprom(uint8_t ID)
    {
        enable_torque(ID, 0);
        return scserial_->write_byte(ID, HLSCL_LOCK, 0);
    }

    int lock_eprom(uint8_t ID)
    {
        return scserial_->write_byte(ID, HLSCL_LOCK, 1);
    }

private:
    // Addresses 41..47 in one burst: ACC, Position, Goal Torque, Speed.
    void pack_pos(uint8_t *buf, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque)
    {
        if(Position < 0)
        {
            Position = -Position;
            Position |= (1 << 15);
        }
        buf[0] = ACC;
        scserial_->host_2_scs(buf + 1, buf + 2, Position);
        scserial_->host_2_scs(buf + 3, buf + 4, Torque);
        scserial_->host_2_scs(buf + 5, buf + 6, Speed);
    }

    SCSerial *scserial_;
};

}

#endif // HLSCL_H
```

- [ ] **Step 4: Wire it into the includes and build files**

At the bottom of `servo/scserial.h`, alongside the existing includes:

```cpp
#include "sms_sts.h"
#include "scscl.h"
#include "hlscl.h"
```

In `FT_SCServo_Debug_Qt.pro`, add to `HEADERS`:

```
    servo/hlscl.h \
```

In `tests/tests.pro`, add to `HEADERS`:

```
    ../servo/hlscl.h
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /tmp/hls-build/tests && qmake /home/phoenix/signalbotics/feetech/FT_SCServo_Debug_Qt/tests/tests.pro && make -j$(nproc) && ./test_packets
```

Expected: `Totals: 9 passed, 0 failed`.

- [ ] **Step 6: Commit**

```bash
git add servo/hlscl.h servo/scserial.h FT_SCServo_Debug_Qt.pro tests/tests.pro tests/test_packets.cpp
git commit -m "feat: add HLSCL control class with Goal Torque at addr 44/45

Ported from FTServo_Linux/src/HLSCL.cpp. Includes a regression test
asserting Goal Torque is never zero-filled."
```

---

### Task 5: Wire HLS into the UI and collapse duplicated dispatch

**Files:**
- Modify: `mainwindow.h:42-43` (declarations), `mainwindow.h:90-92` (members), `mainwindow.h:106` (selected-servo struct)
- Modify: `mainwindow.cpp:29-30` (construction), `mainwindow.cpp:227-296` (dispatch), `mainwindow.cpp:390-395` (scan), `mainwindow.cpp:405-410` (selection), `mainwindow.cpp:465-471`, `526`, `558`, `597`, `626`, `867`

**Interfaces:**
- Consumes: `resolveServo`, `ServoProfile`, `ModelSeries` (Task 3); `HLSCL` (Task 4)
- Produces: `MainWindow::writeGoal(int pos, int time, int speed, int acc, int torque)` — single dispatch point for all position commands

- [ ] **Step 1: Add the HLS member and profile state**

`mainwindow.h`, alongside the existing servo pointers:

```cpp
    feetech_servo::HLSCL *hls_serial_;
```

Change the selected-servo struct to carry the whole profile:

```cpp
    struct
    {
        feetech_servo::ServoProfile profile_;
        int id_ = -1;
    }select_servo_;
```

Replace the two old declarations with:

```cpp
    void applyServoProfile(const feetech_servo::ServoProfile &profile);
    const std::vector<feetech_servo::MemoryConfig>& getMemConfig(feetech_servo::ModelSeries series);
    void writeGoal(int pos, int time, int speed, int acc, int torque);
    int currentTorqueField() const;
```

- [ ] **Step 2: Construct and destroy the HLS object**

`mainwindow.cpp` near line 29:

```cpp
    sms_sts_serial_ = new feetech_servo::SMS_STS(scserial_);
    scs_serial_ = new feetech_servo::SCSCL(scserial_);
    hls_serial_ = new feetech_servo::HLSCL(scserial_);
```

and in the destructor near line 52:

```cpp
    delete hls_serial_;
```

- [ ] **Step 3: Replace `selectServoSeries` with `applyServoProfile`**

Endianness now comes from the profile rather than a name prefix, and unknown servos disable control.

```cpp
void MainWindow::applyServoProfile(const feetech_servo::ServoProfile &profile)
{
    select_servo_.profile_ = profile;
    scserial_->set_end(profile.end);

    // Fail closed: without a resolved register map, any write could land on the
    // wrong register. Show state, refuse to command.
    ui->controlGroupBox->setEnabled(profile.known);

    if(profile.series == feetech_servo::HLS)
    {
        ui->timeLabel->setText("Torque (x6.5mA)");
        if(ui->timeLineEdit->text().toInt() == 0)
            ui->timeLineEdit->setText("500");
    }
    else
    {
        ui->timeLabel->setText("Time");
    }

    updatePorgMemTable();
}
```

If `controlGroupBox` or `timeLabel` are not the actual object names in `mainwindow.ui`, find them first:

```bash
grep -n 'timeLineEdit' -B12 mainwindow.ui | grep -E 'name="(label|groupBox)' 
```

Use the real names; do not invent them.

- [ ] **Step 4: Extend `getMemConfig` for the new series**

```cpp
const std::vector<feetech_servo::MemoryConfig>& MainWindow::getMemConfig(feetech_servo::ModelSeries series)
{
    switch(series)
    {
        case feetech_servo::ModelSeries::SCS:   return feetech_servo::SCSMemConfig;
        case feetech_servo::ModelSeries::SCS2:  return feetech_servo::SCS2MemConfig;
        case feetech_servo::ModelSeries::STS:   return feetech_servo::STSMemConfig;
        case feetech_servo::ModelSeries::SMBL:  return feetech_servo::SMBLMemConfig;
        case feetech_servo::ModelSeries::SMCL:  return feetech_servo::SMCLMemConfig;
        case feetech_servo::ModelSeries::HLS:   return feetech_servo::HLSMemConfig;
        default:                                return feetech_servo::STSMemConfig;
    }
}
```

- [ ] **Step 5: Collapse the nine dispatch sites into `writeGoal`**

```cpp
int MainWindow::currentTorqueField() const
{
    return ui->timeLineEdit->text().toInt();
}

void MainWindow::writeGoal(int pos, int time, int speed, int acc, int torque)
{
    if(!select_servo_.profile_.known)
        return;

    switch(select_servo_.profile_.series)
    {
        case feetech_servo::ModelSeries::SCS:
        case feetech_servo::ModelSeries::SCS2:
            scs_serial_->write_pos(select_servo_.id_, pos, time, speed);
            break;

        case feetech_servo::ModelSeries::HLS:
            hls_serial_->servo_mode(select_servo_.id_);
            hls_serial_->write_pos_ex(select_servo_.id_, pos, speed, acc, torque);
            break;

        default:
            sms_sts_serial_->rotation_mode(select_servo_.id_);
            sms_sts_serial_->write_pos_ex(select_servo_.id_, pos, speed, acc);
            break;
    }
}
```

Then rewrite `writePos`, `syncWritePos` and `regWritePos` to route through the same switch shape, and replace each of the six auto-debug branches (lines ~526, 558, 597, 626) with:

```cpp
        writeGoal(latest_auto_debug_goal_, 0, 0, 0, currentTorqueField());
```

Apply the same three-way switch to the torque-enable handler at line ~465 and the memory-write handler at line ~867, replacing `if(... == SCS)` with a switch that routes `HLS` to `hls_serial_`.

- [ ] **Step 6: Update the scan and selection paths**

At line ~390, read both identifiers and resolve:

```cpp
        if(0 < ret)
        {
            int mid = scserial_->read_model_number(ret);
            int fw  = scserial_->read_firmware_version(ret);
            auto profile = feetech_servo::resolveServo(mid, fw);

            qInfo("ID %d: model=0x%04x firmware=0x%04x -> %s",
                  ret, mid, fw, qPrintable(profile.name));

            appendServoList(ret, profile.name);
            id_list_.push_back(ret);
            select_servo_.id_ = ret;
            applyServoProfile(profile);
        }
```

The `qInfo` line is required — verification step 1 depends on the raw bytes being observable.

At line ~405, selection must re-resolve rather than parse the displayed name. Store the profile per row so reselection does not need a second round of serial reads:

```cpp
    // Stash the resolved profile on the row so selection needs no serial IO.
    auto *id_item = new QStandardItem(QString::number(id));
    id_item->setData(QVariant::fromValue(static_cast<int>(profile.series)), Qt::UserRole + 1);
    id_item->setData(profile.end,   Qt::UserRole + 2);
    id_item->setData(profile.known, Qt::UserRole + 3);
```

and in `onServoListSelection`, rebuild a `ServoProfile` from those roles and call `applyServoProfile`. Update `appendServoList` to take the profile.

- [ ] **Step 7: Build the app and run the tests**

```bash
cd /tmp/hls-build/app && qmake /home/phoenix/signalbotics/feetech/FT_SCServo_Debug_Qt/FT_SCServo_Debug_Qt.pro && make -j$(nproc)
cd /tmp/hls-build/tests && make -j$(nproc) && ./test_packets
```

Expected: app links clean; `Totals: 9 passed, 0 failed`.

- [ ] **Step 8: Commit**

```bash
git add mainwindow.h mainwindow.cpp
git commit -m "feat: route HLS servos through HLSCL, fail closed on unknown

Collapses nine duplicated two-way control branches into writeGoal().
timeLineEdit is relabelled to Torque (x6.5mA) for HLS and defaults to 500."
```

---

### Task 6: Hardware verification

No code. This is the evidence-gathering task, and nothing may be described as working until it passes.

**Files:** none. Produces `docs/hls-verification.md`.

**Preconditions:**
- HLS servo connected via USB-TTL adapter, powered, **unloaded and clear of its travel limits**
- `ls /dev/ttyUSB*` shows a port; user is in the `dialout` group
- Steps 3 and 5 command a physical actuator — invoke `roboforge:safe-hardware-deployment` before each

- [ ] **Step 1: Identification**

Run the app, scan. Record the `qInfo` line.

Expected: name is `HLS39xx` (not `Unknown`), firmware major is 3 and minor is within 40-59. **If the firmware is outside that range, stop** — the resolver's central assumption is wrong and the plan needs revisiting before any motion command.

- [ ] **Step 2: Register table**

Compare the register view against FD 1.9.8.5 under Wine, side by side. Confirm `Goal Torque` at 44 and `Kp`/`Kd`/`Ki`/`Km` at 50-53.

Expected divergence, not a defect: velocity readings differ by 25x from FD, because per-family display scaling is explicitly out of scope (spec, Reference data).

- [ ] **Step 3: Control — the acceptance test**

Set a goal position, confirm `Torque (x6.5mA)` reads 500, send.

Expected: **the servo physically moves to the commanded position.** Nothing in this work may be called done until this is observed.

- [ ] **Step 4: Non-regression on SCS and STS**

Scan, read state, and command a move on an SCS servo and an STS servo. Compare against the same operations on `master` (`git stash` or a second build) if anything looks off.

Expected: identical behaviour to before the change.

- [ ] **Step 5: SMCL position-offset dir_bit**

On an SMS/SMCL servo, read Position Offset (addr 33) and compare against FD's reading for the same servo.

Expected: they now agree. Task 2 changed `dir_bit` 15 → 11. If they disagree, CN was wrong for this row and it must be reverted with a note in the spec.

- [ ] **Step 6: Fail-closed**

Confirm the control panel is disabled for a servo the tables do not know. If no such servo is available, temporarily comment out one model-table row, rebuild, and confirm.

- [ ] **Step 7: Record the evidence**

Write `docs/hls-verification.md` with the actual `qInfo` output, what moved, what did not, and anything unverified. Commit it — unlike the spec and plan, this is a deliverable and stays.

```bash
git add docs/hls-verification.md
git commit -m "docs: record HLS hardware verification results"
```

---

## Notes for the implementer

- **`sync_write_pos_ex` signature changed shape** for HLS (extra `Torque[]`). `MainWindow::syncWritePos` must build a torque vector for the HLS branch.
- **`servo/scserial.h` includes `hlscl.h` at the bottom**, and `hlscl.h` forward-declares `SCSerial` and calls its methods inline. This works because the include sits after the class definition — same trick `sms_sts.h` already relies on. Do not move the include to the top.
- **`tests/` is a separate qmake project.** It is not built by the main `.pro`. Run both.
- If a generated table row trips the `no English name` guard, the correct action is to add a `NAME_OVERRIDES` entry with a name derived from the Chinese term — never to drop the row or invent an unrelated name.
