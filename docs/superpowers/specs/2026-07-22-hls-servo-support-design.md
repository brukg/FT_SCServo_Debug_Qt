# Design: HLS servo support for FT_SCServo_Debug_Qt

Date: 2026-07-22
Branch: `feat/hls-servo-support`

## Problem

HLS-series servos connected to the Qt debug tool are detected by `Ping`, are listed
as `Unknown`, report their state correctly, and **cannot be commanded to move**.

The Windows tool FD 1.9.8.5 drives the same servos correctly under Wine.

## Root cause

Three defects compound. All three are confirmed by reading code, not inferred.

### 1. The model table stops at major 9

`servo/scserial.cpp:9-69` hardcodes a model-number → name map whose highest major
version is `9`. HLS and HTS servos report **model major 10**. The lookup misses and
returns `"Unknown"`.

### 2. `Unknown` silently resolves to the wrong register map

`getModelSeries(QString)` (`servo/scserial.cpp:79-97`) infers the servo family by
string-matching the *display name*:

```cpp
if      (modelName.startsWith("STS"))                          return STS;
else if (modelName.startsWith("SC"))                           return SCS;
else if (modelName.startsWith("SM") && modelName.contains("BL")) return SMBL;
else                                                            return SMCL;   // <-- "Unknown" lands here
```

There is no default-deny. Any unrecognised servo is treated as SMCL.

This is the structural defect. Even with every model name added, the function
cannot be correct: model major 10 spans **two different register maps** —
`HTS3235`/`STS3045BL` use the STS map, `HLS3625` uses the HLS map. A name prefix
cannot distinguish them.

### 3. Reads work by coincidence; writes corrupt a live register

The read-only SRAM block is byte-identical across SMCL, STS and HLS — position@56,
velocity@58, load@60, voltage@62, temperature@63, moving@66, current@69. So the
status panel and graphs are correct **by luck**, which is why the failure looks
like "reads fine, won't move".

Writes are not so lucky. `MainWindow::writePos` (`mainwindow.cpp:257`) calls
`SMS_STS::write_pos_ex` (`servo/sms_sts.h:60`), which writes a 7-byte burst
starting at address 41:

```
[ ACC | PosL | PosH |  0  |  0  | SpeedL | SpeedH ]
  41     42     43    44    45      46      47
```

Addresses 44/45 are family-dependent:

| Family | Addr 44/45 | Effect of writing 0 |
|---|---|---|
| SCS | Running Time | harmless |
| SMS / STS | PWM open-loop speed / Goal Time | harmless |
| **HLS** | **Goal Torque (max torque current)** | **0 mA current limit — servo cannot move** |

FeeTech's own example (`FTServo_Linux/examples/HLSCL/WritePos/WritePos.cpp`)
documents the units:

```cpp
//最大扭矩电流T=500*6.5=3250mA
hlscl.WritePosEx(0xfe, 4095, 60, 50, 500);
```

So every position command the Qt app sends to an HLS servo sets its torque current
limit to zero. The servo is obeying correctly.

## How FD 1.9.8.5 actually does it

FD is **data-driven**, not hardcoded. `FD1985-250729/ft_setup_bat/setup.log`
(GB18030-encoded) is parsed at startup and contains three sections:

| Section | Key | Contents |
|---|---|---|
| `[型号]` | model number (addr 3,4) | display name + endianness flag |
| `[调试]` | firmware version (addr 0,1) + endianness | feedback range, control start addr, position/velocity display scaling, torque-enable addr |
| `[内存]` ×6 | firmware version + endianness | full register map |

The critical insight is that **FD keys the register map off firmware version, not
model number**. HLS is firmware `3.40`–`3.59`.

The firmware ranges overlap — `3,0,39,0` (STS) and `3,20,39,1` (SCSXX-2) — and are
disambiguated by the endianness flag, which appears in *both* the model table and
the memory-table headers. That is why the flag is duplicated.

## Design

### Component 1 — Two-key servo resolver

Replace name-string inference with FD's algorithm:

```
ServoProfile resolve(uint16_t model_number, uint16_t firmware_version):
    (name, scs_endian) = modelTable.lookup(model_number)     // default: "Unknown", endian 0
    series             = firmwareTable.lookup(firmware_version, scs_endian)
```

Worked examples:

| Servo | Model | Flag | Firmware | Matches | Series |
|---|---|---|---|---|---|
| `HLS3625` | 10.13 | 0 | 3.4x | `3,40,59,0` | **HLS** |
| `HTS3235` | 10.1 | 0 | 3.0x | `3,0,39,0` | STS |
| `SCS15-2` | 9.15 | **1** | 3.2x | `3,20,39,1` | SCS |

`SCS15-2` demonstrates why the flag is load-bearing: its firmware also falls inside
the STS range, and only the endianness flag separates them.

Changes:

- `ModelSeries` gains `HLS` and `SCS2`.
- New `SCSerial::read_firmware_version(int id)` reading addresses 0 and 1, mirroring
  the existing `read_model_number` (`servo/scserial.cpp:366`).
- `getModelSeries(QString)` is **deleted**, not extended. Leaving it in place invites
  a future caller to reintroduce the bug.
- `MainWindow::selectServoSeries` (`mainwindow.cpp:227`) stops deriving endianness
  from `startsWith("SC")` and takes it from the resolved profile.

**Unknown servos must fail closed.** When the model number is absent from the table,
the app displays `Unknown` and **disables the control panel** rather than silently
picking a register map. Reporting nothing is recoverable; commanding a servo through
the wrong map is not. This is the single most important behavioural change in the
design — it is what converts the current silent-wrong-write into a visible refusal.

### Component 2 — Register tables regenerated from `setup.log`

`servo/servo_types.h` currently holds four tables (`SMCLMemConfig`, `SMBLMemConfig`,
`STSMemConfig`, `SCSMemConfig`). After this change it holds six, all regenerated
from FD's config: the four existing ones plus `HLSMemConfig` and `SCS2MemConfig`.

The model table in `servo/scserial.cpp` is regenerated from `[型号]` — 77 entries,
up from 58.

#### EN/CN reconciliation policy

`setup.log` (Chinese) and `setup_en.log` (English) are **not translations of one
source**. They have drifted independently and disagree in 48 places.

**Policy: CN is authoritative for structure, size, direction bits, ranges and
defaults. EN supplies display names. Where an EN name contradicts CN semantics, CN
wins and the English name is written by hand.**

Justification: EN's HLS addr 44 reads `Goal PWM, ±32766`, which is a verbatim
copy of the STS row above it, while CN reads `目标扭矩, ±2048` — and CN is
corroborated by two independent sources, `HLSCL.h:35` (`HLSCL_GOAL_TORQUE_L 44`)
and the 6.5 mA example. EN is also missing `相位` (addr 18, SCS) entirely, which
shows it was hand-edited rather than regenerated.

Conflicts requiring a written English name:

| Table | Addr | CN (authoritative) | EN (rejected) | Name to use |
|---|---|---|---|---|
| HLS | 44 | `目标扭矩`, ±2048 | `Goal PWM`, ±32766 | `Goal Torque` |
| SCS | 18 | `相位` | *absent* | `Phase` |
| SCSXX-2 | 34 | `位置偏移`, ±127, dir 7 | *absent* | `Position Offset Value` |
| SCSXX-2 | 44 | `PWM开环速度` | `Running Time` | `Goal PWM` |

Model-table conflicts, CN taken:

- `10,21` — CN `HLS3615`, EN `HLS3960`
- `10,22` — CN `HLS3960`, EN absent
- `5,3` — CN `SCS2304`, EN absent

The remaining ~40 conflicts are numeric (defaults, ranges, direction bits) and
resolve to CN by policy without needing a naming decision. The implementation plan
enumerates each one.

#### Pre-existing bug this surfaces

CN gives SMS Position Offset (addr 33) `dir_bit = 11`; EN gives `15`. The current
code hardcodes `15` (`servo_types.h:101` and `:155`). Since the value range is
±2047 — eleven bits — the sign bit is bit 11 and **CN is correct**. Sign decoding
of Position Offset is therefore already wrong for SMS servos today.

This is a real behaviour change for hardware that currently "works", so it is
called out separately in the plan and verified on an SMS servo before and after,
rather than being folded silently into the HLS work.

### Component 3 — `servo/hlscl.h`

Header-only, matching the existing style of `servo/sms_sts.h` and `servo/scscl.h`,
ported from `FTServo_Linux/src/HLSCL.cpp`:

- `write_pos_ex(id, pos, speed, acc, torque)`
- `reg_write_pos_ex(id, pos, speed, acc, torque)`
- `sync_write_pos_ex(ids, n, pos[], speed[], acc[], torque[])`
- `write_spe(id, speed, acc, torque)`, `write_ele(id, torque)`
- `enable_torque`, `lock_eprom` / `unlock_eprom`, `calibration_ofs`
- mode setters: servo(0), wheel(1), ele(2) — HLS `Work Mode` range is **0–4** vs
  STS's 0–3

Added to `HEADERS` in `FT_SCServo_Debug_Qt.pro`.

### Component 4 — Control dispatch and UI

`mainwindow.cpp` currently repeats the same two-way branch in nine places
(lines 257, 271, 287, 465, 526, 558, 597, 626, 867):

```cpp
if (select_servo_.model_ == ModelSeries::SCS) { scs_serial_->... }
else                                          { sms_sts_serial_->... }
```

Adding a third arm nine times over would trade one bug for nine future ones. These
collapse into a single `MainWindow::writeGoal(pos, time, speed, acc, torque)` helper
that switches on series once. The nine call sites become calls to it.

**UI.** `timeLineEdit` already occupies exactly the protocol slot that varies by
family (addr 44/45). Rather than adding a widget, it is relabelled on servo
selection:

| Series | Label | Default |
|---|---|---|
| SCS | `Time` | 0 |
| SMS / STS | `Time` | 0 |
| **HLS** | **`Torque (x6.5mA)`** | **500** |

`500` is FeeTech's own example value (≈3250 mA). The current effective value of `0`
is precisely what immobilises the servo.

## Scope

In scope — all four families the register work touches:

- **HLS** — model 10.10–10.22 (HLS3606/3612/3615/3620/3625/3640/3915/3925/3930/3935/3950/3955/3960)
- **HTS** — model 10.1–10.8, STS register map
- **STS…BL** — model 10.5, 10.6, 10.9, 10.25, STS register map
- **Newly-added SCS/STS/SM models** — SCS2304, SCS1025, SCS215-2, SCS46-2, SCS25-2,
  STS3015/3045/3235/5420/3095/3250, SM24BL, SM70BLHV, SM80BLHV, SM160BLHV, SM260BLHV

Out of scope:

- Runtime parsing of `setup.log` (tables stay compile-time constants). The resolver
  and tables are structured so a future parser can replace them without touching
  `mainwindow.cpp`, but that migration is deliberately not attempted in the same
  change that unbreaks the motors.
- The Modbus/PWM profiles in `setup_modbus.log` / `setup_pwm.log`.
- Firmware `20.x` (`[调试]` row `20,0,19,0`), which uses a different feedback range
  (64–80) and control address (36).

## Error handling

| Condition | Behaviour |
|---|---|
| Model number not in table | Display `Unknown`, **disable control panel**, log raw model+firmware bytes |
| Firmware version matches no profile | Same as above; do not guess a register map |
| Firmware matches profile but model unknown | Display `Unknown (fw 3.42)`, enable control using the firmware-resolved profile |
| `read_model_number` / `read_firmware_version` returns -1 | Treat as failed ping; do not add to servo list |

The third row is the one that keeps the tool useful when FeeTech ships a servo
newer than the table: the register map comes from firmware, so control is safe even
when the name is not known.

## Verification

Hardware is available, so every step has a hardware gate. Steps 3 and 5 command a
physical actuator; `roboforge:safe-hardware-deployment` is invoked before each, and
the servo must be unloaded and clear of its travel limits.

| # | Step | Evidence required |
|---|---|---|
| 1 | Identification | Scan lists `HLS39xx`, not `Unknown`. Raw model and firmware bytes logged, and firmware confirmed inside 3.40–3.59 before the resolver is trusted. |
| 2 | Register table | Register view compared side-by-side against FD under Wine. `Goal Torque`@44 and `Kp/Kd/Ki/Km`@50–53 present. |
| 3 | **Control** | **HLS servo physically moves in response to a position command.** This is the acceptance test for the whole change. |
| 4 | Non-regression | Scan, read and move an SCS and an STS servo. Compare against behaviour recorded on `master` before the change. |
| 5 | SMS dir_bit | Read Position Offset on an SMS servo before and after the `15`→`11` change; confirm the post-change value matches FD's reading. |
| 6 | Fail-closed | Confirm an unknown servo disables the control panel rather than defaulting to SMCL. |
| 7 | Packet unit test | Assert the 7-byte burst at addr 41 carries the torque value rather than `0`, for each series. Regression lock on the original defect. |

Step 7 is the test that stops this bug returning. Steps 4 and 5 exist because this
change touches tables that currently-working servos depend on; "the HLS moves now"
is not sufficient evidence that nothing else broke.

## Files touched

| File | Change |
|---|---|
| `servo/servo_types.h` | 4 tables regenerated, `HLSMemConfig` + `SCS2MemConfig` added |
| `servo/scserial.h` | `ModelSeries` gains `HLS`/`SCS2`; `getModelSeries(QString)` removed; profile resolver declared |
| `servo/scserial.cpp` | Model table regenerated (77 entries); firmware profile table added; `read_firmware_version` added |
| `servo/hlscl.h` | **New** — ported from `FTServo_Linux/src/HLSCL.cpp` |
| `mainwindow.h` / `.cpp` | Nine branches collapsed into `writeGoal`; fail-closed handling; `timeLineEdit` relabelling |
| `FT_SCServo_Debug_Qt.pro` | `servo/hlscl.h` added to `HEADERS` |

## Reference data

Source of truth: `FD1.9.8.5(250729)/FD1985-250729/ft_setup_bat/setup.log`, GB18030.
Read with `iconv -f GB18030 -t UTF-8`.

`[调试]` firmware profiles (`fw_major, minor_start, minor_end, scs_endian,
feedback_start, feedback_end, control_start, pos_scale, vel_scale, _, torque_addr, _, _`):

```
0,  0, 39, 1, 56, 72, 41, 4,  2, 0, 40, 0, 0   SCS
1,  0, 19, 0, 56, 72, 41, 1,  2, 0, 40, 0, 0   SMCL
1, 20, 39, 0, 56, 72, 41, 1,  2, 0, 40, 0, 0   SMCL
2, 40, 69, 0, 56, 72, 41, 1,  2, 0, 40, 0, 0   SMBL
3,  0, 39, 0, 56, 72, 41, 1,  2, 0, 40, 0, 0   STS
3, 20, 39, 1, 56, 72, 41, 4,  2, 0, 40, 0, 0   SCS2
3, 40, 59, 0, 56, 72, 41, 1, 50, 0, 40, 0, 0   HLS
20, 0, 19, 0, 64, 80, 36, 1,  0, 0,  0, 0, 0   (out of scope)
```

Note HLS `vel_scale = 50` against STS's `2`. This matches the example's `V*50`
timing formula and `60 * 0.732 = 43.92 rpm`.

**The app applies no display scaling today** — `mainwindow.cpp` plots raw register
values, and no `pos_scale`/`vel_scale` equivalent exists anywhere in the codebase.
Adding per-family scaling is therefore a *new feature*, not part of restoring HLS
control, and is **out of scope** here.

Consequence, accepted deliberately: HLS velocity readings in the graph and status
panel will be numerically 25× different from what FD displays for the same servo.
This is cosmetic — it affects neither the register view nor any command sent to the
servo — but it will be visible during step 2 of verification when comparing against
FD side by side, and must not be mistaken for a defect. The `pos_scale`/`vel_scale`
columns are captured in the firmware profile table so that a follow-up change can
implement scaling without re-deriving them.
