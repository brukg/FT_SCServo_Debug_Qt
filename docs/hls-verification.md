# HLS servo support — hardware verification

Procedure for validating HLS support on real hardware.

**Status: HLS identification and control both verified on hardware (2026-07-22).**
The acceptance test passed — an HLS3955 moved to a commanded position with zero
error. Steps 4 and 5 remain unrun because no SCS/STS/SMS servo is on this bus.

## Background

FD 1.9.8.5 drives HLS servos correctly; the Qt tool did not. Three defects:

1. The model table stopped at major 9; HLS reports major 10, so it displayed `Unknown`.
2. `Unknown` fell through `getModelSeries()`'s catch-all to SMCL — no default-deny.
3. Position commands write a 7-byte burst at address 41. Addresses 44/45 were
   zero-filled, which is harmless on SMS/STS but on HLS is **Goal Torque** — the
   max torque current in 6.5mA units. A zero there commands 0 mA and the servo
   cannot move.

The register map is now selected by **firmware version** (addresses 0/1), matching
FD, because model major 10 spans two different maps: HTS/STS…BL use the STS map,
HLS uses the HLS map.

## Build

From the repo root. `build/` is gitignored.

```bash
mkdir -p build && cd build && qmake .. && make -j$(nproc)
./FT_SCServo_Debug_Qt
```

Unit tests (no hardware needed):

```bash
mkdir -p build/tests && cd build/tests && qmake ../../tests/tests.pro && make -j$(nproc)
./test_packets
```

Read-only bus probe — pings every ID and reports what the resolver makes of each.
Sends no writes, so it is safe to run at any time:

```bash
mkdir -p build/probe && cd build/probe && qmake ../../tools/probe.pro && make -j$(nproc)
./probe_bus /dev/ttyACM0 1000000
```

## Finding the serial port

Do **not** assume `/dev/ttyUSB*`. FeeTech's CH340/CH343 adapters enumerate as
**CDC-ACM** on current kernels, appearing as `/dev/ttyACM0` with no `usbserial`
driver involved. Enumerate by id instead, which works regardless of driver:

```bash
ls -l /dev/serial/by-id/
```

On this machine that resolves to:

```
usb-1a86_USB_Single_Serial_5B79079548-if00 -> ../../ttyACM0
```

(`1a86` is QinHeng, the CH34x vendor.)

## Safety

Steps 3 and 5 command a physical actuator.

- Servo **unloaded** and clear of its travel limits before any motion command.
- Torque field starts at 500 (≈3250 mA). Lower it for the first move if the servo
  is small; raise only after motion is confirmed.
- Keep the power supply within reach.

## Procedure

| # | Step | Expected | Result |
|---|---|---|---|
| 1 | Scan for servos, capture model + firmware bytes. | Name is `HLS39xx`, **not** `Unknown`. Firmware major 3, minor within 40–59. | ✅ **pass** — see below |
| 2 | Open the register view. Compare against FD 1.9.8.5 under Wine, side by side. | `Goal Torque` at addr 44; `Kp`/`Kd`/`Ki`/`Km` at 50–53; Work Mode max 4. | ⬜ pending |
| 3 | **Command a position with non-zero torque.** | **The servo physically moves to the commanded position.** | ✅ **PASS** — ID 1 moved 3083→2883, 0 counts error |
| 4 | Repeat scan / read / move on an SCS servo and an STS servo. | Identical behaviour to before this change. | ⬜ pending — no SCS/STS servo on this bus |
| 5 | On an SMS/SMCL servo, read Position Offset (addr 33). Compare with FD's reading. | They agree. | ⬜ pending — no SMS servo on this bus |
| 6 | Select a servo the tables do not know. | Register map still resolves from firmware; control stays enabled. | ✅ **pass** — ID 4 is exactly this case |
| 7 | Run the unit tests. | `Totals: 11 passed, 0 failed`. | ✅ pass |

### Step 1 result — measured 2026-07-22 on `/dev/ttyACM0` @ 1 Mbps

```
ID 1  model=0x130a (major 10, minor 19)  firmware=0x2b03 (3.43)  -> HLS3955           series=HLS known=yes
      pos=3083 volt=117 temp=28 mode=0 torque_en=1 goal_torque(44)=0
ID 2  model=0x140a (major 10, minor 20)  firmware=0x2b03 (3.43)  -> HLS3915           series=HLS known=yes
      pos=4095 volt=117 temp=33 mode=0 torque_en=0 goal_torque(44)=500
ID 4  model=0x1b0a (major 10, minor 27)  firmware=0x2d03 (3.45)  -> Unknown (fw 3.45)  series=HLS known=yes
      pos=4078 volt=118 temp=33 mode=0 torque_en=0 goal_torque(44)=450
```

Firmware 3.43 and 3.45 both fall inside 3.40–3.59, confirming on real hardware the
assumption the whole resolver rests on.

**ID 1 shows the original defect in the servo's own registers**: torque enabled,
goal torque 0. That is the state the Qt tool left servos in — energised, with a
0 mA current limit, unable to move.

### Step 6 result — model newer than FD's own table

ID 4 reports model minor **27**, which is absent from FD 1.9.8.5's `[型号]` table
(it stops at 25). FeeTech shipped a servo newer than their own debug tool.

Because the register map is keyed off firmware rather than the name, ID 4 still
resolves to the HLS map and **remains controllable**, displayed as
`Unknown (fw 3.45)`. Under the previous name-matching logic it would have fallen
through to SMCL and been silently mis-driven. This is the case that justified
firmware-keyed resolution over simply extending the model list.

### Step 3 result — acceptance test PASSED, 2026-07-22

`tools/motion_test.cpp` against ID 1 (HLS3955) on `/dev/ttyACM0` @ 1 Mbps.
Guarded: torque 150 (≈975 mA, vs the servo's own limit of 980), speed 30, accel
20, motion bounded in code to −200 counts (≈17.6°).

```
== pre-flight ==
  present_pos = 3083   work_mode = 0   torque_enable = 1   goal_torque = 0

== e-stop test (torque disable) ==
  torque_enable after disable = 0  OK

== motion ==
  3083 -> 2883  (delta -200, torque 150, speed 30, acc 20)
  write_pos_ex returned 1
  goal_torque now reads 150
  t=0ms    pos=3082  (moved -1)
  t=200ms  pos=3059  (moved -24)
  t=401ms  pos=2967  (moved -116)
  t=601ms  pos=2893  (moved -190)
  t=802ms  pos=2884  (moved -199)
  t=1006ms pos=2883  (moved -200)

== result ==
  start 3083 | target 2883 | final 2883 | moved -200 | error 0 counts
```

`goal_torque` transitioning from 0 to 150 is the fix itself, observed in the
servo's own register. Before this change the Qt tool wrote 0 there on every
command, which is a 0 mA current limit and why the servo never moved.

Reproduce:

```bash
mkdir -p build/motion && cd build/motion
qmake ../../tools/motion_test.pro && make -j$(nproc)
./motion_test /dev/ttyACM0 1000000 1
```

The tool disables torque before returning, and aborts without commanding motion
if the torque-disable check fails.

### Step 1 — stop condition

If firmware is **outside 3.40–3.59**, stop and report it. The resolver's central
assumption is that HLS lives in that range; if it doesn't, the firmware profile
table needs correcting before any motion command is sent.

### Step 3 — the acceptance test

Nothing in this work may be described as done until the servo moves. If it does
not:

- Check the Torque field is non-zero. Zero reproduces the original bug exactly.
- Check Torque Enable is on.
- Check Work Mode is 0 (servo/position mode). `writeGoal` sets this automatically
  for HLS, but a previously-set mode 2 (constant force) would behave differently.

### Step 5 — expected behaviour change

This change corrects the SMCL Position Offset `dir_bit` from 15 to 11. The value
range is ±2047, which is 11 bits, so the sign bit is bit 11 — the old value was
wrong. This means **sign decoding of Position Offset changes for SMS servos**,
which currently appear to work. If FD and the Qt tool now disagree, the CN config
was wrong for this row and it should be reverted with a note in the spec.

### Step 2 — expected divergence, not a defect

HLS velocity readings will differ from FD by a factor of 25. FD scales velocity
per family (`vel_scale` 50 for HLS vs 2 for STS); the Qt tool applies no display
scaling at all and never has. Adding it is a separate feature, deliberately out of
scope. It affects display only — not the register view, not any command sent.

## What was verified without hardware

| Check | Evidence |
|---|---|
| App builds clean, no new warnings | `qmake && make` on Qt 5.15.18 |
| App launches without crashing | ran under `QT_QPA_PLATFORM=offscreen`, exit 124 (timeout, i.e. ran) |
| HLS position command carries torque at 44/45 | `hls_write_pos_ex_puts_torque_at_44_45` |
| Negative positions encode as sign bit 15 | `hls_write_pos_ex_encodes_negative_position_as_sign_bit` |
| HTS and HLS resolve to different maps despite sharing model major 10 | `resolver_distinguishes_hts_from_hls_same_major` |
| Endianness breaks the overlapping 3.20–3.39 firmware range | `resolver_uses_endianness_to_break_firmware_overlap` |
| Unknown servos fail closed | `resolver_fails_closed_on_unknown_model` |
| Unknown model + known firmware is still controllable | `resolver_controls_unknown_model_with_known_firmware` |
| SMS/STS write path unchanged | `sms_sts_write_pos_ex_emits_zero_at_44_45`, `sms_sts_packet_framing_is_wellformed` |

Unit tests prove the **bytes on the wire** are right; they cannot prove the servo
responds to them. Step 3 above closes that gap on real hardware.

## Regenerating the register tables

The tables in `servo/servo_types.h` and the model table in `servo/scserial.cpp`
are generated from FD's config, not hand-written:

```bash
python3 tools/gen_servo_tables.py "path/to/FD1985-250729/ft_setup_bat"
```

FD ships two config files that have drifted apart and disagree in ~48 places.
`setup.log` (Chinese) is authoritative for structure, sizes, direction bits and
ranges; `setup_en.log` supplies display names. Where the English name contradicts
Chinese semantics — HLS addr 44 being the clearest case, where EN says `Goal PWM`
copied from the STS table — the generator overrides it via `NAME_OVERRIDES`.
