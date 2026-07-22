# HLS servo support — hardware verification

Procedure for validating HLS support on real hardware. **Results are pending: no
serial adapter was connected when the code was written, so nothing below has been
executed against a servo.** Fill in the Result column as you go.

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

```bash
cd <repo>
mkdir -p /tmp/hls-build/app && cd /tmp/hls-build/app
qmake <repo>/FT_SCServo_Debug_Qt.pro && make -j$(nproc)
./FT_SCServo_Debug_Qt
```

Unit tests (no hardware needed):

```bash
mkdir -p /tmp/hls-build/tests && cd /tmp/hls-build/tests
qmake <repo>/tests/tests.pro && make -j$(nproc) && ./test_packets
```

## Safety

Steps 3 and 5 command a physical actuator.

- Servo **unloaded** and clear of its travel limits before any motion command.
- Torque field starts at 500 (≈3250 mA). Lower it for the first move if the servo
  is small; raise only after motion is confirmed.
- Keep the power supply within reach.

## Procedure

| # | Step | Expected | Result |
|---|---|---|---|
| 1 | Scan for servos. Capture the `qInfo` line from stdout. | Name is `HLS39xx`, **not** `Unknown`. Firmware major is 3, minor within 40–59. | ⬜ pending |
| 2 | Open the register view. Compare against FD 1.9.8.5 under Wine, side by side. | `Goal Torque` at addr 44; `Kp`/`Kd`/`Ki`/`Km` at 50–53; Work Mode max 4. | ⬜ pending |
| 3 | **Set a goal position. Confirm the Torque field reads 500. Send.** | **The servo physically moves to the commanded position.** | ⬜ pending |
| 4 | Repeat scan / read / move on an SCS servo and an STS servo. | Identical behaviour to before this change. | ⬜ pending |
| 5 | On an SMS/SMCL servo, read Position Offset (addr 33). Compare with FD's reading. | They agree. | ⬜ pending |
| 6 | Select a servo the tables do not know. | "Servo Control" group is **disabled** — no commands can be sent. | ⬜ pending |
| 7 | Run the unit tests. | `Totals: 11 passed, 0 failed`. | ✅ passing (no hardware needed) |

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

Unit tests prove the **bytes on the wire** are right. They cannot prove the servo
responds to them. That is what step 3 is for.

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
