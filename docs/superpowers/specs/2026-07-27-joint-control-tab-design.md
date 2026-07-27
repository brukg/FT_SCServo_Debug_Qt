# Design: Joint Control tab

Date: 2026-07-27
Branch: `feat/joint-control-tab`

## Problem

The Debug tab controls one selected servo at a time. Driving a multi-joint robot
needs per-joint torque enable/disable, a per-joint position adjuster, and the
ability to move a chosen subset of joints together. The existing "Sync Write"
radio mode writes to **all** discovered servos with a single shared position,
which cannot express "move these specific joints to these specific targets".

## Goal

A new tab, **Joint Control**, that lists every discovered servo as a row with:
per-row torque enable, a per-row position adjuster (live jog), a live present-position
readout, and a per-row "include in sync write" checkbox — plus master controls to
enable/disable torque across all servos and to sync-write the selected group.

## Interaction model (decided)

- **Live jog + explicit group sync.** Dragging a row's adjuster writes to that servo
  immediately (moves only if that servo's torque is enabled — a natural interlock).
  The per-row **Sync** checkbox arms a joint for the group; the master **Sync Write**
  button then sends every armed joint's target in coordinated sync packets.
- **Tab name: Joint Control.**
- **Live present position shown per row.**

## Key constraint: SYNC WRITE cannot mix series in one packet

A SYNC WRITE is one broadcast to many IDs, all written at the **same register with
an identical payload**. The payload differs by series:

- HLS: 7 bytes at addr 41, torque at bytes 44/45.
- STS/SMS: 7 bytes at addr 41, **zeros** at 44/45 (writing HLS torque bytes here hits
  Goal Time / PWM — the exact divergence the HLS fix corrected).
- SCS: different address (42), different layout, 0–1023 range.

Therefore a mixed-series group cannot share one packet. **The Sync Write buckets the
armed joints by series and emits one `sync_write_pos_ex` per series present.** For an
all-HLS robot this is exactly one packet — truly simultaneous. Mixed types send
back-to-back packets (simultaneous within each series). This limitation is documented
in the UI status line when a mixed group is synced.

## Architecture

### Shared dispatch refactor (serves both tabs)

Per-series dispatch is currently duplicated inline (torque enable in
`onTorqueEnableCheckBoxStateChanged`, single write in `writeGoal`, sync write in
`syncWritePos`). Extract three shared units so the new tab and the Debug tab use one
tested path. They are `MainWindow` methods (they need the `scs_serial_` /
`sms_sts_serial_` / `hls_serial_` members), and `JointControlTab` calls them through a
back-pointer or Qt signals — see Components:

- `void enableTorqueFor(uint8_t id, const ServoProfile &p, bool on)` — per-series torque write.
- `int writeGoalFor(uint8_t id, const ServoProfile &p, int pos, int speed, int acc, int torque)` — per-series single position write.
- `void syncWriteGroup(const std::vector<GroupTarget> &targets, int speed, int acc, int torque)` — buckets by series, emits one sync packet per series. `GroupTarget{uint8_t id; ServoProfile profile; int pos;}`.

### Central discovered-servo store

Replace the loose `std::vector<uint8_t> id_list_` plus row-stashed `ServoProfile`
(currently on `ServoListView` rows via `Qt::UserRole+1..3`) with:

```cpp
struct DiscoveredServo { uint8_t id; feetech_servo::ServoProfile profile; };
std::vector<DiscoveredServo> discovered_;
```

The left-panel Search populates `discovered_`; both the Debug tab's servo list and the
Joint Control tab render from it. The Debug tab's existing selection and control paths
reroute through `discovered_` and the shared helpers. `id_list_` is removed.

### Components

**`JointRow` (new widget, one per servo)** — a self-contained composite:

```
[ID] [name] [Torque ☐] [====slider====] [target ▢] [pos: NNNN] [Sync ☐]
```

- Torque checkbox toggled → `enableTorqueFor(id, profile, checked)` immediately.
- Slider / target spinbox changed by the user → live jog: `writeGoalFor(id, profile,
  target, tabSpeed, tabAcc, tabTorque)`. Slider and spinbox stay synchronized.
- Slider range by series: SCS `0–1023`, STS/SMS/HLS `0–4095`.
- Present-position label updated by the tab's poll timer.
- Sync checkbox: pure UI state (armed/not); read by the master Sync Write.
- A fail-closed (`profile.known == false`) servo renders its controls disabled, matching
  the Debug tab's behaviour.

`JointRow` exposes: `id()`, `profile()`, `target()`, `isSyncArmed()`,
`setPresentPosition(int)`, `setTorque(bool)`, and signals `torqueToggled(bool)` /
`jogged(int target)`.

**`JointControlTab` master bar (top of tab):**

```
[Torque OFF All] [Torque ON All] [Select-all Sync] [Sync Write]   speed[__] acc[__] torque[__]
```

- **Torque OFF All** — soft e-stop; calls `enableTorqueFor(..., false)` for every row.
  Always enabled whenever the port is open.
- **Torque ON All** — enables torque on every known servo.
- **Select-all Sync** — toggles every row's Sync checkbox.
- **Sync Write** — collects armed rows' `GroupTarget`s, calls `syncWriteGroup`.
- **speed / acc / torque** — shared move parameters applied to every jog and sync write
  on this tab (per-row would be 20× clutter). `torque` applies to HLS joints only;
  ignored by STS/SCS.

### Layout

The tab is a `QScrollArea` wrapping a `QVBoxLayout` of `JointRow`s (needed for ~20
servos), with the master bar fixed above it. Consistent with the resizable-window work
already merged.

## Data flow

1. Search on the left panel fills `discovered_`.
2. On search completion (or a Refresh action), `JointControlTab` rebuilds its rows from
   `discovered_`.
3. Torque toggle → immediate per-servo write.
4. Adjuster drag → immediate per-servo jog write (if torque on).
5. Sync Write → bucket armed rows by series → one `sync_write_pos_ex` per series.
6. Poll timer (only while this tab is active) updates present-position labels.

## Live readout & bus load

Present position is polled only when the Joint Control tab is the active tab, and
**round-robin — a bounded number of servos per timer tick** — so 20 servos do not
saturate the 1 Mbaud bus. The round-robin bound is a named constant. (A future
optimization is protocol SYNC READ; out of scope here.)

## Safety

- Torque defaults **off** — servos appear as discovered; nothing is energized by opening
  the tab.
- A jog cannot move a torque-off joint (the servo ignores a goal with torque disabled),
  so enabling torque is a deliberate first step.
- **Sync Write is only ever explicit** — never fires on a drag.
- **Torque OFF All** is the panic button, always reachable.
- Before any hardware test, invoke `roboforge:safe-hardware-deployment`. First group
  moves use a conservative shared speed/torque, servos unloaded and clear of limits.

## Error handling

- A write to a servo that stops responding is logged and skipped; the tab stays usable.
- Sync Write with zero armed joints is a no-op with a status-line message.
- Mixed-series sync writes emit one packet per series and note in the status line that
  cross-series simultaneity is best-effort.
- Unknown-series (fail-closed) servos show disabled row controls.

## Scope

In scope: the tab, the three shared dispatch helpers, the `discovered_` store, the
`JointRow` widget, master controls, live per-row readout, packet-level tests.

Out of scope: protocol SYNC READ, saving/loading joint poses to file, trajectory
playback, per-row speed/acc/torque, multi-turn positions beyond the single-rev range.
These are deliberately deferred.

## Testing

Packet-level unit tests using the existing `CapturingSerial` harness
(`tests/test_packets.cpp`):

1. `syncWriteGroup` on an all-HLS group emits exactly one sync packet, torque at 44/45.
2. `syncWriteGroup` on a mixed HLS+STS group emits one packet per series, each with the
   correct payload (HLS carries torque, STS carries zeros at 44/45).
3. `writeGoalFor` dispatches to the correct control class per series (SCS/STS/HLS).
4. `enableTorqueFor` writes torque-enable to the correct register per series.

UI wiring, live readout, and multi-joint motion are verified on hardware (the 3 HLS
servos on the bench), gated by `roboforge:safe-hardware-deployment`.

## Files touched

| File | Change |
|---|---|
| `mainwindow.h` / `.cpp` | `enableTorqueFor`, `writeGoalFor`, `syncWriteGroup` as MainWindow methods; `discovered_` store; reroute Debug tab through them; remove `id_list_` |
| `jointrow.h` / `jointrow.cpp` | **New** — per-servo row widget |
| `jointcontroltab.h` / `jointcontroltab.cpp` | **New** — tab widget: master bar, scroll list, poll timer |
| `mainwindow.ui` | Add the Joint Control tab to `tabWidget` (after Programming) |
| `mainwindow.h` / `.cpp` | Own the tab; feed it `discovered_`; rebuild rows on search completion |
| `FT_SCServo_Debug_Qt.pro` | Add new sources/headers |
| `tests/test_packets.cpp` | The four dispatch/sync tests above |

## Reference: existing hooks this builds on

- `feetech_servo::ServoProfile` (`servo/scserial.h`) — `{name, series, end, known}`.
- `resolveServo(model, firmware)` — already used by Search.
- `HLSCL/SMS_STS/SCSCL::sync_write_pos_ex` / `enable_torque` — per-series primitives.
- Search loop at `mainwindow.cpp:onSearchTimerTimeout` populates discovered servos.
- `writeGoal` / `syncWritePos` (`mainwindow.cpp`) — the dispatch shapes being extracted.
