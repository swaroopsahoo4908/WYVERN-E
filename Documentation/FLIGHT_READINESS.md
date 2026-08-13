# GTR70E WYVERN Flight Computer, Flight-Readiness Document

**Authors:** Swaroop Sahoo, Chris Liu, Allison Hong  
**Date:** 2026-08-12  
**Program:** GTR70E WYVERN


**⚠ Open item (2026-08-09):** flight hardware is now a custom RP2350B PCB with one external
STEMMA-QT port (one external BNO085 at the TVC-bay/electronics boundary, see
`WYVERN_E4_Recovery.md` §1), not the Pico 2 W + tri-IMU setup described below. The state machine,
recovery, and power sections remain accurate; the IMU-count/mux language is stale pending a
reconciliation pass against the real board.

This is the top-level readiness summary for the Pico 2 W flight-computer firmware. Read
`CONFLICTS.md` first, it's the frozen parameter table and the record of the resolved design
conflicts (including the PID gain retune, §1) that this firmware's behavior depends on. Also read
`COMPATIBILITY.md` for the full I2C/SPI/PWM/ADC/power audit across every component, it surfaces
three unresolved hardware conflicts (§4 below) that are not fixable from firmware alone. This
document covers: what was built, what must be bench-verified before the firmware can be trusted in
the air, and the go/no-go sequence.

## 1. What this firmware does

- **Core 0 (500 Hz, real-time, never blocks):** reads the tri-IMU GRV attitude (gimbal on a
  dedicated I2C1 bus; body + recovery behind the PCA9548A mux on I2C0, 2-of-3-reduced-to-2-of-2
  voting between body/recovery), runs the dual-axis PID (`wyvern_pid.h`), commands the pitch/yaw
  TVC servos, runs launch-detect and the BOOT→ARMED→BOOST→COAST→RECOVER→DESCENT→LANDED state
  machine, and pushes one log frame per tick into a lock-free inter-core FIFO.
- **Core 1 (housekeeping, may block):** drains that FIFO to a microSD flight log (`sd_logger.h`),
  monitors the flight battery over a new ADC divider (`battery.h`), services the camera gate /
  status LED+buzzer (`launch_status.h`), and optionally broadcasts a ~20 Hz UDP bench-telemetry feed
  over the onboard CYW43439 (`wifi_telemetry.h`, disabled by default, `WIFI_ENABLED 0` in
  `wyvern4_tvc.ino`).
- **The Pico never drives recovery.** Recovery is the **F15-4 motor's own ejection charge**, fired
  4 s after burnout (t ≈ 7.45 s, 1.18 s past apogee), pressurizing the Lower BT and **separating the
  two body tubes at the bulkhead joint** (the airframe is now two body tubes, Lower BT/Upper BT, 
  joined at one bulkhead, not a single continuous tube). There is no pyro, e-match, CO2, or recovery
  computer of any kind in the vehicle, the FC only logs baro/IMU and streams WiFi telemetry. See
  `WYVERN_E4_Recovery.md` and the feasibility study (`Simulations/we4_ejection_feasibility.py`).

## 2. Design conflicts resolved (full detail in `CONFLICTS.md`)

1. **PID gains**: Kp=0.10/Ki=0.40/Kd=0.18, margin-verified across a 24-point phase/gain-margin sweep
   (4 atmospheres × 6 burn-time slices), worst case PM=44.7°, GM=12.6 dB against a 30° target — see
   `PID_TUNING_REPORT.md` for the full sweep and `CONFLICTS.md` §1 for the parameter table.
2. **Recovery architecture**, recovery is now the F15-4 motor ejection charge separating the two
   body tubes at the bulkhead joint (no RRC3+, no pyro, no CO2). **Moot for the FC** since it never
   drives recovery, the flight computer only observes. Prior CO2/RRC3 deploy logic has been removed
   from the firmware.
3. **Battery ADC**, the 2S LiPo pack (before the BEC) is sensed on GP26 (ADC0) through a 100 kΩ/62 kΩ
   divider (keeps 2S full-charge 8.4 V at ~3.21 V, under the 3.3 V ADC ref), one resistor-pair harness
   addition, and the reading is now also carried across the core boundary into every flight-log row
   (`batt_v` in the schema-v2 `LogFrame`, see §5). Firmware warns at 6.4 V and inhibits arming below 6.0 V.

## 3. Hardware items flagged by `COMPATIBILITY.md`, do not fly past these silently

1. **Gimbal deflection is measured by the 3-axis load balance plus the gimbal BNO085.** There's no
   ranging hardware anywhere on the vehicle or either ground rig; mux ch4 is a spare.
2. **BSS138 level shifter is orphaned** in the BOM, not routed on any schematic or firmware net.
   Resolve by either removing it from the BOM or identifying and wiring the net that needs it.
3. **Ground-rig DAQ is the Raspberry Pi Pico** on both `wyvern4_gse_servo_rig.ino` and
   `wyvern4_gse_solenoid_rig.ino`, matching `GSE_TestStands.md` and `gen_wiring4.py`.

## 4. Action items before this flies, do not skip these

| # | Item | Why it matters | How to clear it |
|---|---|---|---|
| 1 | **Ground-test the bulkhead separation joint** | Recovery now depends entirely on the F15-4 charge pressurizing the Lower BT to separate the two body tubes at the bulkhead joint, this is the single point of the recovery system | Do the bench ground-separation test in `WYVERN_E4_Recovery.md` §8: fire a representative charge (or the motor's own charge in a restrained static test) and confirm the bulkhead joint releases cleanly in the 50-150 N band, the chute deploys, and the servo/STEMMA-QT cable pass-through survives the separation event. |
| 3 | **Confirm LAUNCH_IRQ wiring (GP7)** | The hardware inertial-switch backup to the software 3g/50 ms launch latch is an *assumption*, no design doc specifies this pin | Either wire the redundant mechanical switch to GP7 (active-low, per `launch_status.h`), or remove the IRQ branch from `LaunchDetect::update()` if no such switch exists in this build. Flying with an undocumented floating input is worse than removing the dead code path. |
| 4 | **Confirm RBF sense polarity** | `wyvern4_tvc.ino` assumes RBF pulled = HIGH (pull-up, switch open); this is a documented assumption, not measured. GP6/GP1 (formerly RRC_ARM / RRC3 telemetry) are now spare | With the RBF switch in each position, read `HB:...rbf=` over serial via `host_monitor.py` and confirm it matches "pulled" when you expect it to. |
| 5 | **Verify the battery divider + shared-rail decoupling in hardware** | GP26/100k-62k is a firmware-side allocation; the resistors, 1000 µF servo bulk cap, 100 µF VSYS cap and SS34 hold-up Schottky are harness additions that don't exist on any current board yet | Install the divider + decoupling, then with a known bench voltage on the 2S input confirm `host_monitor.py`'s `batt=` reading agrees with a multimeter within ~2%, and scope VSYS during a servo stall to confirm it stays above the Pico brown-out threshold. |
| 6 | **Confirm SH2_ACCELEROMETER support on your BNO085 firmware revision** | Launch-detect and landing-detect both depend on `imu_grv.h` enabling a live accelerometer report on the body IMU, on top of GRV | During self-test, watch for `accel_mag_g` settling near 1.0 at rest (visible in the FIFO/log, not currently in a SELFTEST line), if it stays at the 1.0 g software default forever, the accel report failed to enable and launch/landing detect are running on a frozen fallback. Add a bench print if you want this surfaced explicitly before flight. |
| 7 | **Servo throw and gimbal mechanical limit** | Firmware clamps to ±8° in software; confirm the printed gimbal + servo linkage physically allow ±8° travel (raised from ±5° for wind authority) with no binding | During the SERVO self-test sweep, visually confirm no binding/buzzing at the endpoints. |

## 5. Preflight bench sequence (ground test, every time before flight)

1. Power the Pico from USB (or flight battery + a serial/UDP monitor) with the vehicle **horizontal
   and restrained**, the servo sweep and IMU motion during self-test are expected, not a fault.
2. Run `python3 test_code/selftest.py /dev/tty.usbmodemXXXX` (macOS) or `.../ttyACM0` (Linux).
3. Confirm **every** row in the printed table reads `PASS` (or `SKIP` for WIFI if disabled, or
   WAIT on RBF is correct if the switch is still inserted).
4. Pull the RBF switch; confirm the table/heartbeat shows `rbf_pulled=True` and the status LED/
   buzzer switches to the ARMED pattern (`launch_status.h`).
5. Confirm `SELFTEST:DONE:PASS` and `>>> PREFLIGHT GO <<<` from `selftest.py`'s own exit message.
6. Re-insert the RBF, disconnect the bench monitor, and proceed to the pad per the normal rocketry
   range-safety procedure, RBF removal and arming should be one of the last actions at the pad,
   matching its name.
7. **If any row reads `FAIL` or `NOT SEEN`, do not fly.** `selftest.py` exits non-zero in both
   cases (suitable for scripting into a larger ground-station go/no-go gate); re-run after fixing
   the indicated subsystem.

## 6. Firmware file map

The flight firmware is now a proper Arduino IDE sketch folder, `gtr70e_wyvern_tvc/` (folder name matches
the `.ino` filename, as the IDE requires), containing the main sketch plus every header as a tab in
the same folder. Open `gtr70e_wyvern_tvc/wyvern4_tvc.ino` in the IDE and every file below loads as a tab.

| File | Role |
|---|---|
| `gtr70e_wyvern_tvc/wyvern4_tvc.ino` | Main sketch: pin map, dual-core ownership, state machine, setup/loop |
| `gtr70e_wyvern_tvc/wyvern_pid.h` | Dual-axis PID: anti-windup, filtered derivative, slew limit, bumpless reset. Frozen gains: Kp=0.10/Ki=0.40/Kd=0.18 (margin-verified retune, §2 item 1) |
| `gtr70e_wyvern_tvc/i2c_mux.h` | PCA9548A driver: channel select/cache, bus recovery |
| `gtr70e_wyvern_tvc/imu_grv.h` | Tri-IMU GRV driver, quaternion math, 2-of-2 voting, body accelerometer |
| `gtr70e_wyvern_tvc/baro.h` | BME680 + BMP388 (Adafruit 3966) combined driver, ground-datum altitude |
| `gtr70e_wyvern_tvc/battery.h` | 2S LiPo ADC monitor (GP26 100k/62k divider; 6.4/6.0 V cutoffs); voltage now also snapshotted cross-core into every log row |
| `gtr70e_wyvern_tvc/launch_status.h` | Launch-detect, camera gate, status LED/buzzer patterns |
| `gtr70e_wyvern_tvc/sd_logger.h` | **Schema v2**: 37-field `LogFrame` (up from 19) + inter-core FIFO + microSD flight logger. Adds flight time, loop-timing jitter, IMU vote disagreement, commanded setpoint, per-axis P/I/D term breakdown, battery flags/voltage, and cumulative dropped-frame count, see the header's schema-v2 comment for the full rationale |
| `gtr70e_wyvern_tvc/rrc3_telemetry.h` | **Deprecated stub**, RRC3+ removed (motor-ejection recovery); no longer included by the sketch |
| `gtr70e_wyvern_tvc/wifi_telemetry.h` | Optional CYW43439 UDP bench telemetry broadcaster |
| `test_code/host_monitor.py` | Parses the live serial protocol, tabulates self-test + heartbeat |
| `test_code/selftest.py` | Go/no-go wrapper around `host_monitor.py` for the bench sequence above |
| `docs/CONFLICTS.md` | Design-conflict memo + frozen parameter table (read this first) |
| `docs/COMPATIBILITY.md` | Full I2C/SPI/PWM/ADC/power compatibility audit across every component and both ground rigs |

## 7. Known limitations / honest caveats

- Recovery is a **single passive event**, the F15-4 motor's own ejection charge, now separating the
  two body tubes at the bulkhead joint rather than popping a nose off one continuous tube. There is
  no electronic deploy path and no backup channel; the ground-separation test (action item #1) is
  the primary way to build confidence before flight. The bulkhead joint's release-force spec and the
  bay-pressurization margin against it both need re-verification for the new two-BT geometry, see
  `WYVERN_E4_Recovery.md` §4, §6.
- Launch detect's hardware-IRQ branch (GP7) and the camera/CAM_EN polarity are implemented per the
  design docs' stated intent but are unconfirmed against an actual wired harness (action item #3).
- The WiFi telemetry path is bench-only by explicit design (fire-and-forget UDP, no flight-critical
  dependency), it is not, and should not become, part of any go/no-go criterion.
- 2-of-3 IMU voting is, in this hardware configuration, actually 2-of-2 (body vs. recovery) for
  vehicle attitude, since the gimbal IMU measures a different physical quantity (nozzle attitude)
  and can't be voted against the other two, see `imu_grv.h`'s header comment for the reasoning.
