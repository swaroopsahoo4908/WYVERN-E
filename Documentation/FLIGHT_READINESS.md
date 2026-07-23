# WYVERN-E 4.0 Flight Computer — Flight-Readiness Document

This is the top-level readiness summary for the Pico 2 W flight-computer firmware. Read
`CONFLICTS.md` first — it's the frozen parameter table and the record of the resolved design
conflicts (including the PID gain retune, §1) that this firmware's behavior depends on. Also read
`COMPATIBILITY.md` for the full I2C/SPI/PWM/ADC/power audit across every component — it surfaces
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
  over the onboard CYW43439 (`wifi_telemetry.h`, disabled by default — `WIFI_ENABLED 0` in
  `wyvern4_tvc.ino`).
- **The Pico never drives recovery.** Recovery is the **F15-4 motor's own ejection charge**, fired
  4 s after burnout (t ≈ 7.45 s, 0.66 s past apogee) and routed through a solid-walled bypass tube
  past the sealed FC bay into the recovery bay to release the nose. There is no pyro, e-match, CO2,
  or recovery computer of any kind in the vehicle — the FC only logs baro/IMU and streams WiFi
  telemetry. See `WYVERN_E4_Recovery.md` and the feasibility study (`Simulations/we4_ejection_feasibility.py`).

## 2. Design conflicts resolved (full detail in `CONFLICTS.md`)

1. **PID gains** — flowchart said Kp=8/Ki=1.5/Kd=1.2 (stale, flagged unstable against servo lag by
   the original header comment); a prior audit round settled on Kp=2.0/Ki=0.4/Kd=0.5 from a
   closed-loop atmosphere sweep, but that gain set was later found to be **unstable** under a
   rigorous phase/gain-margin analysis across 24 operating points (worst case PM=−6.2°, GM=−2.0 dB
   against a 30° target). **Firmware now uses the margin-verified retune: Kp=0.10/Ki=0.40/Kd=0.18**
   (PM=33.1°, GM=9.3 dB worst case across all 24 points) — see `PID_TUNING_REPORT.md` for the full
   sweep and `CONFLICTS.md` §1 for the supersession record.
2. **Recovery architecture** — recovery is now the F15-4 motor ejection charge via a bypass tube
   (no RRC3+, no pyro, no CO2). **Moot for the FC** since it never drives recovery — the flight
   computer only observes. Prior CO2/RRC3 deploy logic has been removed from the firmware.
3. **Battery ADC** — the 2S LiPo pack (before the BEC) is sensed on GP26 (ADC0) through a 100 kΩ/62 kΩ
   divider (keeps 2S full-charge 8.4 V at ~3.21 V, under the 3.3 V ADC ref), one resistor-pair harness
   addition, and the reading is now also carried across the core boundary into every flight-log row
   (`batt_v` in the schema-v2 `LogFrame` — see §5). Firmware warns at 6.4 V and inhibits arming below 6.0 V.

## 3. Hardware conflicts flagged by `COMPATIBILITY.md` — NOT resolved by firmware, do not fly past these silently

1. **VL53L4CD ToF sensors — REMOVED (2026-07).** Deleted from the BOM and design entirely; gimbal
   deflection is measured by the 3-axis load balance + gimbal BNO085, so no ToF driver, XSHUT plan,
   or extra mux channel is needed. Mux ch4 is a spare. Not a flight-readiness item any longer.
2. **BSS138 level shifter is orphaned** in the BOM — not routed on any schematic or firmware net.
   Resolve by either removing it from the BOM or identifying and wiring the net that needs it.
3. **Ground-rig DAQ MCU conflict: BOM/older wiring says Nano/Teensy, this deliverable targets Pico.**
   Both ground-test sketches in this round (§ below) are written for the Raspberry Pi Pico per the
   current task scope, and `GSE_TestStands.md`/`gen_wiring4.py` have been updated to match. If the
   bench hardware is actually a Nano/Teensy, the bit-banged HX711 timing and pin numbers in those
   sketches need porting before use — see `CONFLICTS.md` §6 for the full record.

## 4. Action items before this flies — do not skip these

| # | Item | Why it matters | How to clear it |
|---|---|---|---|
| 1 | **Ground-test the ejection gas path** | Recovery now depends entirely on the F15-4 charge pressurizing the recovery bay through the bypass tube to release the nose — this is the single point of the recovery system | Do the bench ground-ejection test in `WYVERN_E4_Recovery.md`: fire a representative charge (or the motor's own charge in a restrained static test) and confirm the friction-fit nose releases cleanly and the chute deploys. Confirm the bypass-tube joints and both bulkhead seals are gas-tight. |
| 3 | **Confirm LAUNCH_IRQ wiring (GP7)** | The hardware inertial-switch backup to the software 3g/50 ms launch latch is an *assumption* — no design doc specifies this pin | Either wire the redundant mechanical switch to GP7 (active-low, per `launch_status.h`), or remove the IRQ branch from `LaunchDetect::update()` if no such switch exists in this build. Flying with an undocumented floating input is worse than removing the dead code path. |
| 4 | **Confirm RBF sense polarity** | `wyvern4_tvc.ino` assumes RBF pulled = HIGH (pull-up, switch open); this is a documented assumption, not measured. GP6/GP1 (formerly RRC_ARM / RRC3 telemetry) are now spare | With the RBF switch in each position, read `HB:...rbf=` over serial via `host_monitor.py` and confirm it matches "pulled" when you expect it to. |
| 5 | **Verify the battery divider + shared-rail decoupling in hardware** | GP26/100k-62k is a firmware-side allocation; the resistors, 1000 µF servo bulk cap, 100 µF VSYS cap and SS34 hold-up Schottky are harness additions that don't exist on any current board yet | Install the divider + decoupling, then with a known bench voltage on the 2S input confirm `host_monitor.py`'s `batt=` reading agrees with a multimeter within ~2%, and scope VSYS during a servo stall to confirm it stays above the Pico brown-out threshold. |
| 6 | **Confirm SH2_ACCELEROMETER support on your BNO085 firmware revision** | Launch-detect and landing-detect both depend on `imu_grv.h` enabling a live accelerometer report on the body IMU, on top of GRV | During self-test, watch for `accel_mag_g` settling near 1.0 at rest (visible in the FIFO/log, not currently in a SELFTEST line) — if it stays at the 1.0 g software default forever, the accel report failed to enable and launch/landing detect are running on a frozen fallback. Add a bench print if you want this surfaced explicitly before flight. |
| 7 | **Servo throw and gimbal mechanical limit** | Firmware clamps to ±8° in software; confirm the printed gimbal + servo linkage physically allow ±8° travel (raised from ±5° for wind authority) with no binding | During the SERVO self-test sweep, visually confirm no binding/buzzing at the endpoints. |

## 5. Preflight bench sequence (ground test, every time before flight)

1. Power the Pico from USB (or flight battery + a serial/UDP monitor) with the vehicle **horizontal
   and restrained** — the servo sweep and IMU motion during self-test are expected, not a fault.
2. Run `python3 test_code/selftest.py /dev/tty.usbmodemXXXX` (macOS) or `.../ttyACM0` (Linux).
3. Confirm **every** row in the printed table reads `PASS` (or `SKIP` for WIFI if disabled, or
   WAIT on RBF is correct if the switch is still inserted).
4. Pull the RBF switch; confirm the table/heartbeat shows `rbf_pulled=True` and the status LED/
   buzzer switches to the ARMED pattern (`launch_status.h`).
5. Confirm `SELFTEST:DONE:PASS` and `>>> PREFLIGHT GO <<<` from `selftest.py`'s own exit message.
6. Re-insert the RBF, disconnect the bench monitor, and proceed to the pad per the normal rocketry
   range-safety procedure — RBF removal and arming should be one of the last actions at the pad,
   matching its name.
7. **If any row reads `FAIL` or `NOT SEEN`, do not fly.** `selftest.py` exits non-zero in both
   cases (suitable for scripting into a larger ground-station go/no-go gate); re-run after fixing
   the indicated subsystem.

## 6. Firmware file map

The flight firmware is now a proper Arduino IDE sketch folder — `wyvern4_tvc/` (folder name matches
the `.ino` filename, as the IDE requires), containing the main sketch plus every header as a tab in
the same folder. Open `wyvern4_tvc/wyvern4_tvc.ino` in the IDE and every file below loads as a tab.

| File | Role |
|---|---|
| `wyvern4_tvc/wyvern4_tvc.ino` | Main sketch: pin map, dual-core ownership, state machine, setup/loop |
| `wyvern4_tvc/wyvern_pid.h` | Dual-axis PID: anti-windup, filtered derivative, slew limit, bumpless reset. Frozen gains: Kp=0.10/Ki=0.40/Kd=0.18 (margin-verified retune, §2 item 1) |
| `wyvern4_tvc/i2c_mux.h` | PCA9548A driver: channel select/cache, bus recovery |
| `wyvern4_tvc/imu_grv.h` | Tri-IMU GRV driver, quaternion math, 2-of-2 voting, body accelerometer |
| `wyvern4_tvc/baro.h` | BME688 + BMP388 (Adafruit 3966) combined driver, ground-datum altitude |
| `wyvern4_tvc/battery.h` | 2S LiPo ADC monitor (GP26 100k/62k divider; 6.4/6.0 V cutoffs); voltage now also snapshotted cross-core into every log row |
| `wyvern4_tvc/launch_status.h` | Launch-detect, camera gate, status LED/buzzer patterns |
| `wyvern4_tvc/sd_logger.h` | **Schema v2**: 37-field `LogFrame` (up from 19) + inter-core FIFO + microSD flight logger. Adds flight time, loop-timing jitter, IMU vote disagreement, commanded setpoint, per-axis P/I/D term breakdown, battery flags/voltage, and cumulative dropped-frame count — see the header's schema-v2 comment for the full rationale |
| `wyvern4_tvc/rrc3_telemetry.h` | **Deprecated stub** — RRC3+ removed (motor-ejection recovery); no longer included by the sketch |
| `wyvern4_tvc/wifi_telemetry.h` | Optional CYW43439 UDP bench telemetry broadcaster |
| `test_code/host_monitor.py` | Parses the live serial protocol, tabulates self-test + heartbeat |
| `test_code/selftest.py` | Go/no-go wrapper around `host_monitor.py` for the bench sequence above |
| `docs/CONFLICTS.md` | Design-conflict memo + frozen parameter table (read this first) |
| `docs/COMPATIBILITY.md` | Full I2C/SPI/PWM/ADC/power compatibility audit across every component and both ground rigs |

## 7. Known limitations / honest caveats

- Recovery is a **single passive event** — the F15-4 motor's own ejection charge. There is no
  electronic deploy path and no backup channel; the ground-ejection test (action item #1) is the
  primary way to build confidence before flight. Robustness margin is the 3.4× bay-pressurization
  figure in `WYVERN_E4_Recovery.md`.
- Launch detect's hardware-IRQ branch (GP7) and the camera/CAM_EN polarity are implemented per the
  design docs' stated intent but are unconfirmed against an actual wired harness (action item #3).
- The WiFi telemetry path is bench-only by explicit design (fire-and-forget UDP, no flight-critical
  dependency) — it is not, and should not become, part of any go/no-go criterion.
- 2-of-3 IMU voting is, in this hardware configuration, actually 2-of-2 (body vs. recovery) for
  vehicle attitude, since the gimbal IMU measures a different physical quantity (nozzle attitude)
  and can't be voted against the other two — see `imu_grv.h`'s header comment for the reasoning.
