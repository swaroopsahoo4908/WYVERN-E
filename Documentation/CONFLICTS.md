# WYVERN-E 4.0 — Design-Conflict Memo & Frozen Firmware Parameters

This firmware was written against the project's design files as they exist today. Two design
conflicts are recorded below: the PID-gain supersession (§1) and the recovery-architecture change
to F15-4 motor ejection (§2). Both are resolved in favor of the current/validated source, and the
firmware implements the resolution. The former third item (RRC3+ telemetry byte format) is retired:
the RRC3+ has been removed from the vehicle (§3).

## 1. PID gains: flowchart vs. validated header/sim — SUPERSEDED by margin-analysis retune (2026)

**This section has been updated. The gains below are frozen as of the phase-margin retune; do not
read `Kp=2.0/Ki=0.4/Kd=0.5` anywhere in this repo as current — it is a documented-superseded value.**

- `Flight Computer/flowcharts/02_tvc_control_loop.mermaid` lists **Kp=8 Ki=1.5 Kd=1.2** (stale from
  the original design pass, never simulated, flagged unstable by the header's own original comment).
- The gains that were live in `wyvern_pid.h`/`we4_atmos_tvc.py` through the prior audit round —
  **Kp=2.0 Ki=0.4 Kd=0.5** — were *themselves* subsequently found to be unstable once evaluated
  rigorously: a phase/gain-margin sweep across 24 operating points (4 atmospheres — ISA T_sl=288.15K,
  cold=258.15K, hot=313.15K, high-DA=298.15K — × 6 burn-time slices at 0.6/1.0/1.7/2.5/2.9/3.4 s
  into the 3.45 s burn) with the servo modeled as `TAU_SERVO=0.04 s` plus a ~2 ms Padé-2 transport
  delay found a **worst-case phase margin of −6.2° and gain margin of −2.0 dB** against a 30° PM
  target — i.e. genuinely unstable at some points in the envelope, not merely under-margined.
- A re-tune against the same 24-point sweep (see `Documentation/PID_TUNING_REPORT.md` for the full
  margin tables and `Simulations/we4_pid_retune.py` for the search) found
  **Kp=0.10, Ki=0.40, Kd=0.18** achieves **PM=33.1°, GM=9.3 dB at every one of the 24 points** —
  clearing the 30° target with margin to spare across the full atmosphere/burn-time envelope.
- **Resolution: firmware (`wyvern_pid.h`) now uses Kp=0.10, Ki=0.40, Kd=0.18.** The Kp=2.0/Ki=0.4/
  Kd=0.5 gains recorded earlier in this memo (and still in `we4_atmos_tvc.py`'s in-file defaults,
  which predate the margin sweep and should be updated to match) are superseded, not current. The
  flowchart remains stale against *both* historical gain sets and should be regenerated from this
  table, not from either prior value.

## 2. Recovery architecture: motor ejection via bypass tube — RESOLVED (design change)

- Recovery was previously debated between a timer-forced electronic deploy (finless-era) and an
  apogee-primary RRC3+ dual-deploy (finned-era). **Both are now obsolete.** The current vehicle uses
  the **F15-4 motor's own ejection charge**, fired 4 s after burnout (t ≈ 7.45 s, 0.66 s past
  apogee), routed through a solid-walled bypass tube past the sealed FC bay into the recovery bay to
  release the friction-fit nose (see `WYVERN_E4_Recovery.md` and `Simulations/we4_ejection_feasibility.py`).
- This **eliminates** the RRC3+, the isolated 9 V recovery battery, the e-match/black-powder charge,
  and the earlier CO2 solenoid system entirely. The finned airframe (4×72 mm, +1.0 cal, 705 g
  liftoff) is stable to apogee, so a single passive event just past apogee is appropriate.
- **Resolution: a non-issue for the Pico firmware**, because *the flight computer never drives
  recovery* — the motor does. The Pico only **observes**: it logs baro/IMU for apogee/landing and
  streams WiFi telemetry. All prior CO2/RRC3 deploy logic has been removed from the firmware
  (`co2_deploy.h` and `rrc3_telemetry.h` are now deprecation stubs). The mermaid recovery flowchart
  has been regenerated from the verified Recovery doc.

## 3. (removed) RRC3+ Comm-Port byte format

The RRC3+ altimeter has been removed from the vehicle (motor-ejection recovery), so its serial
byte-format documentation gap no longer applies. `rrc3_telemetry.h` is a deprecation stub and is
not included by the flight sketch.

## 4. Power architecture: light 2S LiPo → 5 V UBEC + battery-voltage ADC sense

The vehicle runs the entire avionics domain off a **light 2S LiPo (7.4 V, ~450 mAh; Zeee 4-pk)**
feeding a single **5 V/6 V UBEC set to 5 V**. That one rail powers Pico 2 W VSYS (1.8–5.5 V range),
the camera, and both servos — the servos run fine at 5 V (~1.8 kg·cm, >2× the ~0.9 kg·cm demand), so
a separate 6 V servo BEC is not needed at this scale. Because the servos and the Pico share the 5 V
rail, add decoupling — **1000 µF** low-ESR bulk cap at the servos, **100 µF** at VSYS, and an **SS34
Schottky** from rail → VSYS as a hold-up diode — and run the servo feed and the VSYS feed as separate
star runs off the UBEC output so servo-stall transients (~1 A each) can't brown-out the Pico.

Battery-voltage sense is on **GP26 (ADC0)**, tapping the LiPo pack *before* the BEC:

- **R_top = 100 kΩ** (VBAT → ADC node), **R_bot = 62 kΩ** (ADC node → GND)
- Divider ratio = 62/(100+62) = 0.3827. At 2S full charge (8.4 V) the ADC sees ≈ 3.21 V (just under
  the 3.3 V reference); at the 6.0 V cutoff (3.0 V/cell) it sees ≈ 2.30 V.
- Firmware (`battery.h`) un-does the divider in software (`V_BAT = V_ADC / 0.3827`) and flags
  low-battery below **6.4 V (3.2 V/cell)**, inhibiting arming below the **6.0 V (3.0 V/cell)** hard cutoff.
- **GP28 (ADC2)** is reserved as a spare analog input (e.g., a servo-current shunt) but is not wired
  by default.

This divider is a one-resistor-pair addition to the FC-bay harness; it does not touch the servo
rail or any existing net (there is no recovery power domain — recovery is the motor's own charge).

## 5. Frozen parameter table (firmware is written against these values — single source of truth)

| Parameter | Value | Source |
|---|---|---|
| MCU / board | Raspberry Pi Pico 2 W (RP2350), Arduino-Pico (earlephilhower) core | 01_FlightComputer_Spec.md |
| Control loop rate | 500 Hz (dt = 2.0 ms) on core 0 | 01_FlightComputer_Spec.md, flowcharts/02 |
| TVC engage delay | t ≥ 0.5 s after launch detect (past F15 ignition spike) | we4_atmos_tvc.py |
| Burnout / TVC cutoff | t = 3.45 s | we4_flightsim.py, we4_atmos_tvc.py |
| PID gains (pitch = yaw, decoupled) | Kp=0.10, Ki=0.40, Kd=0.18 | wyvern_pid.h; margin-verified 24-point sweep, PID_TUNING_REPORT.md (PM=33.1°, GM=9.3 dB worst case) |
| Derivative filter time constant | tau_d = 0.02 s | wyvern_pid.h, pid_reference.py |
| Integral clamp | ±0.4 (anti-windup) | wyvern_pid.h, pid_reference.py |
| Output (gimbal) limit | ±8.0° (0.1396 rad) | wyvern_pid.h `OUT_LIM_DEG=8.0` (raised 5→8 for wind/weathercock authority) |
| Servo lag (model) | tau_servo ≈ 0.04 s | we4_atmos_tvc.py |
| Launch detect | \|a\| > 3 g sustained ≥ 50 ms, body BNO085 accelerometer report | flowcharts/01_flight_state_machine.mermaid |
| I2C0 (mux trunk) | GP16 SDA / GP17 SCL | wyvern4_tvc.ino, gen_wiring4.py |
| I2C1 (gimbal BNO085, dedicated) | GP18 SDA / GP19 SCL | wyvern4_tvc.ino, gen_wiring4.py |
| PCA9548A mux address | 0x70; ch0 body BNO085, ch1 recovery BNO085, ch2 BME688 (0x76), ch3 BMP388 (0x77, Adafruit 3966), ch4 spare | wyvern4_tvc.ino, gen_wiring4.py |
| Gimbal BNO085 address | 0x4A on I2C1 | wyvern4_tvc.ino |
| SPI0 (microSD) | SCK GP2 / MOSI GP3 / MISO GP4 / CS GP5 | wyvern4_tvc.ino, gen_wiring4.py |
| Servos | GP14 pitch (PWM), GP15 yaw (PWM), neutral=90°, travel ±8° | wyvern4_tvc.ino, t3_servo_sweep.ino |
| GP1 (UART0 RX) | spare (formerly RRC3+ telemetry tap; RRC3+ removed — motor-ejection recovery) | — |
| GP6 | spare (formerly RRC_ARM sense; RRC3+ removed — motor-ejection recovery) | — |
| LAUNCH_IRQ | GP7 (reserved; optional redundant mechanical inertial-switch input, OR'd into the software launch latch) | wyvern4_tvc.ino (assumption — undocumented elsewhere) |
| CAM_EN | GP8 (drives action-camera power gate per Camera_Solution.md) | wyvern4_tvc.ino |
| LED / Buzzer | GP9 / GP10 | wyvern4_tvc.ino |
| RBF (remove-before-flight) sense | GP22 | wyvern4_tvc.ino |
| Battery ADC (NEW — gap filled, §4) | GP26 (ADC0), 100k/62k divider, V_BAT = V_ADC/0.3827 | this memo |
| WiFi/BLE | Onboard CYW43439, bench-only UDP broadcast, never blocks core 0 | 01_FlightComputer_Spec.md |
| Recovery sequencing | F15-4 motor ejection charge via bypass tube, t ≈ 7.45 s (0.66 s past apogee); no electronic deploy; FC only observes/logs | WYVERN_E4_Recovery.md |

All seven firmware modules below are written to this table. If the bench reveals a different real
pin/address (e.g., the LAUNCH_IRQ assumption above), update this table and the `#define`s in
`wyvern4_tvc.ino` together — they're meant to be the same source of truth.

## 6. New from the full component compatibility audit — three items NOT resolved by this memo

`Documentation/COMPATIBILITY.md` is the full I2C/SPI/PWM/ADC/power audit across every component and
the ground-test rigs. Everything in sections 1–5 above remains true and unaffected. Three findings
from that audit are genuine open conflicts, called out here rather than silently fixed, per the same
policy this memo already follows for §1–3:

1. **VL53L4CD ToF sensors — REMOVED from the design (2026-07).** The ToF ranging sensors have been
   deleted from the BOM and the design entirely (flight and ground rig). Gimbal deflection on the
   solenoid balance is taken from the 3-axis load balance plus the gimbal BNO085, so no ToF ring,
   driver, XSHUT sequencing, or extra mux channel is needed. Mux ch4 is simply a spare, unpopulated.
   (The solenoid-rig sketch's legacy `tof_ring.h` module is no longer wired in.)
2. **BSS138 level shifter is orphaned.** It appears in the BOM but is not routed in any schematic,
   wiring generator, or firmware comment. Either every net that needs it is already 3.3V-native (in
   which case it should be removed from the BOM) or a real level-shifted net was designed but never
   wired (in which case identify which net and route it). Unresolved pending a BOM/schematic review.
3. **Ground-rig DAQ MCU: BOM says Arduino Nano/Teensy, deliverable spec says Raspberry Pi Pico.**
   Every wiring diagram and design document that predates this audit round specifies a Nano-class
   board for the ground-test load-cell/IMU DAQ. The two ground-test rig sketches delivered in this
   round (`wyvern4_gse_servo_rig.ino`, `wyvern4_gse_solenoid_rig.ino`) target the **Pico** per the
   current task requirements, and `GSE_TestStands.md` / `gen_wiring4.py` have been updated to match
   the Pico pinout. This is flagged, not silently resolved: if a Nano/Teensy is what's actually on
   the bench, the ground-rig sketches need porting (bit-banged HX711 timing and pin numbers are
   MCU-specific) before they'll run as-is. Treat "Pico" as the resolution going forward for any new
   ground-rig work, and update or retire the older Nano-based wiring references accordingly.
