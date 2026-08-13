# GTR70E WYVERN, Flight Computer

*Single custom PCB1 (bare RP2350B, QFN-80, Ø62 mm board), flight computer and real-time TVC controller.*

See `01_FlightComputer_Spec.md` for the full architecture writeup.

---

## Architecture summary

Dual-core RP2350B split for determinism:

- *Core 0, real-time control.* 500 Hz TVC loop only: read external + body BNO085 (Game Rotation
  Vector), vote attitude, run PID, command servos. Nothing on core 0 blocks.
- *Core 1, logging + comms.* Drains the inter-core ring buffer to microSD over SPI, services
  optional bench-only Wi-Fi telemetry (`WIFI_ENABLED=0` in flight; the board has no onboard radio,
  so this is a debug-header add-on, not a flight capability), handles housekeeping (camera gate,
  status LED).

Two body tubes, one bulkhead joint:

| Bay | Contents |
|---|---|
| Lower BT (TVC bay) | F15-4 · 2-axis 2-servo gimbal · motor mount |
| Upper BT (FC bay) | Custom PCB1 (bare RP2350B, Ø62 mm) · body BNO085 · BME680 + BMP388 · microSD · i3 4K Thumb Action Camera |
| Bulkhead joint | External BNO085 (STEMMA-QT) mounted here · motor-ejection separation point · 24″ chute · Nomex |

PID gains (auto-tuned): *Kp* 0.10 / *Ki* 0.40 / *Kd* 0.18 · ±8° gimbal authority.

---

## Folder structure

```
Flight Computer/
├── README.md ← this file
├── 01_FlightComputer_Spec.md ← full architecture + sensor config
├── 02_RRC3_Telemetry_Logging.md ← DEPRECATED/REMOVED (redirect only)
├── BOM/ ← empty; FC line items live in Documentation/WYVERN_E4_BOM.xlsx
├── firmware/
│ └── wyvern4_tvc/
│ ├── wyvern4_tvc.ino ← main flight firmware (Arduino-Pico core, board "weact_rp2350b")
│ ├── wyvern_pid.h ← PID controller
│ ├── imu_grv.h ← BNO085 Game Rotation Vector driver
│ ├── sd_logger.h ← microSD ring-buffer logger
│ ├── wifi_telemetry.h ← Wi-Fi bench telemetry
│ ├── baro.h ← BME680 + BMP388 barometric driver
│ └── … ← supporting headers
├── flowcharts/ ← Mermaid state/logic diagrams
│ ├── 01_flight_state_machine.mermaid ← BOOT→ARMED→BOOST→COAST→RECOVER→LANDED
│ ├── 02_tvc_control_loop.mermaid ← 500 Hz PID loop flowchart
│ ├── 03_recovery_logic.mermaid ← motor-ejection separation logic
│ └── 04_power_tree.mermaid ← power distribution diagram
├── ground_test_rigs/ ← bench DAQ, runs on off-the-shelf Pico/Pico 2 W (not the flight computer)
│ ├── wyvern4_gse_servo_rig/
│ │ └── wyvern4_gse_servo_rig.ino ← servo sweep / TVC balance test
│ └── wyvern4_gse_solenoid_rig/
│ └── wyvern4_gse_solenoid_rig.ino ← solenoid ground test (A/B comparison)
├── test_code/
│ ├── t1_i2c_scan.ino ← I²C bus scan (verify all BNO085 addresses)
│ ├── t2_imu_grv_deflection.ino ← GRV deflection read + servo command check
│ ├── t3_servo_sweep.ino ← full ±8° gimbal sweep test
│ ├── t4_sensors_sdlog.ino ← all sensors → microSD log verification
│ ├── host_monitor.py ← Wi-Fi telemetry monitor (run on laptop)
│ └── selftest.py ← automated bench self-test sequence
└── wiring/
    ├── WYVERN_E4_flight_harness.kicad_sch ← flight wiring schematic
    ├── WYVERN_E4_flight_wiring_connected.kicad_sch ← connected (net-tied) version
    ├── WYVERN_E4_flight_wiring_connected_preview.png ← rendered preview
    ├── WYVERN_E4_tvc_balance_harness.kicad_sch ← 3-axis TVC balance harness
    ├── WYVERN_E4_tvc_balance_servo_harness.kicad_sch
    ├── WYVERN_E4_tvc_balance_solenoid_harness.kicad_sch
    └── gen_wiring4.py ← KiCad schematic generator
```

---

## Bench test sequence

Run these in order before any motor firing:

1. `test_code/t1_i2c_scan.ino`, confirm both BNO085s respond on their expected addresses.
2. `test_code/t2_imu_grv_deflection.ino`, manually tilt the airframe, verify body/external
   quaternions agree and servo commands track.
3. `test_code/t3_servo_sweep.ino`, full ±8° sweep, check for binding and correct direction.
4. `test_code/t4_sensors_sdlog.ino`, all sensors write to microSD; verify file on SD card.
5. `test_code/selftest.py` + `host_monitor.py` (laptop), USB-serial bench self-test verification.

Upload firmware via Arduino IDE 2.x with the [Arduino-Pico core](https://github.com/earlephilhower/arduino-pico) installed, board **"WeAct RP2350B"** (`weact_rp2350b`) — the bare-silicon RP2350B
target this custom board (PCB1) actually uses, not a Pico/Pico 2 W module profile.

---

## Flight state machine

```
BOOT → ARMED → BOOST (F15-4 burn, 3.45 s, 500 Hz TVC) → COAST (brief)
     → RECOVER (motor ejection ~t=7.5 s, bulkhead joint separates) → DESCENT → LANDED
```

See `flowcharts/01_flight_state_machine.mermaid` for the full Mermaid diagram.

---

## Related

- `../Documentation/WYVERN_E4_BUILD_READINESS.md`, GO/NO-GO checklist
- `../Documentation/WYVERN_E4_PID_AUTOTUNE_REPORT.md`, PID gain derivation
- `../Simulations/we4_atmos_tvc.py`, closed-loop TVC atmospheric simulation
