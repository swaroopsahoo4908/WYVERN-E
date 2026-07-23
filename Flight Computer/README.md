# WYVERN-E 4.0 — Flight Computer

*Single Raspberry Pi Pico 2 W (RP2350) — flight computer and real-time TVC controller.*

See `01_FlightComputer_Spec.md` for the full architecture writeup.

---

## Architecture summary

Dual-core RP2350 split for determinism:

- *Core 0 — real-time control.* 500 Hz TVC loop only: read gimbal + body BNO085 (Game Rotation
  Vector), compute nozzle deflection, run PID, command servos. Nothing on core 0 blocks.
- *Core 1 — logging + comms.* Drains the inter-core ring buffer to microSD over SPI, services
  Wi-Fi bench telemetry, handles housekeeping (camera gate, status LED).

Three bays, two sealed bulkheads:

| Bay | Contents |
|---|---|
| Engine/TVC bay | F15-4 · 2-axis 2-servo gimbal · gimbal BNO085 (dedicated I²C) |
| Flight-computer bay | Pico 2 W · body BNO085 · BME688 + BMP388 · microSD · i3 4K Thumb Action Camera · Wi-Fi |
| Recovery bay | Motor ejection (F15-4 bypass tube) · 18″ chute · Nomex · 3rd BNO085 (redundant) |

PID gains (auto-tuned): *Kp* 0.10 / *Ki* 0.40 / *Kd* 0.18 · ±8° gimbal authority.

---

## Folder structure

```
Flight Computer/
├── README.md                              ← this file
├── 01_FlightComputer_Spec.md              ← full architecture + sensor config
├── 02_RRC3_Telemetry_Logging.md          ← DEPRECATED/REMOVED (redirect only)
├── BOM/
│   └── WYVERN_E3_FlightComputer_BOM.xlsx ← FC bill of materials
├── firmware/
│   └── wyvern4_tvc/
│       ├── wyvern4_tvc.ino               ← main flight firmware (Arduino/Pico SDK)
│       ├── wyvern_pid.h                  ← PID controller
│       ├── imu_grv.h                     ← BNO085 Game Rotation Vector driver
│       ├── sd_logger.h                   ← microSD ring-buffer logger
│       ├── wifi_telemetry.h              ← Wi-Fi bench telemetry
│       ├── baro.h                        ← BME688 + BMP388 barometric driver
│       └── …                            ← supporting headers
├── flowcharts/                            ← Mermaid state/logic diagrams
│   ├── 01_flight_state_machine.mermaid   ← BOOT→ARMED→BOOST→COAST→RECOVER→LANDED
│   ├── 02_tvc_control_loop.mermaid       ← 500 Hz PID loop flowchart
│   ├── 03_recovery_logic.mermaid         ← motor-ejection bypass logic
│   └── 04_power_tree.mermaid             ← power distribution diagram
├── ground_test_rigs/
│   ├── wyvern4_gse_servo_rig/
│   │   └── wyvern4_gse_servo_rig.ino    ← servo sweep / TVC balance test
│   └── wyvern4_gse_solenoid_rig/
│       └── wyvern4_gse_solenoid_rig.ino ← solenoid ground test (A/B comparison)
├── test_code/
│   ├── t1_i2c_scan.ino                  ← I²C bus scan (verify all BNO085 addresses)
│   ├── t2_imu_grv_deflection.ino        ← GRV deflection read + servo command check
│   ├── t3_servo_sweep.ino               ← full ±8° gimbal sweep test
│   ├── t4_sensors_sdlog.ino             ← all sensors → microSD log verification
│   ├── host_monitor.py                  ← Wi-Fi telemetry monitor (run on laptop)
│   └── selftest.py                      ← automated bench self-test sequence
└── wiring/
    ├── WYVERN_E4_flight_harness.kicad_sch            ← flight wiring schematic
    ├── WYVERN_E4_flight_wiring_connected.kicad_sch   ← connected (net-tied) version
    ├── WYVERN_E4_flight_wiring_connected_preview.png ← rendered preview
    ├── WYVERN_E4_tvc_balance_harness.kicad_sch       ← 3-axis TVC balance harness
    ├── WYVERN_E4_tvc_balance_servo_harness.kicad_sch
    ├── WYVERN_E4_tvc_balance_solenoid_harness.kicad_sch
    └── gen_wiring4.py                                ← KiCad schematic generator
```

---

## Bench test sequence

Run these in order before any motor firing:

1. `test_code/t1_i2c_scan.ino` — confirm all 3× BNO085 respond on their expected addresses.
2. `test_code/t2_imu_grv_deflection.ino` — manually tilt gimbal, verify servo commands track.
3. `test_code/t3_servo_sweep.ino` — full ±8° sweep, check for binding and correct direction.
4. `test_code/t4_sensors_sdlog.ino` — all sensors write to microSD; verify file on SD card.
5. `test_code/selftest.py` + `host_monitor.py` (laptop) — Wi-Fi telemetry stream live verification.

Upload firmware via Arduino IDE 2.x with the [Raspberry Pi Pico 2 W board package](https://github.com/earlephilhower/arduino-pico) installed.

---

## Flight state machine

```
BOOT → ARMED → BOOST (F15-4 burn, 3.45 s, 500 Hz TVC) → COAST (brief)
     → RECOVER (motor ejection ~t=7.5 s via bypass tube) → DESCENT → LANDED
```

See `flowcharts/01_flight_state_machine.mermaid` for the full Mermaid diagram.

---

## Related

- `../Documentation/WYVERN_E4_BUILD_READINESS.md` — GO/NO-GO checklist
- `../Documentation/WYVERN_E4_PID_AUTOTUNE_REPORT.md` — PID gain derivation
- `../Simulations/we4_atmos_tvc.py` — closed-loop TVC atmospheric simulation
