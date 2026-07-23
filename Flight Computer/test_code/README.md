# WYVERN-E 4.0 — Test Programs
**Raspberry Pi Pico 2 W (RP2350)** Arduino sketches + host monitor. Build with the **Arduino-Pico
core** (earlephilhower), board *"Raspberry Pi Pico 2 W"*. Flash each, watch USB serial at 115200.

| File | Tests |
|---|---|
| `t1_i2c_scan.ino` | PCA9548A mux + behind every channel + I²C1 gimbal bus — every sensor enumerates |
| `t2_imu_grv_deflection.ino` | BNO085 **Game Rotation Vector** + nozzle deflection q_body⁻¹⊗q_gimbal (gimbal I²C1, body via mux ch0) |
| `t3_servo_sweep.ino` | RP2350 hardware-PWM ±8° gimbal sweep (GP14/GP15) |
| `t4_sensors_sdlog.ino` | BMP388 + BME688 (behind mux) → SPI microSD CSV @100 Hz |
| `host_monitor.py` | reads Pico serial, tabulates preflight PASS/FAIL |
| `selftest.py` | preflight checklist |

**Libraries (Library Manager):** Adafruit_BNO08x, Adafruit_BMP3XX (BMP388), Adafruit_BME680, plus the
Arduino-Pico built-ins `Servo`, `Wire`, `SPI`, `SD`. The mux is plain `Wire` writes (no extra lib).
**Pins:** I²C0 GP16/17 (mux), I²C1 GP18/19 (gimbal), SPI0 microSD GP2/3/4/5, servos GP14/15 — see
`../01_FlightComputer_Spec.md` §3.
