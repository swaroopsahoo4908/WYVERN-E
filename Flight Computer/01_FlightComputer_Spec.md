# GTR70E WYVERN, Flight Computer Specification

**Authors:** Swaroop Sahoo, Chris Liu, Allison Hong  
**Date:** 2026-08-12  
**Program:** GTR70E WYVERN


### Custom PCB1 (bare RP2350B, QFN-80, Ø62 mm board), flight computer *and* real-time TVC controller

*Reconciled 2026-08-11/12 against the fabricated PCB1 netlist, BOM, and schematic
(`PCB/Netlist_PCB1_2026-08-11.tel`, `PCB/SCH_Schematic1_2026-08-11.pdf`), traced pin-by-pin in
`Flight Computer/firmware/wyvern4_tvc/`. Two earlier passes of this document assumed a mux'd
dual-I2C, PCA9548A, 3× BNO085 architecture that was never actually fabricated — this version
matches the real board.*

## 1. Architecture

No Raspberry Pi 5, no Linux, no Teensy, and **no Pico/Pico-W module of any kind** — PCB1 carries a
standalone **RP2350B** (48 GPIO, QFN-80, external QSPI flash) as the entire avionics brain, on the
Arduino-Pico core's `weact_rp2350b` board profile, not `rpipico2`/`rpipico2w`. It reads the IMUs,
closes the TVC loop at 500 Hz, drives the 2 servos with hardware PWM, logs to an SPI microSD
breakout, and observes recovery/camera state. The dual M33 cores are split for determinism:

- **Core 0, real-time control.** The 500 Hz TVC loop, plus everything on the one shared I²C bus
  (body BNO055, external BNO085, BME680, INA226) since RP2350 has a single I²C0 peripheral and only
  one core may safely own it at a time. Nothing on core 0 is allowed to block.
- **Core 1, logging + comms.** Drains an inter-core FIFO to the microSD over SPI0, services the
  (currently inert) bench-only WiFi telemetry code path, and handles housekeeping (camera gate,
  RBF sense, status LED). SD writes can stall here for milliseconds without ever jittering the
  control loop on core 0.

Two body tubes, one bulkhead joint:

- **Lower BT (TVC bay)**, F15-4 + 2-axis 2-servo gimbal + motor mount.
- **Upper BT (FC bay)**, custom PCB1, **body BNO055** (onboard, primary attitude), BME680, SPI
  microSD log, action camera (self-contained, see Camera Solution).
- **Bulkhead joint**, the separation point for **motor ejection** (F15-4 charge pressurizes the
  Lower BT directly against the joint), wiring pass-throughs for the servo extensions and the
  STEMMA-QT cable to the **external BNO085** mounted right at the joint. The FC does not actuate
  recovery, the motor's own delayed ejection charge deploys the chute.

## 2. IMU configuration (body BNO055 + external BNO085, 2-of-2 voting)

The two IMUs on this board are **not the same chip**: the onboard unit (U2) is a **Bosch BNO055**
(register protocol, address `0x28`, confirmed via netlist trace — COM3 tied GND), while the
external unit is an off-board **Adafruit BNO085** breakout on the STEMMA-QT port (CN2, address
`0x4A`, Adafruit default). Both run 6-axis accel+gyro fusion with the magnetometer excluded from
the estimate — BNO085 in `SH2_GAME_ROTATION_VECTOR`, BNO055 in `OPERATION_MODE_IMUPLUS` (Bosch's
equivalent) — a mag-fused IMU inches from two servo motors reads corrupted heading, and neither
board needs magnetic-north reference for a ~7 s flight. They vote against each other for attitude
fault detection.

**There is no gimbal-mounted IMU**, so nozzle deflection (q_body⁻¹ ⊗ q_gimbal) is not computable in
flight — the control loop is body-attitude-only. Direct gimbal-deflection measurement is covered by
the 3-axis load balance on the ground rigs (`WYVERN_E4_GSE_TestStands.md`), not in-flight sensing.

## 3. Bus map (custom PCB1, RP2350B)

The real board has **one shared I²C bus, no mux, no second controller** — every earlier design that
assumed a PCA9548A mux or a dedicated I²C1 for the external IMU was never fabricated this way.

| Bus | Pins | Members |
|---|---|---|
| **I²C0** (single shared bus) | GP0 SDA / GP1 SCL | body BNO055 (0x28) · external BNO085 (0x4A, STEMMA-QT CN2) · BME680 (0x76) · INA226 (U4, address TBD by bench scan — see §4) |
| **SPI0** (microSD) | per `sd_logger.h` | flight-data log (full-rate IMU/baro/control) |
| **PWM** | GP2 (S1 pitch) / GP3 (S2 yaw) | 2× servo signal, hardware PWM slices |
| **GPIO** | GP12 (RBF sense, H1 pin13 — **not wired to any switch on this board rev**, see note below) | discrete I/O |
| **Radio** | none populated | no CYW43439, no onboard WiFi/BLE — `WIFI_ENABLED` is hard-defined `0` |

**Open item, RBF.** There is no software-readable remove-before-flight pin on this board as
fabricated. U13 (the physical slide switch near the power path) connects to nothing on U1 in the
netlist trace — both its terminals sit in the power domain, not on any GPIO. Arming safety on PCB1
is currently provided entirely by U13 being a literal power switch: the board isn't running until
it's flipped. GP12 is kept wired to an `INPUT_PULLUP` software gate as a hook for a future bodge
wire, but as fabricated nothing is soldered there, so this stage of the BOOT gate provides no actual
protection yet.

> **3.3 V logic.** RP2350B GPIO is 3.3 V. All STEMMA-QT sensors (BNO085, BME680) are 3.3 V-safe.
> Recovery is motor-driven (F15-4 ejection at the bulkhead joint); there are no deploy actuators, no
> recovery battery, and no pyro for the FC to drive.

## 4. Power

- **2S LiPo → onboard buck, no discrete UBEC.** One 2S LiPo (7.4 V, ~450 mAh) feeds PCB1's XT30
  input directly into the onboard **TPS564201** synchronous buck (U15), which supplies the ~5 V
  servo/camera rail; 3.3 V logic comes off the board's onboard LDO stage. There is no separate UBEC
  module in this design — the "one 5 V UBEC" language in the project brief refers to this onboard
  buck, not an added part.
- **Battery monitor wiring defect (INA226, U4).** Traced pin-by-pin against the netlist: U4's
  VBUS/VIN- both land on the buck's **output** (VBUCK, ~4.98 V calculated from the feedback
  divider), not the raw pack input, so `getBusVoltage()` currently reads the regulated rail, not the
  6.0–8.4 V 2S pack range — the 6.4 V/6.0 V (3.2/3.0 V-per-cell) cutoffs from the project brief do
  **not** apply to this reading as wired. VIN+/VIN- also don't span a real series shunt, so
  current/power readings aren't physically meaningful either. Firmware (`battery.h`) has been
  updated to rail-sag thresholds instead of pack-voltage thresholds as a software workaround; the
  real fix is a board revision routing U4 across an actual pack-current shunt. See
  `Documentation/CONFLICTS.md` §3 for the full defect record.
- **Address strap.** U4's A1 pin isn't cleanly strapped to any of the INA226's four supported
  address levels (it sees ~5 V off VBUCK through R10, not GND/VS+/SDA/SCL) — `0x40` is the current
  best-guess bench-scan candidate, not a confirmed address. Run `test_code/t1_i2c_scan.ino` and
  update `INA226_ADDR` to whatever the scan actually finds before trusting this class's output.
- **Recovery:** none, the F15-4 motor's own ejection charge deploys the chute at the bulkhead joint.
  No recovery battery or deploy electronics.

## 5. Control loop (500 Hz, deterministic, core 0)

**TVC is disabled for the first 0.5 s** (the F15 ignition spike), the fins hold attitude passively;
at t = 0.5 s the loop engages on the smooth thrust curve. Each cycle: read external + body IMU
(fused orientation, both accel+gyro-only) → PID about the setpoint (stabilize-to-vertical, then a
4° commanded maneuver) → clip to ±8° → hardware PWM to the 2 servos (GP2/GP3) → push a log frame to
the core-1 ring buffer. At burnout thrust → 0 ⇒ no control authority ⇒ coast to the F15-4 ejection
(t ≈ 7.45 s, 0.58 s past apogee), which separates the two body tubes at the bulkhead joint and
deploys the chute. Firmware lives in `firmware/wyvern4_tvc/` (sketch name = folder name, all `.h`
files are tabs, see `Documentation/COMPATIBILITY.md` and `Documentation/CONFLICTS.md`).

## 6. Logging + telemetry (core 1)

Core 1 pops log frames from the inter-core FIFO and streams them to the SPI microSD as CSV/binary at
full rate (no flush in the control path). `wifi_telemetry.h`'s UDP broadcaster code path exists but
is inert — **PCB1 has no radio chip populated**, so `WIFI_ENABLED` stays `0` and telemetry is
logged, not streamed. Flight data of record is always the on-board microSD, pulled post-flight.

## 7. Why the custom PCB1 (vs. the Teensy 4.1 / Pico 2 W it replaces)

| | Teensy 4.1 (was) | Pico 2 W (interim design, never fabricated) | Custom PCB1 (now) |
|---|---|---|---|
| Core | 1× 600 MHz M7 | 2× 150 MHz M33 | **2× 150 MHz M33** (control + logging split) |
| MCU | discrete module | Pico 2 W module | **bare RP2350B, QFN-80, on the custom board** |
| Real-time guarantee | single thread; SD flush risk | core 0 never touches SD/radio | **core 0 owns the sole I²C bus + control loop; SD/radio isolated to core 1** |
| microSD | built-in SDIO | SPI breakout | SPI breakout (on PCB1) |
| Wireless | none | Wi-Fi 802.11n + BLE (CYW43439) | **none populated** — `WIFI_ENABLED=0`, telemetry is log-only |
| I²C | 3 controllers | 2 + PCA9548A mux (never built) | **1 shared bus, no mux** (netlist-confirmed) |
| PWM | FlexPWM | RP2350 PWM slices | RP2350B PWM slices (GP2/GP3) |
| Power | — | Pico VSYS + discrete UBEC | **onboard TPS564201 buck, no discrete UBEC** |

The move from the never-fabricated Pico-2-W-module design to the real custom board (PCB1) trades a
COTS module's WiFi/BLE and simpler bus topology for a purpose-built Ø62 mm board sized to the
airframe, at the cost of two open hardware findings (INA226 wiring, RBF pin) tracked in
`Documentation/CONFLICTS.md`.

## References

CEVA, Inc. (2023). *BNO08X datasheet* (Rev. 1.17). https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

Raspberry Pi Ltd. (2024). *RP2350 datasheet*. https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf

Texas Instruments. (n.d.). *TPS564201: 4.5-V to 17-V input, 4-A synchronous step-down voltage regulator* (SLVSFB5) [Datasheet]. https://www.ti.com/lit/ds/symlink/tps564201.pdf
