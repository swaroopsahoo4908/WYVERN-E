# WYVERN-E 4.0 — Flight Computer Specification

### Single Raspberry Pi Pico 2 W (RP2350) — flight computer *and* real-time TVC controller

## 1. Architecture

No Raspberry Pi 5, no Linux, no Teensy. A **Raspberry Pi Pico 2 W** (RP2350: dual-core 150 MHz
Cortex-M33, 520 KB SRAM, 4 MB flash, on-board CYW43439 Wi-Fi/BLE) is the entire avionics brain. It
reads the IMUs, closes the TVC loop at 500 Hz, drives the 2 servos with hardware PWM, logs to an
SPI microSD breakout, and triggers recovery/camera. The dual cores are split for determinism:

- **Core 0 — real-time control.** The 500 Hz TVC loop *only*: read gimbal + body BNO085 (Game
  Rotation Vector), compute nozzle deflection, run the PID, command the servos. Nothing on core 0
  is allowed to block.
- **Core 1 — logging + comms.** Drains an inter-core ring buffer to the microSD over SPI, services
  Wi-Fi/BLE bench telemetry, and handles housekeeping (camera gate, status LED). SD writes
  and Wi-Fi can stall here for milliseconds without ever jittering the control loop on core 0.

This core split is the headline upgrade over a single-threaded MCU: the blocking I/O (SD, radio)
that used to threaten loop timing is physically on the other core. RP2350 also offers a hardware
single-precision FPU on each M33, so the quaternion math runs natively.

Three bays, two bulkheads:

- **Engine/TVC bay** — F15-4 + 2-axis 2-servo gimbal + **gimbal BNO085** on a dedicated I²C bus
  (reads true nozzle attitude inside the gimbal — catches linkage backlash/flex).
- **Flight-computer bay** — Pico 2 W, **body BNO085** (primary attitude), BME688, BMP388 (Adafruit
  3966), SPI microSD log, action camera (self-contained — see Camera Solution).
- **Recovery bay** — **motor ejection** (F15-4 charge routed through a bypass tube past the sealed
  FC bay), parachute + Nomex blanket, **3rd BNO085** (redundant body attitude for 2-of-3 voting). The
  FC does not actuate recovery — the motor's own delayed ejection charge deploys the chute.

## 2. IMU configuration — Game Rotation Vector (critical)

All three BNO085 run in **Game Rotation Vector** mode (accel + gyro fusion, **magnetometer
disabled**). A mag-fused IMU inches from two servo motors reads corrupted heading; GRV gives full
quaternion orientation referenced to power-on (relative, not magnetic-north) — exactly what TVC
needs. Gimbal deflection each loop:

$$q_{\text{defl}} = q_{\text{body}}^{-1}\otimes q_{\text{gimbal}} \;\Rightarrow\; (\theta_{pitch},\theta_{yaw})$$

computed on core 0. Body and gimbal share the same GRV reference frame; the recovery unit votes
against the body unit. GRV yaw drifts slowly without a mag reference — negligible over the ~7 s flight.

Because all three BNO085 share I²C address **0x4A**, the two on the shared bus (body, recovery) are
separated by a **PCA9548A 8-channel mux**; the gimbal unit gets its own dedicated I²C controller
(no mux latency, electrically isolated from the noisier shared bus).

## 3. Bus map (Pico 2 W, RP2350)

| Bus | Pins | Members |
|---|---|---|
| **I²C0** (mux trunk) | GP16 SDA / GP17 SCL | PCA9548A @0x70 → ch0 body BNO085 (0x4A), ch1 recovery BNO085 (0x4A), ch2 BME688 (0x76), ch3 BMP388 (0x77), ch4 spare (unpopulated) |
| **I²C1** (dedicated) | GP18 SDA / GP19 SCL | gimbal BNO085 (0x4A) — real-time TVC sensor, isolated |
| **SPI0** (microSD) | GP2 SCK / GP3 MOSI / GP4 MISO / GP5 CS | flight-data log (full-rate IMU/baro/control) |
| **PWM** | GP14 (S1 pitch) / GP15 (S2 yaw) | 2× servo signal, hardware PWM slices |
| **GPIO** | GP7 launch IRQ (BNO085 INT) · GP8 camera power gate · GP9 status LED · GP10 buzzer · GP22 RBF arm-pin sense · GP1/GP6 spare (freed — no deploy actuation) | discrete I/O |
| **ADC** | GP26/ADC0 (2S LiPo pack monitor, 100k/62k divider) · GP28 (servo-current shunt, opt) | analog |
| **Wi-Fi/BLE** | CYW43439 (internal) | bench/preflight telemetry (UDP), optional low-altitude link |

The three BNO085 are all 0x4A: gimbal on its own controller (I²C1); body + recovery isolated on
mux channels 0/1. The two baros have unique addresses (0x76/0x77) but ride their own mux channels
for clean pull-up isolation.

> **3.3 V logic.** RP2350 GPIO is 3.3 V. All STEMMA-QT sensors (BNO085, BME688, **BMP388 Adafruit
> 3966**) are 3.3 V-safe. Recovery is motor-driven (F15-4 ejection via bypass tube) — there are no
> deploy actuators, no recovery battery, and no pyro for the FC to drive.

## 4. Power

- **Light 2S LiPo → 5 V UBEC:** one 2S LiPo (7.4 V, ~450 mAh; Zeee 4-pk) feeds a single 5 V/6 V UBEC
  set to **5 V**, whose one rail powers Pico 2 W VSYS (1.8–5.5 V, on-board buck-boost to 3.3 V for the
  IMUs/baros), the camera, and both TVC servos. The servos run happily at 5 V (~1.8 kg·cm, still >2×
  the ~0.9 kg·cm gimbal demand). One BEC, one rail — no separate 6 V servo BEC needed at this scale.
- **Decoupling:** servo stall/reversal transients (~1 A each) must not brown-out the Pico, so add a
  **1000 µF** low-ESR bulk cap across the servo V+/GND at the servos, **100 µF** at VSYS, and an
  **SS34 Schottky** from rail → VSYS as a hold-up diode; keep the servo feed and the VSYS feed as
  separate star runs off the UBEC output.
- **Monitoring:** pack voltage (before the BEC) on GP26/ADC0 through the 100k/62k divider, which keeps
  2S full-charge (8.4 V) at ~3.21 V, just under the 3.3 V ADC ref; firmware `battery.h` warns at
  6.4 V (3.2 V/cell) and inhibits arming below 6.0 V (3.0 V/cell).
- **Recovery:** none — the F15-4 motor's own ejection charge deploys the chute (via the bypass tube). No recovery battery or deploy electronics.

The Pico 2 W draws ~30–100 mA (Wi-Fi off/on); the avionics budget is dominated by the servos and
camera. A 2S 450 mAh pack (~30 g) gives comfortable pad + flight endurance, and with the Hobbywing
UBEC (~10 g) and the i3 4K Thumb Action Camera (~36 g) the power+camera group is ~76 g — consistent with the
122 g FC-bay line and the 705 g / 435 ft / 1.10 cal flight budget.

## 5. Control loop (500 Hz, deterministic — core 0)

**TVC is disabled for the first 0.5 s** (the F15 ignition spike) — the fins hold attitude passively;
at t = 0.5 s the loop engages on the smooth thrust curve. Each cycle: read gimbal + body BNO085 (GRV)
→ `q_defl` → PID about the setpoint (stabilize-to-vertical, then commanded maneuver) → clip to ±8° →
hardware PWM to the 2 servos → push a log frame to the core-1 ring buffer. Servo slew (~0.04 s lag
modelled) keeps the gimbal inside ±8° with authority headroom (low-wind pitch dev <4°; halved saturation vs ±5°). At burnout thrust → 0 ⇒ no control
authority ⇒ coast to the F15-4 ejection (~t = 7.5 s, 0.7 s past apogee) which deploys the chute via the bypass tube. The 2 ms cycle budget is comfortable at
150 MHz with the hardware FPU. Firmware lives in the Arduino IDE sketch folder `firmware/wyvern4_tvc/`
(sketch name = folder name, all `.h` files are tabs — see `Documentation/COMPATIBILITY.md` and
`Documentation/CONFLICTS.md` §6 for the sketch-folder reorganization record).

## 6. Logging + telemetry (core 1)

Core 1 pops log frames from the inter-core FIFO and streams them to the SPI microSD as CSV/binary at
full rate (no flush in the control path). The CYW43439 provides an **optional Wi-Fi UDP telemetry
feed** for bench/preflight monitoring — a genuine new capability over the Teensy's USB-only link
(live quaternion + deflection + baro on a laptop with no tether). Flight data of record is always the
on-board microSD, pulled post-flight; Wi-Fi is bench/range-line-of-sight only.

## 7. Why the Pico 2 W (vs. the Teensy 4.1 it replaces)

| | Teensy 4.1 (was) | Pico 2 W (now) |
|---|---|---|
| Core | 1× 600 MHz M7 | **2× 150 MHz M33** (control + logging split) |
| Real-time guarantee | single thread; SD flush risk | **core 0 never touches SD/radio** |
| microSD | built-in SDIO | SPI breakout (in BOM) |
| Wireless | none | **Wi-Fi 802.11n + BLE** (bench telemetry) |
| I²C controllers | 3 | 2 + **PCA9548A mux** (in BOM) |
| PWM | FlexPWM | RP2350 PWM slices (16 ch) |
| FPU | yes (M7) | yes (per-core M33) |
| Cost | $31.50 | **$20.95** |

The trade is raw clock for a cleaner real-time story (a dedicated logging core), wireless bench
telemetry, and lower cost — at the price of moving SD to SPI and adding the I²C mux, both already on
the BOM.
