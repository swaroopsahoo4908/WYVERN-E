# WYVERN-E 4.0 — Component Compatibility Audit

**Scope:** Every I2C, SPI, PWM, ADC, and power pairing across the flight computer (Raspberry Pi Pico 2 W / RP2350, `wyvern4_tvc.ino` + headers) and the ground-test rig (3-axis TVC balance harness + static thrust stand), cross-checked against the BOM part list and the frozen pin map in `Documentation/CONFLICTS.md` §5.

**Sources examined:** `Flight Computer/firmware/*.{ino,h}`, `Flight Computer/wiring/gen_wiring4.py` + `gen_connected_sch.py` (+ resulting `.kicad_sch`), `Flight Computer/test_code/*`, `Flight Computer/flowcharts/*.mermaid`, `Flight Computer/01_FlightComputer_Spec.md`, `Flight Computer/02_RRC3_Telemetry_Logging.md`, and all ten files under `Documentation/`.

**What this audit can and cannot verify:** the audit is a *static* review of source, wiring generators, and documentation — it confirms what is wired, coded, and internally consistent, and flags what is not. It cannot confirm bench-measured current draw, actual signal levels on a physical harness, or I2C bus electrical margin (capacitance, pull-up value, rise time) — those require a meter and an oscilloscope on real hardware. Every current-draw figure below is a **datasheet/vendor-typical estimate**, explicitly labeled, not a measurement.

---

## Summary Table

| # | Pairing | Verdict | One-line reason |
|---|---|---|---|
| 1a | I2C0 mux channel/address map (PCA9548A, 4 populated + 1 spare of 8) | **PASS** | No address collisions; both 0x4A devices are mux-isolated one-at-a-time |
| 1b | I2C1 dedicated gimbal BNO085 | **PASS** | Sole device on bus, isolated from I2C0 mux faults by design |
| 1c | I2C bus speed vs. device max rate | **PASS** (default Wire.begin() clock, see note) | Arduino-Pico default 100 kHz is within all three devices' rated range |
| 1d | VL53L4CD ToF ranging | **RESOLVED — removed (2026-07)** | ToF sensors deleted from the BOM and design entirely; gimbal deflection is measured by the 3-axis load balance, so no ToF driver/XSHUT plan is needed |
| 2 | SPI0 microSD | **PASS** | Dedicated 4-wire bus, no sharing, no conflicting CS |
| 2b | HX711 (ground rig) mislabeled as SPI | **CAUTION — terminology** | HX711 uses a proprietary 2-wire DT/SCK protocol, not SPI |
| 3 | PWM servo outputs (GP14/GP15) | **PASS** | Two independent PWM slices, standard hobby-servo timing, travel-limited in software |
| 3b | Ground-rig solenoid PWM via IRF520 | **CAUTION** | No PWM frequency/flyback-diode spec found in provided docs |
| 4 | ADC0 battery divider (GP26) | **PASS** | 100k/62k keeps 2S LiPo full-charge (8.4 V) at ~3.21 V, just under the 3.3 V reference |
| 4b | Load cells (ground rig) vs. Pico ADC | **PASS (non-issue)** | HX711 has its own 24-bit ADC; consumes 0 Pico ADC channels |
| 5a | Recovery = F15-4 motor ejection via bypass tube | **PASS — no recovery electronics** | Motor's own charge; no RRC3+, no UART tap, no recovery battery, no pyro driven by the FC |
| 5b | (retired) BSS138 level shifter | **N/A** | Was only for the RRC3+ UART tap, which is removed; drop from BOM if still listed |
| 5c | 2S LiPo → one 5 V UBEC → shared 5 V rail (servos + VSYS) | **PASS w/ decoupling** | UBEC (set 5 V) drives VSYS + servos + camera; star-wire the two feeds and add 1000 µF @ servos + 100 µF + SS34 hold-up @ VSYS so servo transients don't brown-out the Pico |
| 6a | Ground-rig DAQ board | **RESOLVED (2026-07)** | Both rig wiring blocks now specify a Raspberry Pi Pico / KB2040 DAQ (matches the spec); the Arduino Nano is an owned bench-test auxiliary only (Already Acquired), not the balance DAQ |
| 6b | Ground-rig GPIO budget (Pico substitution scenario) | **CAUTION — provisional, no VL53L4CD ring assumed** | Fits comfortably without ToF ring; tight/unresolved if VL53L4CD ring is added without a mux |

---

## 1. I2C — Address Collisions & Bus Loading

### 1a. I2C0 mux trunk (Wire, GP16 SDA / GP17 SCL) — PASS

The frozen channel map in `i2c_mux.h` and confirmed in `CONFLICTS.md` §5:

- ch0 — body BNO085, `0x4A`
- ch1 — recovery BNO085, `0x4A`
- ch2 — BME688, `0x76`
- ch3 — BMP388 (Adafruit 3966), `0x77`
- ch4 — spare, unpopulated
- ch5, ch6, ch7 — **unassigned** (3 of 8 channels have no documented use anywhere in the repo)

Both BNO085 units share address `0x4A`, which is normally a collision — but the PCA9548A mux resolves it structurally: `I2CMux::select()` enables exactly one channel bitmask at a time, so only one `0x4A` device is ever visible to the bus master at once. Channel 0xFF deselects all as a safe idle state, useful before talking to a device that should not also be exposed to whatever device is currently selected on another channel sharing the same address (both BNO085s sit at 0x4A), per the driver's own inline comment. This is a correct, standard use of an I2C mux for same-address devices — **verdict PASS**.

Total documented population is 4 of 8 channels; 1 more (ch4) is earmarked but unpopulated, leaving **3 channels (ch5–ch7) with no assignment anywhere in firmware, wiring generator, or documentation** — worth noting as available headroom, not a defect.

### 1b. I2C1 dedicated gimbal bus (Wire1, GP18 SDA / GP19 SCL) — PASS

The gimbal BNO085 sits alone at `0x4A` on a physically separate bus from the mux trunk. The driver header states the rationale directly: the gimbal BNO085 is not behind the mux — it is on the dedicated I2C1 bus (Wire1), per spec, so that mux failure/bus-lockup on I2C0 can never take down the one IMU the control loop must always have. No address collision is possible since it is the sole device on the segment. **Verdict PASS.**

### 1c. I2C bus clock speed — PASS (with a caveat)

`wyvern4_tvc.ino` initializes both buses with `Wire.setSDA(SDA0); Wire.setSCL(SCL0); Wire.begin();` and the matching call for `Wire1` — **no explicit `setClock()` call is present in the firmware**, so both buses run at the Arduino-Pico core's default of 100 kHz (standard mode). All four device families on these buses (BNO085, BME688, BMP388, PCA9548A) support both 100 kHz and 400 kHz per their datasheets, so 100 kHz is safely within spec for all of them — **PASS**, but the mux-trunk bus is carrying four sensors' worth of traffic (dual IMU vote + dual baro read) at only 100 kHz, which is a design headroom choice, not a defect. If bench timing margin ever becomes tight at 500 Hz control-loop rate, an explicit `Wire.setClock(400000)` is the lever to pull — worth flagging as a tuning option, not a required fix.

### 1d. VL53L4CD ToF — RESOLVED, removed from the design (2026-07)

**The VL53L4CD Time-of-Flight sensors have been removed entirely** from the BOM and the design. Gimbal
deflection on the solenoid balance rig is taken from the 3-axis load balance (thrust vector) plus the
gimbal BNO085, so no ToF ring, driver, or XSHUT/address plan is required. The paragraphs below are
retained only as a record of why the ToF approach was dropped.

The BOM previously specified 6x Adafruit VL53L4CD (STEMMA QT). A full-repo search found:
- **No VL53L4CD driver/include anywhere** in firmware, test code, or wiring.
- **No "XSHUT" reference anywhere** in the repo.

Both are required to use more than one VL53L4CD on a shared bus: all boards default to I2C address `0x29`, and the standard technique is to hold every unit's XSHUT low except one, assign it a new address over I2C, release the next XSHUT, and repeat. None of that reassignment logic exists in this codebase. Separately, the flight-side mux (`i2c_mux.h`) has only **one** documented spare channel (ch4), which the mux's own comment names as the VL53L4CD slot — `ch4 = spare (e.g. VL53L4CD AGL) — unpopulated by default` — sufficient for at most one ToF unit per mux channel used, or up to 6 units on ch4 alone *if* XSHUT-based re-addressing were implemented (none of the 6 units could otherwise coexist at the shared default address `0x29` on one channel).

**Finding:** the 6x VL53L4CD units in the BOM are parts-only at this point in the repo — neither flight firmware nor the ground-rig scripts drive them. This is a genuine gap that must be closed (driver + XSHUT sequencing + a channel/address plan) before these sensors can be brought online on either rig.

---

## 2. SPI

### 2. SPI0 microSD (GP2 SCK / GP3 MOSI / GP4 MISO / GP5 CS) — PASS

`sd_logger.h` pins are unambiguous: `static constexpr uint8_t PIN_SCK = 2, PIN_MOSI = 3, PIN_MISO = 4, PIN_CS = 5;`. This is the sole SPI0 device on the flight computer — no bus sharing, no second CS line to arbitrate, and the driver's own architecture (core-1 FIFO drain, burst writes, periodic flush) is built specifically so that SD writes and Wi-Fi can stall here for milliseconds without ever jittering the control loop on core 0, per the design note carried over from `01_FlightComputer_Spec.md`. **Verdict PASS** — nothing to arbitrate, timing isolation is deliberate and core-separated.

### 2b. HX711 (ground rig) — CAUTION, terminology only

The BOM lists three NOYITO HX711 load-cell amplifiers on the balance rig (5 kg axial + 2x1 kg lateral) and a fourth on the separate static-thrust stand. The HX711 is **not an SPI device** — it uses a proprietary 2-wire synchronous serial protocol (one `DT` data-out pin, one `SCK` clock-in pin per channel), read by bit-banging, not by any standard SPI/I2C peripheral. `gen_wiring4.py`'s own labels reflect this correctly (`HX1_DT/SCK: axial Z (5kg)`, `HX2_DT/SCK: lateral X (1kg)`, `HX3_DT/SCK: lateral Y (1kg)`) — i.e., the wiring generator already treats it as DT/SCK pairs, not SPI MOSI/MISO/SCK/CS. This audit flags it only because "SPI" appears in casual BOM/task language elsewhere — the actual wiring is correct. **No fix required; documentation precision only:** each HX711 channel consumes **2 GPIOs** (DT + SCK), not a shared SPI bus, and three channels do not share pins with each other — each needs its own DT/SCK pair (6 GPIOs total for the 3-axis balance rig).

---

## 3. PWM

### 3. Flight TVC servos (GP14 pitch / GP15 yaw) — PASS

`wyvern4_tvc.ino`: `#define PIN_SERVO_P 14 // pitch servo (PWM)`, `#define PIN_SERVO_Y 15 // yaw servo (PWM)`, driven through the Arduino-Pico `Servo` library (`g_servo_pitch.attach(PIN_SERVO_P)`), with commands clamped to `SERVO_NEUTRAL_DEG +/- 8.0 deg` (via `wyvern_pid.h OUT_LIM_DEG=8.0`) before being written — matching the CONFLICTS.md frozen value (`Output (gimbal) limit +/-8.0 deg`, raised 5->8 for wind/weathercock authority) and the EMAX ES08MA II's mechanical/electrical rating as a standard-PWM analog micro servo (~2.0 kg·cm at 6 V, well above the ~0.9 kg·cm gimbal demand). Two independent RP2350 PWM slices, no pin sharing, no conflict. **Verdict PASS.**

### 3b. Ground-rig solenoid actuator (50N 12V solenoids x2 via IRF520, "Actuator A" swap-in) — CAUTION

`gen_wiring4.py`'s balance-harness block lists the alternate actuator as: `A: 2x 50N solenoid via IRF520 (PWM/Nano)` — i.e., the same DAQ board drives the solenoids over IRF520 MOSFET modules on PWM-capable pins. `WYVERN_E4_GSE_TestStands.md` documents the rig's load-cell/mechanical design in detail but **does not specify** a PWM switching frequency for the solenoid drive, nor call out flyback-diode protection explicitly in the provided documentation set — the IRF520 breakout modules in this BOM (HiLetgo, "pack of 4") do carry an onboard flyback diode across the load terminal per their standard design, but this repo does not document or confirm that the diode is present/adequate for a 50N/12V inductive solenoid's turn-off transient. **Recommendation:** before energizing, confirm (a) the IRF520 module's onboard flyback diode is rated for the solenoid's stored energy at 12V/its coil current, and (b) a PWM frequency is chosen and documented (a few hundred Hz to a few kHz is typical for MOSFET-switched DC solenoids/relays) — neither is currently in the repo, so this is a **documentation and pre-bench-test gap**, not a wiring error.

---

## 4. ADC

### 4. Battery voltage divider (GP26 / ADC0) — PASS

This channel monitors the 2S LiPo flight pack (tapped before the BEC); `CONFLICTS.md` §4 documents GP26 (ADC0) with R_top = 100 kOhm (VBAT -> ADC node) and R_bot = 62 kOhm (ADC node -> GND), giving a divider ratio of 62/(100+62) = 0.3827. At 2S full charge (8.4V) the ADC sees ~3.21V, just under the 3.3V reference; at the 6.0V cutoff (3.0 V/cell) it sees ~2.30V. `battery.h` implements the matching math exactly (`DIVIDER_RATIO = 62/(100+62)`, `ADC_VREF = 3.30`, 12-bit/4095 counts), un-does the divider in software, and flags low-battery at 6.4V (3.2 V/cell) with a 6.0V (3.0 V/cell) arm-inhibit — **verdict PASS** (~90 mV headroom at full charge). `GP28 (ADC2)` is reserved as a second spare analog input but is unwired by default; no conflict, just documented headroom.

### 4b. Ground-rig load cells vs. Pico ADC — PASS (non-issue, worth stating explicitly)

The four HX711 modules (5 kg, 2x1 kg, 20 kg) each carry their **own onboard 24-bit ADC** for the strain-gauge bridge — they output a digital DT/SCK stream to the host MCU and consume **zero** channels of the Pico's onboard ADC (ADC0–ADC2, GP26–GP28). Likewise, the 6x VL53L4CD (once implemented) and all three BNO085 IMUs are digital I2C-only devices and consume no ADC channel either. Stating this explicitly because "ADC" appears in the audit's required section list — there is no shared/contended ADC resource between flight and ground-rig hardware; the only ADC use in the entire project is the single flight-battery divider above.

---

## 5. Power

### 5a. Recovery = F15-4 motor ejection via bypass tube — PASS (no recovery electronics)

The earlier design tapped an RRC3+ recovery computer's UART and carried an isolated 9 V recovery
battery. **That is all removed.** Recovery is now the F15-4 motor's own ejection charge, fired 4 s
after burnout (t ≈ 7.45 s), routed through a solid-walled bypass tube past the sealed FC bay into
the recovery bay to release the friction-fit nose (see `WYVERN_E4_Recovery.md`,
`Simulations/we4_ejection_feasibility.py`, and the FEA §4 ejection-load check in
`WYVERN_E4_FEA_Structural.md`).

Consequences for this audit: there is no recovery UART line, no shared serial reference, no
recovery battery, and the flight computer drives **no** pyro or deploy hardware whatsoever — it only
logs baro/IMU and streams WiFi telemetry. The logic-level-mismatch risk that dominated the old 5a
(RRC3+ TX vs. 3.3 V GP1) no longer exists because that connection no longer exists. GP1 and GP6 are
now spare.

**Verdict: PASS** — removing the recovery electronics removes the entire class of shared-reference /
level-mismatch / isolated-battery concerns this section previously tracked.

### 5b. (retired) BSS138 level shifter

The BSS138 existed in the BOM only as the candidate level-shifter for the RRC3+ Comm-Port TX → Pico
GP1 tap. With the RRC3+ removed, there is no 5 V→3.3 V UART line to shift; the part is no longer
needed and should be dropped from the BOM if still listed. **N/A.**

### 5c. 2S LiPo → one 5 V UBEC → shared 5 V rail — PASS with required decoupling

The power tree is a **light 2S LiPo (7.4 V, ~450 mAh) → one 5 V/6 V UBEC (set 5 V)**, consistently
represented across `gen_wiring4.py`, `gen_connected_sch.py`, and the power-tree flowchart:

- **2S LiPo -> arming switch/fuse -> 5 V UBEC -> single +5V rail -> Pico VSYS + camera + 2 servos.**
  Every load is inside its input range: Pico VSYS 1.8–5.5 V, servos 4.8–6 V (run at 5 V), camera 5 V.
  One UBEC and one rail — a separate 6 V servo BEC isn't needed since 5 V gives the servos ~1.8 kg·cm,
  well above the ~0.9 kg·cm demand.
- **Shared-rail decoupling is mandatory** (the one real trade of a single rail): run the servo feed and
  the VSYS feed as separate star runs off the UBEC output, put **1000 µF** low-ESR bulk across the
  servo V+/GND at the servos, **100 µF** at VSYS, and an **SS34 Schottky** from rail → VSYS as a
  hold-up diode, so a ~1 A servo-stall transient can't sag VSYS enough to reset the Pico.

The isolated 9 V recovery rail present in the earlier design is **removed** with the RRC3+ (recovery
is now the motor's own ejection charge). **Verdict PASS with the decoupling above installed.**

### Power budget — datasheet estimates only (explicitly not bench-measured)

The following are vendor/datasheet-typical figures, not measurements on this hardware, and are provided only to sanity-check that the UBEC/pack are not obviously undersized:

| Load on the 5 V rail | Detail | Typical draw (datasheet/vendor, estimated) |
|---|---|---|
| Logic + sensors + camera | Pico 2 W (WiFi active) + 3x BNO085 + BMP388/BME688 + PCA9548A + i3 4K Thumb Action Camera | Pico 2 W on the order of 80-130 mA active with WiFi; each BNO085 on the order of 10-15 mA typical fusion-mode; baro sensors low single-digit mA; camera vendor-quoted around 100-150 mA recording |
| 2x servos (run at 5 V) | 2x EMAX ES08MA II (analog, as purchased) | Vendor-quoted no-load current in the tens of mA per servo, with stall current on the order of roughly 1 A per servo possible under load — two servos stalling simultaneously is the design corner case the bulk cap + Schottky hold-up is sized to ride out; the Hobbywing UBEC is 3 A-rated |

A 2S 450 mAh pack covers this aggregate draw with comfortable pad + flight endurance, and the light-pack group (LiPo ~30 g + UBEC ~10 g + i3 4K Thumb Action Camera ~36 g ≈ 76 g) sits inside the 122 g FC-bay budget. **This audit cannot certify actual current draw or the brown-out margin** — that requires a bench multimeter/current-clamp session under worst-case simultaneous load (servos slewing + camera recording + WiFi telemetry active + SD burst write), watching VSYS on a scope during a servo stall to confirm the hold-up cap/diode holds it above the Pico's brown-out threshold.

---

## 6. Ground-Test Rig GPIO Budget & DAQ Board Mismatch

### 6a. Ground-rig DAQ board — RESOLVED (2026-07): Raspberry Pi Pico / KB2040

Both ground-rig wiring blocks in `gen_wiring4.py` now specify a **Raspberry Pi Pico / Pico 2 W** DAQ
(e.g. "RPi PICO / PICO 2 W — solenoid-rig DAQ"), matching `WYVERN_E4_GSE_TestStands.md` and the BOM,
which lists an Adafruit **KB2040 (RP2040)** as the balance/bench DAQ. The **Arduino Nano** that
earlier appeared here is now an **owned bench-test auxiliary** (Already Acquired), used only for quick
breadboard checks — it is not the balance DAQ. All rig DAQ boards are therefore 3.3 V-logic RP2040/
RP2350 parts, native-compatible with the HX711 DT/SCK lines and STEMMA-QT I2C at 3.3 V; there is no
5 V↔3.3 V level-shift needed. **Verdict PASS.**

### 6b. Ground-rig GPIO budget under a Pico substitution — CAUTION, provisional

If the ground rig is redesigned around a Raspberry Pi Pico (per the deliverable spec) rather than the BOM's Nano, a first-pass GPIO budget (the Pico/Pico 2 W has 26 usable GPIO pins, as WYVERN-E4's own flight design already relies on):

| Function | GPIO pins consumed |
|---|---|
| 3x HX711 (axial Z, lateral X, lateral Y), 2 pins each (DT+SCK) | 6 |
| Gimbal-feedback BNO085 (I2C, shared with flight design's dedicated-bus pattern) | 2 |
| IRF520 solenoid PWM drive x2 (Actuator-A variant only) | 2 |
| Servo PWM x2 (Actuator-B variant only, mutually exclusive with the 2 solenoid PWM above per the BOM's "swap A/B" note) | 2 |
| **Subtotal (no VL53L4CD ring)** | **10 of 26** |
| 6x VL53L4CD ring, if added without a mux (I2C shared 2 pins + 6x XSHUT for address reassignment) | +8 (2 shared I2C + 6 XSHUT) |
| **Subtotal (with VL53L4CD ring, no mux)** | **18 of 26** |

Without the VL53L4CD ring, a Pico-based ground rig fits with wide GPIO margin (10 of 26 used) — **PASS** for that configuration. If the 6x VL53L4CD units are added to the ground rig directly (rather than only to flight, and without a PCA9548A-style mux, which is not in the ground-rig BOM), the budget still fits (18 of 26) but consumes most of the remaining headroom once a fourth static-stand HX711, any UART/USB-serial link to a host laptop, and status LEDs are added — **CAUTION, not FAIL**, but worth designing explicitly rather than assuming it fits, since this repo currently has **no** VL53L4CD integration on any rig to check against (see Finding 1d).

---

## Findings Requiring Resolution Before Flight/Bench Use

1. **Ground-rig DAQ board — RESOLVED.** The ground-rig sketches, `WYVERN_E4_GSE_TestStands.md`, and the BOM all target the **Raspberry Pi Pico** (3.3 V logic, matching the HX711/BNO085 wiring). The earlier Nano/Teensy alternative is retired.
2. **BSS138 level shifter is retired.** It existed only for the RRC3+ Comm-Port UART tap, which is removed (motor-ejection recovery). Drop it from the BOM if still listed.
3. **VL53L4CD x6 (BOM) has no driver, no XSHUT/address-reassignment code, and only one spare mux channel on the flight side.** Currently BOM-only across both flight and ground rigs. Needs a driver, an address/XSHUT plan, and (for flight) a decision on whether all 6 units are flight-relevant or ground-rig-only.
4. **Recovery is the F15-4 motor ejection charge (via bypass tube), not an electronic deploy.** The RRC3+ UART tap, its level-shift risk, and the isolated 9 V recovery rail are all removed; the primary pre-flight recovery check is now the ground-ejection/seal test in `WYVERN_E4_Recovery.md`.
5. **IRF520 solenoid drive (ground rig) has no documented PWM frequency or confirmed flyback-diode adequacy** for the 50N/12V solenoid's inductive turn-off transient in the provided documentation.
6. **All power-budget figures in Section 5 are datasheet/vendor estimates, not bench measurements.** A current-clamp/multimeter session per rail under worst-case simultaneous load is required before the power budget can be called verified rather than estimated.
7. **Three I2C0 mux channels (ch5–ch7) are unassigned** — not a defect, but worth a decision on whether they're reserved for the VL53L4CD ring (Finding 3) or left open.

---

*Audit compiled from static review of the provided WYVERN-E 4.0 firmware/wiring/documentation bundle and the given BOM. No physical hardware was measured; all "PASS" verdicts reflect internal design/documentation consistency, not bench validation.*
