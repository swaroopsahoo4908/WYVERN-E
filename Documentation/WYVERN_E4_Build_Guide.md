---
updated_at: 2026-08-01
---

# WYVERN-E — Build, Bench and Range Guide

### A Skylight Rocketry Venture
##### Companion to `WYVERN_E4_Timeline_14Day.md`. Sections map to timeline days.

Every parameter below is the frozen value from `CONFLICTS.md` §5. If this guide and that table ever
disagree, **the table wins** — and the disagreement is a defect worth fixing before you build.

---

## A · Print (Timeline Days 4–6)

### A1 Material zoning — get this right or the thermal margin is gone

**⚠ Airframe is now two body tubes (Lower BT, Upper BT) joined at one bulkhead, not the old
single-tube 3-bay layout — see `WYVERN_E4_Recovery.md` §1.** The old solid-walled bypass tube no
longer exists; the single bulkhead is now the separation joint and takes the ejection gas directly.

| Part | Material | Why |
|---|---|---|
| Nose cone, both body tubes, 4 fins | **PLA** (foamed, ~0.65 g/cm³) | No motor heat. This is where the ~130 g mass saving comes from. |
| Bulkhead (single separation joint) | **PETG-CF** | Takes the ~140 kPa ejection pulse directly and is now sized to **release** at a target force, not just survive one — see `WYVERN_E4_FEA_Structural.md` §4 (open item). |
| TVC bay, motor mount, gimbal | **PETG-CF** | Sustained plume heating. |

### A2 Print settings

- Nozzle 0.4 mm, layer 0.2 mm, 4 walls, 25% gyroid infill.
- **Fins: print flat, 100% infill, and orient so layer lines run spanwise** — a fin that delaminates
  along a chordwise layer line at 36 m/s is a lost vehicle.
- PETG-CF needs an enclosure and a 100–110 °C bed. Dry it first; PC absorbs water and prints badly wet.
- PLA foams on extrusion — run the vendor's recommended flow, not your usual ASA profile.

### A3 Print checks before you assemble

- [ ] Body tubes: check OD 70 mm and roundness at three stations. Ovality kills the friction fit at the bulkhead joint.
- [ ] Bulkhead: servo-extension and STEMMA-QT cable pass-through holes are clear and to size (see `WYVERN_E4_Recovery.md` §1, §5).
- [ ] Fin root: fits the slot without forcing. Forced = pre-stressed = cracks under load.
- [ ] Gimbal: pivots freely through the full range with no binding. **Check this before wiring.**
- [ ] Weigh every part and compare against `3D parts/_generator/mass_report.json`.
      More than ~10% over on a bay tube means over-extrusion, and it comes straight off your margin.

---

## B · Bench bring-up (Timeline Day 7) — Gate 2

Run these in order. **Do not skip ahead**; each test assumes the previous one passed.

### B1 Toolchain

Arduino IDE → Board Manager → install **Arduino-Pico (earlephilhower)**. Select
*Raspberry Pi Pico 2 W*. Library Manager → install `Adafruit_BNO08x`, `Adafruit_BMP3XX`,
`Adafruit_BME680`. `Servo`, `Wire`, `SPI`, `SD`, `WiFi` ship with the core.

```
cd "Flight Computer/test_code"
python3 -m pip install pyserial
```

### B2 `t1_i2c_scan` — is anything on the bus?

Expect: PCA9548A at **0x70** on I2C0, and the gimbal BNO085 at **0x4A** on I2C1.

| Symptom | Cause |
|---|---|
| Nothing on I2C0 | Mux unpowered, or SDA/SCL swapped (GP16 = SDA, GP17 = SCL) |
| 0x70 but no channel devices | Mux channel not selected, or sensor 3V3 not connected |
| Nothing on I2C1 | GP18/GP19 swapped, or the gimbal IMU is on the mux by mistake — it must be on its **own** bus |

### B3 `t2_imu_grv_deflection` — do the IMUs agree?

Enables Game Rotation Vector on all three. Hold the airframe still: all three quaternions should
agree within ~1°. Rotate the gimbal by hand: the computed deflection should track it.

> **Magnetometer must stay off.** GRV mode is accel+gyro only, deliberately — the servos' magnets
> will corrupt any magnetically-referenced heading.

### B4 `t3_servo_sweep` — **calibrate the linkage. Do not skip this.**

This is the step that catches the class of bug that made the firmware clamp to ±5° while every
document claimed ±8°.

1. Flash `t3_servo_sweep.ino`, open Serial Monitor at 115200.
2. Send `c`. Adjust the linkage so the nozzle sits **mechanically centred** at 1500 µs.
3. Send `p`. The servo steps to +8°, 0°, −8°, holding 3 s at each.
4. **Measure the actual nozzle angle** at the +8 and −8 holds with a digital protractor.
5. If measured ≠ 8.0°, set `SERVO_LINKAGE_RATIO = 8.0 / measured_deg`, reflash, repeat from 2.
6. Send `y`, repeat for yaw.
7. Send `s` for the continuous sweep. **Listen** — buzzing or stalling at the extremes means the
   linkage binds before the commanded limit.
8. **Copy the final `SERVO_LINKAGE_RATIO` into `wyvern4_tvc.ino`.** Record it in the build log.

- [ ] Pitch reaches ±8.0° measured, no binding
- [ ] Yaw reaches ±8.0° measured, no binding
- [ ] `SERVO_LINKAGE_RATIO` copied into the flight sketch

### B5 `t4_sensors_sdlog` — does the card actually take data?

Confirm BMP388 and BME688 both read, and that a file appears on the card with plausible rows.
Use a **class 10 or better** card. A slow card is the one thing that can still make the log ring
back up (watch `peak=` in the heartbeat).

### B6 Full self-test — Gate 2

```
python3 selftest.py /dev/ttyACM0        # or /dev/tty.usbmodemXXXX on macOS
```

Every row must be PASS (WIFI may be SKIP, RBF may be WAIT until you pull the pin):

```
MUX · IMU_GIMBAL · IMU_BODY · IMU_RECOVERY · IMU_MINIMUM · BARO_BMP · BARO_BME
SERVO · CORE0_READY · BATTERY · SD · WIFI · RBF · LOG_RING
```

**`LOG_RING` is new and it matters.** It verifies core 1 is actually draining core 0's log ring.
Until this pass the transport dropped 100% of frames while reporting nothing wrong — a flight would
have produced a CSV containing only a header. If `LOG_RING` fails, do not fly; you will get no data.

Watch the heartbeat: `HB:... drop=0 pend=<small> peak=<small>`. `peak` climbing toward 256 means the
card can't keep up.

- [ ] **`>>> PREFLIGHT GO <<<`**

### B7 Bench gotchas

| Symptom | Cause |
|---|---|
| Never leaves BOOT | Self-test failed, RBF still inserted, or battery below 6.0 V |
| `BATTERY: FAIL` on a good pack | GP26 divider not wired — the ADC pin is floating. It is in the schematic now; check you built it. |
| Instant launch detect on the pad | GP7 LAUNCH_IRQ floating low. Wire the switch or remove the branch. |
| `drop` climbing | Slow SD card. Raise `FLUSH_EVERY` or use a faster card. |

---

## C · Airframe assembly (Timeline Day 8)

1. **Bulkhead joint.** Route the servo extensions and STEMMA-QT cable through the pass-through holes
   before final fit — decide slack vs. breakaway connector per `WYVERN_E4_Recovery.md` §5 first, this
   is much harder to redo after the tubes are together. The joint itself is friction-fit/shear-pinned
   to **release** under ejection pressure, not sealed shut — see `WYVERN_E4_FEA_Structural.md` §4 for
   the (still-open) release-force sizing.
2. **TVC bay.** Motor mount + gimbal into the Lower BT. Confirm the gimbal still moves freely after
   bonding.
3. **Fins.** Four, 90° apart. **Alignment matters more than strength here** — a misaligned fin set
   induces roll, and roll couples into the pitch/yaw loops the whole experiment is measuring.
4. **Rail buttons**, 1010, aligned along one fin line.
5. **Recovery.** Chute, 1/8″ Kevlar shock cord, Nomex protector — wadding on both faces of the
   bulkhead joint. Pack so the chute cannot snag the cable pass-through.
6. **Bulkhead release-force check.** Test the joint's separation force by hand before flight: it
   should take firm but not two-handed effort, targeting the 50–150 N band in
   `WYVERN_E4_Recovery.md` §4.

---

## D · Ground separation test (Timeline Day 9) — Gate 3

**Do this before you fly. It is the single point of failure in the entire recovery system.**

The FC does not fire anything — recovery is the F15-4's own charge, separating the two body tubes at
the bulkhead joint. There is no electronic backup. If this path doesn't work, the vehicle comes down
ballistic.

1. Assemble fully, chute packed, **no flight battery, no camera** (don't risk them).
2. Fire a representative ejection charge (or the motor's own charge, restrained) into the Lower BT.
3. Confirm: bulkhead joint releases cleanly in the 50–150 N band · chute deploys fully · servo/
   STEMMA-QT cable pass-through survives the separation without snagging or damage · no scorching
   past the wadding.
4. If the joint sticks: reduce the friction fit. If it releases too easily: increase it.
   **Retest after any change.**

- [ ] Clean separation, chute deployed, cable pass-through intact

---

## E · Ground test stands (Timeline Days 7–9)

### E1 Dead-weight calibration — Gate 4, before any motor

Never put a motor near an uncalibrated stand.

1. Hang known masses spanning the expected range on each channel (axial to ~25 N, lateral to ~4 N).
2. Record counts vs force, fit the transfer function per channel.
3. The 3-axis balance needs the **full 3×3 matrix**, not three independent scalars — the flexures
   are cross-coupled. `Documentation/derive_math.py` derives it; `MATH_DERIVATIONS.md` explains it.
4. Expect the fit to resolve thrust to <1 mN and deflection to ~0.01°.

- [ ] All channels calibrated, residual < 1%

### E2 Commissioning — Gate 5, 2 × E16-4 per stand

Compare the measured curve against the published E16 curve. Within ~10% on total impulse and peak.

> **Watch for mount ringing.** The bench model (`wyvern_datagen/bench_sim.py`) predicts a **42 Hz**
> mount resonance, and at the HX711's 80 SPS rate the Nyquist is 40 Hz — so the ring **aliases** and
> will appear as a spurious low-frequency wobble on the thrust trace. If you see it: stiffen the
> mount, or drop the HX711 to 10 SPS and accept the lower bandwidth. Do not just smooth it away —
> you'd be smoothing away real ignition-transient data too.

### E3 RQ1 — actuator A/B (Timeline Day 9)

The program's headline experiment. Same fixture, same electronics, same control law; only the
actuator changes.

For each actuator class (magnetic-solenoid, then servo), 3 × F15-0:
- Step commands: 0 → +5°, hold, → −5°, hold
- Ramp: continuous sweep across the full ±8°
- PID disturbance rejection: 3° mount tilt, let the loop null it

Extract per actuator: **bandwidth, slew rate, step overshoot, steady-state error, max achievable
deflection.** Log commanded vs measured throughout — that difference is the result.

> **While you are here, settle the servo torque question.** The deep-sim now reports only **2.3×**
> margin against servo stall at the full ±8° (the old 3.7× was computed at 5°, understating the
> hinge load by 1.6×). This rig measures it directly. At the ±8° extremes under thrust, record
> servo current and whether the nozzle actually **holds** the commanded angle. If it droops:
> reduce the linkage ratio, or move the servo rail to 6 V, or fit a higher-torque servo — in that
> order. See `WYVERN_E4_BUILD_READINESS.md` §11.

### E4 RQ2 — PLA vs PETG-CF (Timeline Days 7 and 9)

**Jetvane testing is dropped** (2026-08). RQ2 is now the material justification for the zoning
actually flown.

- **Three-point bend** (Day 7, no motor needed): PLA and PETG-CF coupons, identical print
  parameters — same nozzle, layer height, wall count, infill. Report flexural modulus and
  stiffness. This is the whole reason the TVC assemblies are not PLA.
- **Engine-bay wall temperature** (Day 9, during the static fires): tape a thermocouple to the
  inside of the engine-bay wall and log peak temperature across the 3.45 s burn. Compare against
  PLA's ~55 °C HDT and PETG-CF's ~80 °C. The model predicts a ~40 °C wall — if the measurement is
  much higher, that is a finding worth reporting and it validates the zoning decision directly.

---

## F · Range procedure (Timeline Day 11)

### F1 Go / no-go — all must be YES

- [ ] Wind < 5 m/s, gusts < 2 m/s above mean *(hard stop 7 m/s)*
- [ ] Ceiling > 300 m, no precipitation
- [ ] Ejection test passed (Gate 3)
- [ ] `selftest.py` → **PREFLIGHT GO** in flight configuration
- [ ] Battery > 7.4 V
- [ ] Mass 705 ± 15 g, CG 49.1 ± 1 cm from nose — **measured, not assumed**
- [ ] SD card inserted, empty, seated
- [ ] Camera charged, lens clean
- [ ] RSO briefed, range clear, ≥ 3 m standoff
- [ ] **Anemometer ready — you must record the surface wind at launch** (RQ3 needs it)

### F2 Pad sequence

1. Rocket on the rail, rail buttons free-sliding.
2. Power the FC. **Igniter is NOT installed yet.**
3. Run `selftest.py`. Confirm GO.
4. Pull RBF → state goes ARMED, camera starts.
5. Confirm heartbeat: `state=ARMED batt>7.4V rbf=1 drop=0`.
6. **Record wind speed and direction. Write it down.**
7. Clear the pad. Install the igniter last.
8. Launch on the RSO's call.

### F3 Immediately after recovery

1. Photograph the vehicle **before touching it** — fin condition, nose seat, any scorching.
2. Note where it landed relative to the pad (drift, for the dispersion check).
3. Power down, pull the SD card.
4. **Copy the CSV to two places before doing anything else.**
5. Note anything anomalous while it's fresh: sounds, visible tumble, deployment timing.

### F4 Same-evening reduction

```
cd Simulations
python3 we4_flight_reduce.py FLIGHT_A.csv FLIGHT_B.csv \
        --wind <measured_m_s> --label gainA gainB
```

Produces the RQ3 and RQ4 numbers, the health check, and the figures, into `plots_flight/`.

**Sanity-check the health block before you trust anything else:**

| Field | Expect | If not |
|---|---|---|
| `dropped_frames` | 0 | Data has gaps — note in the paper |
| `loop_dt` median | ~2000 µs | Control loop didn't hold 500 Hz |
| `loop_overrun_pct` | < 1% | Investigate before trusting RQ4 |
| `batt` start → min | > 7.0 V | Sag under servo load; check decoupling |

---

## G · Gain sets for RQ4

| Flight | Kp | Ki | Kd | Rationale |
|---|---|---|---|---|
| A | **0.10** | **0.40** | **0.18** | Flight gains. PM 44.7°, GM 12.6 dB at all 24 operating points. |
| B | 0.10 | 0.05 | 0.05 | The margin-search alternative: much higher margin (PM 49.7°) but ~2.3× the gust deviation. Tests whether extra stability margin costs real tracking performance. |

Change gains **only** in `wyvern_pid_defaults` in `wyvern_pid.h`. Reflash, re-run `selftest.py`,
and **record which gain set is on the vehicle** — mixing this up loses RQ4 entirely.

---

## H · If something goes wrong

| Symptom | Likely cause | Action |
|---|---|---|
| CSV has only a header | Log ring not draining | Check `LOG_RING` in self-test. Do not refly until it passes. |
| No BOOST in the log | Launch detect never fired | Check accel report enabled; check the 3 g threshold against actual peak T/W of 3.66 |
| Gimbal never moved | TVC inhibited | Check state reached BOOST and t ≥ 0.5 s; check servo power |
| Deflection clipped at 5° | The old ±5° clamp is back | Confirm you flashed the fixed `wyvern4_tvc.ino` |
| Chute didn't deploy | Ejection path blocked | **Do not refly.** Repeat Gate 3 ground ejection test. |
| Pitch > 30° | Weathercock in too much wind | Check the recorded wind against the 5 m/s limit |

---

## I · Reference — frozen parameters

| Parameter | Value |
|---|---|
| Liftoff / dry mass | 792 g / 690 g |
| CG / CP / margin | 48.4 cm / 56.8 cm / +1.20 cal |
| Apogee | 98.9 m (324 ft) @ 6.27 s |
| Burnout | 3.45 s, 59.1 m, 28.9 m/s |
| v_max / Mach | 29.7 m/s / M0.087 |
| Deploy | t = 7.45 s, +1.18 s past apogee, 11.5 m/s |
| Descent | 5.0 m/s under 24″ chute |
| PID | Kp 0.10 / Ki 0.40 / Kd 0.18, τ_d 0.02 s, i_lim 0.4 |
| Gimbal limit | **±8.0°** |
| Control rate | 500 Hz, core 0 |
| Launch detect | \|a\| > 2 g sustained ≥ 50 ms |
| Battery | 2S 450 mAh, warn 6.4 V, inhibit 6.0 V |

Full pin map: `CONFLICTS.md` §5.
