# WYVERN-E — Build-Readiness Report

### A Skylight Rocketry Venture
##### Cross-check of all project files against the current (rev-latest) design — build in 1 week, launch in 2

## 1. Verdict

**GO for build.** All project files (docs, BOM, firmware, CAD, wiring, simulations, proposals) have
been reconciled to a single canonical configuration. No superseded mass/apogee/material/motor/
electronics values remain in any active file. The remaining work is *fabrication + a short bench/
ground-test punch-list* (§6), not design or documentation. The 1-week build / 2-week launch schedule
in §7 is achievable.

## 2. Canonical configuration (single source of truth)

| Parameter | Value |
|---|---|
| Airframe | 70 mm OD, ~0.74 m, single stage, 3 bays + 2 sealed bulkheads |
| Liftoff / dry mass | **705 g / 603 g** |
| Motor (flight) | **Estes F15-4** ×4 (4 s delay + ejection charge = recovery) |
| Motor (ground) | **Estes F15-0** ×13 (plugged; static + TVC balance + jetvane) |
| Commissioning | Estes/AeroTech **E16-4** ×6 |
| Apogee | **~429 ft / 130.8 m @ 6.82 s** (RK4(2e-4)+Barrowman) |
| v_max / Mach | **36.5 m/s / Mach 0.107** |
| Max acceleration | **2.67 g** net (3.66 g specific force = peak T/W) |
| T/W | **2.08 avg / 3.66 peak** |
| Fins | 4 × **72 mm** ASA-Aero, root 70 / tip 35 / LE-sweep 25° |
| Stability | CG 49.1 cm / CP 56.8 cm → **+1.10 cal** (→1.3 cal burnout), no ballast |
| Materials | **ASA-Aero**: nose, body, fins, FC + recovery bays · **PC-FR**: both bulkheads, bypass tube, engine/TVC bay, motor mount, gimbal |
| Recovery | **F15-4 motor ejection** via solid-walled bypass tube; deploy t≈7.45 s (+0.63 s past apogee, ~6.1 m/s); 18″ chute → ~6 m/s; **3.4× bay-pressurization margin**; no RRC3/9 V/CO2/e-match |
| Flight computer | **Raspberry Pi Pico 2 W (RP2350)**, dual-core, 500 Hz TVC PID **Kp0.10/Ki0.40/Kd0.18**, ±8° gimbal |
| Sensors | 3× BNO085 (GRV), BME688 + **BMP388** (Adafruit 3966) baro, microSD, i3 4K Thumb Action Camera cam, Wi-Fi bench telemetry |
| Structural margins | flight min SF ~340×; bulkhead-B ejection SF ~8×; bypass tube ~107×; engine-bay thermal < HDT |

## 3. Per-target readiness

### 3.1 Flight computer — READY to build
- Firmware sketch `wyvern4_tvc/` complete: main `.ino` + 10 header tabs, brace/paren balanced,
  no CO2/RRC3 includes (recovery is passive). PID gains margin-verified (`PID_TUNING_REPORT.md`).
- Wiring: `gen_wiring4.py` + `gen_connected_sch.py` regenerate a balanced `.kicad_sch` + preview,
  now consistent with firmware (BMP388 @3.3 V on mux ch3; GP1/GP6 spare; no 9 V/RRC3 block).
- Bench tests present: `t1_i2c_scan`, `t2_imu_grv_deflection`, `t3_servo_sweep`,
  `t4_sensors_sdlog` (now BMP388), plus `selftest.py`/`host_monitor.py` go/no-go harness.
- Telemetry: `telemetry_wifi_flight` + `telemetry_wifi_receiver` sketches.
- **PID: flight-ready, no change.** Gains **0.10/0.40/0.18** confirmed by both the frequency-domain
  margin analysis (`PID_TUNING_REPORT.md`, PM≈32.8°/GM≈9.2 dB) and a time-domain robust auto-tune
  (`PID_AUTOTUNE_REPORT.md`, within ~4% of grid-optimal; integral retained for steady-bias rejection).
- **Digital twin available:** `Simulations/wyvern_datagen/fc_sil.py` (+ GUI *Flight Computer SIL* tab)
  runs the full FC in software-in-the-loop with sensor noise and simulated Wi-Fi telemetry — use it to
  rehearse the flight and sanity-check the state machine before the pad.

### 3.2 Rocket airframe — READY to print
- All printable parts present in `3D parts/`: nose (ASA), 3 bay tubes, both sealed bulkheads
  (PC-FR, with 12 mm bypass pass-through), bypass tube (PC-FR), motor mount, 2-axis gimbal,
  72 mm fin, 1010 rail buttons, full assembly. Superseded parts moved to `_superseded/`.
- FEA (`WYVERN_E4_FEA_Structural.md`) covers flight loads, ejection pressure, and thermal.

### 3.3 Wind tunnel — REMOVED FROM PROGRAM (2026-08)
- The Hofferth (2025) modular tunnel and the airfoil-CFD package that fed it are **out of scope.**
  BOM §11 (tunnel) is struck; the tunnel STLs, print plates, and fan collar are not built.
- Aerodynamic characterization is now carried by the Barrowman/drag-buildup model in the flight-sim
  suite, cross-validated against flight telemetry and the instrumented ground stands. See
  `CONFLICTS.md` §7 for the scope-change record.

### 3.4 Motor test stands — READY to print/build
- **TVC thrust-vector balance**: base, thrust block, flexure template (PC-FR).
- **Static/jetvane stand**: base plate, load-cell bracket, motor tower, steel blast deflector.
- DAQ: Raspberry Pi Pico + load cells/HX711 (BOM §10); ground-rig sketches target Pico.

## 4. Bill of materials

`Documentation/WYVERN_E4_BOM.xlsx` — **8 live sections** spanning **all three build targets**: FC &
sensors, power, TVC (flight servo + ground magnetic A/B), recovery (motor ejection), propulsion
(F15-4 flight / F15-0 ground / E16-4 commissioning), airframe filament (ASA-Aero + PC-FR),
harness/connectors/prototyping, and the TVC balance + static stand. **Former §9 (Hofferth wind
tunnel) has been deleted** per the 2026-08 scope change, and the gross/net formulas rewired to the
eight surviving subtotals. Every live line has a purchase link and verified price. Filament
allocation matches the material zoning in §2.

| Budget line | Value |
|---|---|
| Gross to-buy | $1,290.18 |
| Less reimbursed (launch controller) | −$43.99 |
| **Net out-of-pocket (still to buy)** | **$1,246.19** |
| Already acquired (owned) | $479.27 |
| **Total program spend** | **$1,725.46** (was $1,882 with the tunnel) |

## 5. Files reconciled in this pass

- Fin geometry aligned to **72 mm** in `we4_flightsim`, `we4_validation`, `we4_deepsim`,
  `build_ork4.py` (`.ork` fin height was 60 mm — fixed).
- **Teensy → Pico 2 W** purged from `README.md`, `WYVERN_E4_Camera_Solution.md`,
  `WYVERN_E4_GSE_TestStands.md`, and proposal rev1 §5.1 (kept only as historical "it replaces"
  contrast in the FC spec).
- **BMP280 → BMP388** aligned across wiring generators (now 3.3 V, not 5 V), the regenerated
  schematic + preview, `t4_sensors_sdlog.ino`, test READMEs/selftest, and the audit docs.
- Ground-rig DAQ conflict marked **resolved** (Pico everywhere) in `COMPATIBILITY.md`.
- `baro.h` apogee comment corrected to 429 ft / 130.8 m.
- Full-project sweep confirms **zero** surviving 708/662/648 g, 432 ft, 2.07/3.62 T/W, 58 mm fin,
  CG 46.7 / CP 52.5, or F15-0-as-flight references outside intentional "superseded/removed" notes.

## 6. Pre-flight punch-list (bench + ground test — the only open items)

These are hardware-verification steps from `FLIGHT_READINESS.md` §4 — none are design changes:

1. **Ground-test the ejection gas path** — fire a representative charge; confirm the friction-fit
   nose releases cleanly, the chute deploys, and both bulkhead seals + bypass-tube joints are
   gas-tight. *(Single point of the recovery system — do this first.)*
2. **Confirm LAUNCH_IRQ (GP7) wiring** — wire the redundant inertial switch or remove the branch.
3. **Confirm RBF sense polarity** — verify `HB:...rbf=` matches the switch state.
4. **Verify the 2S LiPo divider** (GP26, 100 k/62 k) against a multimeter within ~2 %, and scope VSYS during a servo stall to confirm the bulk-cap + SS34 hold-up keeps it above the Pico brown-out threshold.
5. **Confirm SH2_ACCELEROMETER** report enables on your BNO085 firmware revision (launch/landing
   detect depend on it).
6. **Servo throw / gimbal mechanical limit** — confirm the printed gimbal + linkage allow the ±8° travel (raised from ±5° for wind authority).
7. **Commission each stand** with ≥2 E16-4 firings before F15-0 data runs (curve vs. published).

Run `selftest.py` before every flight; it gates on all of the above that are observable in software.

## 7. Suggested 1-week build / 2-week launch schedule

**Week 1 — fabricate & bench:**
- Days 1–2: print airframe (ASA-Aero body/nose/fins; PC-FR bulkheads/tube/engine bay/gimbal/mount)
  and both stands. Order-long-lead items already in BOM.
- Days 3–4: assemble FC (Pico 2 W + sensors on perfboard), wire per the schematic, flash firmware,
  run `t1`–`t4` bench tests + `selftest.py`.
- Days 5–7: assemble rocket; install recovery (chute + bypass tube + Nomex); commission stands with
  E16-4; run **ground ejection test** (punch-list #1).

**Week 2 — ground data & fly:**
- Days 8–10: F15-0 static thrust-curve verification + jetvane materials screen; F15-0 TVC balance
  A/B (servo vs. magnetic) on the 3-axis stand; lock the flown actuator (servo).
- Days 11–12: full preflight `selftest.py` GO; range procedures (remote ignition, ≥3 m standoff,
  igniter installed last).
- Days 13–14: **launch on F15-4** (FAA Class-1, no waiver; 705 g < 1500 g). Recover, pull SD +
  Wi-Fi logs, feed flight data back into `Simulations/` for post-flight validation.

## 8. Notes / residual risk

- Recovery is a **single passive event** (motor charge) with no electronic backup — the 3.4×
  pressurization margin and the ground ejection test (punch-list #1) are what retire that risk.
- The vehicle is authority-limited in strong wind (documented low-speed weathercocking); the
  atmospheric dataset + PID tuner in `Simulations/wyvern_datagen/` quantify this — prefer a
  low-wind launch window.
- Because runs no longer overwrite, `Simulations/` and dataset folders will accumulate timestamped
  outputs; prune as needed.

---

## 9. 2026-08 rerun & reconciliation pass

Everything below was regenerated or corrected in a single pass. The verdict in §1 is unchanged
(**GO for build**); the numbers moved because the motor model was corrected, not because the design
changed.

### 9.1 Scope change

The **wind tunnel and the airfoil-CFD package are removed from the program** (`CONFLICTS.md` §7).
`Simulations/CFD/` is deleted, BOM §9 is deleted, and the proposal's five research questions
consolidate to four — the two tunnel-only questions become a single question on how accurately a
Barrowman-class model predicts the as-built vehicle's passive aerodynamics, scored against flight
telemetry. Every surviving question still has two independent methods (Proposal §3, Table 0).

### 9.2 Defects found and fixed

| # | Defect | Effect | Fix |
|---|---|---|---|
| 1 | F15 thrust curve renormalized to 49.6 N·s from a shape integrating to 41.97 N·s | peak thrust inflated to **29.9 N** vs Estes' published 25.3 N; peak T/W read 4.32 against the 3.66 quoted repo-wide | sustain block lifted +2.4408 N so the curve matches impulse, peak **and** average simultaneously |
| 2 | `fc_sil.py` fed the launch detector kinematic acceleration, not specific force | peaked at 2.65 g against the firmware's 3 g latch — the SIL state machine **never left ARMED** in any flight: no BOOST, no TVC, no deploy, ~70 m/s ballistic "touchdown" in every logged run | accelerometer now reports specific force (peaks at 3.66 g = peak T/W) |
| 3 | `we4_sim.py` hard-coded **Kp=8.0/Ki=1.5/Kd=1.2** | the TVC plot disagreed with the firmware, `pid_reference.py`, and `CONFLICTS.md` §1, which record those gains as superseded and unstable | gains set to the frozen 0.10/0.40/0.18 |
| 4 | `we4_deepsim.py` used **CG 0.467 / CP 0.537**; `we4_validation.py` used **CG 0.467** | every margin, flutter, CG-tolerance and stability gate was scored against a pre-ASA-Aero, pre-camera vehicle; validation reported 1.44 cal where the real margin is 1.10 | both set to the canonical 0.491 / 0.568 |
| 5 | `we4_stability.py` reported `fin_span_mm: 35.0` while evaluating `s35=0.055` | the "35 mm fins are unstable (−0.52 cal)" finding was quoting a **55 mm** fin; the flown 72 mm fin was never evaluated | file rewritten; 35 mm is **−0.99 cal (unstable)**, flown 72 mm reproduces 49.1/56.8/+1.10 exactly |
| 6 | Deploy sampled at **t = 4.0 s** in three files | reported a 29 m/s deploy against the Recovery doc's ~6.5 m/s — the retired finless-era electronic timer, not motor ejection | deploy is now t = burnout + 4 s = **7.45 s**; integration runs through apogee to reach it |
| 7 | Three different Cd for one vehicle (0.50 / 0.58 / 0.539) | apogee disagreed between scripts | unified to the **0.539** componentwise buildup |
| 8 | Mass stack summed to **606 g / 708 g** | contradicted the 603/705 used everywhere, including the FAA Class-1 argument | harness estimate trimmed 25→22 g; dry total is now exactly 603 g |
| 9 | `we4_deepsim.py` battery pack **850 mAh** | no such pack exists in the BOM or power tree | set to the Zeee 2S **450 mAh** → 40 flights/charge |
| 10 | Generic motor shapes normalized to impulse only | published peaks wrong by up to 2× (D12 read 14.2 N vs 29.7 N) — the number that sizes the load cell | decay rate solved per motor; all five motors now match published impulse and peak |
| 11 | `build_ork4.py` gave ASA-Aero nose/fins **PC-FR density**, and announced a 150 g-ballast config | the `.ork` cross-check modelled the wrong vehicle | densities corrected to 650 kg/m³; no ballast |
| 12 | Control-authority margin swept from t=0 to burnout | both endpoints are thrust-zero, so the reported minimum was always exactly 0.0 mN·m | swept over the TVC-active window → **71.7 mN·m** |

### 9.3 Fidelity increases

- **Integrator:** semi-implicit Euler → RK4 throughout; `we4_flightsim` dt 1e-3 → 2e-4.
- **Atmosphere:** exponential density → ISA troposphere with real lapse rate.
- **Wind:** scalar mean → power-law shear with per-flight roughness exponent + Dryden-form
  turbulence (the old model was a single deterministic sinusoid, coherent across the whole ensemble).
- **Dispersion:** atmosphere only → atmosphere **plus** liftoff mass, CG station, Cd, total impulse
  and thrust-axis misalignment.
- **TVC loop:** first-order servo → second-order servo with slew-rate limit, explicit transport
  delay, 500 Hz zero-order hold, filtered derivative, and a noisy/biased/quantized gyro.
- **Ground stands:** ideal load cells → full signal chain — mount resonance (42 Hz, ζ=0.035),
  HX711 quantization and sample-rate aliasing, thermal zero drift, calibration-slope residual, and a
  returned uncertainty budget. Flags that the 42 Hz mount ring **aliases** against the 80 SPS
  Nyquist of 40 Hz: stiffen the mount or filter before trusting peak thrust.
- **Datasets:** regenerated at 6.0 M rows / 344 MB across 50 shards, widened from 23 to 34 columns
  (build dispersion and rail-exit/coast-Cd are now first-class), every file ≤ 26 MB.

### 9.4 Post-rerun canonical numbers

| Quantity | Value |
|---|---|
| Apogee | **130.8 m / 429 ft @ 6.82 s** (was 435 ft) |
| Burnout | 3.45 s, 72.8 m, 35.7 m/s |
| v_max / Mach | 36.5 m/s / **Mach 0.107** (docs previously said "Mach ~0.4") |
| Max acceleration | 2.67 g net (3.66 g specific force) |
| CG / CP / margin | 49.1 cm / 56.8 cm / **+1.10 cal** |
| Deploy | t = 7.45 s, +0.63 s past apogee, 6.1 m/s |
| PID margins | PM **32.8°**, GM **9.2 dB**, worst gust pitch **1.31°**, gimbal 1.68° |
| Gates | validation **10/13**, deepsim **8/8** |
| Cross-file check | **14/14** numeric agreements between the summary JSONs |

The three flagged validation gates are unchanged in character and share one root cause: the F15 is
underpowered for a 705 g vehicle. Rail exit is 6.1 m/s against a 15 m/s rule of thumb, peak T/W is
3.66 against a 5.0 rule of thumb, and weathercock reaches 63° at 10 m/s. This is a launch-window
constraint, not a design defect — but it is real and should not be presented as passing.
