# WYVERN-E 4.0 — Build-Readiness Report

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
| Apogee | **~435 ft / 133 m @ ~6.81 s** (RK4+Barrowman) |
| T/W | **2.08 avg / 3.66 peak** |
| Fins | 4 × **72 mm** ASA-Aero, root 70 / tip 35 / LE-sweep 25° |
| Stability | CG 49.1 cm / CP 56.8 cm → **+1.10 cal** (→1.3 cal burnout), no ballast |
| Materials | **ASA-Aero**: nose, body, fins, FC + recovery bays · **PC-FR**: both bulkheads, bypass tube, engine/TVC bay, motor mount, gimbal |
| Recovery | **F15-4 motor ejection** via solid-walled bypass tube; deploy t≈7.45 s (+0.64 s past apogee, ~6.3 m/s); 18″ chute → ~6 m/s; **3.4× bay-pressurization margin**; no RRC3/9 V/CO2/e-match |
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
  margin analysis (`PID_TUNING_REPORT.md`, PM≈33°/GM≈9.3 dB) and a time-domain robust auto-tune
  (`PID_AUTOTUNE_REPORT.md`, within ~4% of grid-optimal; integral retained for steady-bias rejection).
- **Digital twin available:** `Simulations/wyvern_datagen/fc_sil.py` (+ GUI *Flight Computer SIL* tab)
  runs the full FC in software-in-the-loop with sensor noise and simulated Wi-Fi telemetry — use it to
  rehearse the flight and sanity-check the state machine before the pad.

### 3.2 Rocket airframe — READY to print
- All printable parts present in `3D parts/`: nose (ASA), 3 bay tubes, both sealed bulkheads
  (PC-FR, with 12 mm bypass pass-through), bypass tube (PC-FR), motor mount, 2-axis gimbal,
  72 mm fin, 1010 rail buttons, full assembly. Superseded parts moved to `_superseded/`.
- FEA (`WYVERN_E4_FEA_Structural.md`) covers flight loads, ejection pressure, and thermal.

### 3.3 Wind tunnel — READY to print/build
- Hofferth (2025) modular design: 84 STLs + Bambu print plates (0.4 mm & 0.8 mm nozzle) +
  120 mm fan collar + source PDF + `Wind Tunnel/README.md`. Itemized in BOM §11.

### 3.4 Motor test stands — READY to print/build
- **TVC thrust-vector balance**: base, thrust block, flexure template (PC-FR).
- **Static/jetvane stand**: base plate, load-cell bracket, motor tower, steel blast deflector.
- DAQ: Raspberry Pi Pico + load cells/HX711 (BOM §10); ground-rig sketches target Pico.

## 4. Bill of materials

`Documentation/WYVERN_E4_BOM.xlsx` — 11 sections spanning **all four build targets**: FC & sensors,
power, TVC (flight servo + ground magnetic A/B), recovery (motor ejection), propulsion
(F15-4 flight / F15-0 ground / E16-4 commissioning), airframe filament (ASA-Aero + PC-FR),
harness/connectors/prototyping, TVC balance + static stand, and the Hofferth wind tunnel. Every
line has a live purchase link and verified price. Filament allocation matches the material zoning
in §2.

## 5. Files reconciled in this pass

- Fin geometry aligned to **72 mm** in `we4_flightsim`, `we4_validation`, `we4_deepsim`,
  `build_ork4.py` (`.ork` fin height was 60 mm — fixed).
- **Teensy → Pico 2 W** purged from `README.md`, `WYVERN_E4_Camera_Solution.md`,
  `WYVERN_E4_GSE_TestStands.md`, and proposal rev1 §5.1 (kept only as historical "it replaces"
  contrast in the FC spec).
- **BMP280 → BMP388** aligned across wiring generators (now 3.3 V, not 5 V), the regenerated
  schematic + preview, `t4_sensors_sdlog.ino`, test READMEs/selftest, and the audit docs.
- Ground-rig DAQ conflict marked **resolved** (Pico everywhere) in `COMPATIBILITY.md`.
- `baro.h` apogee comment corrected to 435 ft / 133 m.
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
- Days 1–2: print airframe (ASA-Aero body/nose/fins; PC-FR bulkheads/tube/engine bay/gimbal/mount),
  wind tunnel, and both stands. Order-long-lead items already in BOM.
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
