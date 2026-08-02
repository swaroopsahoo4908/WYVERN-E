# WYVERN-E

A single-stage, 70 mm, **finned active-TVC sustainer** demonstrating closed-loop thrust-vector
control on a **Raspberry Pi Pico 2 W (RP2350)** flight computer, powered by the **Estes F15-4**. The two TVC actuation
methods (magnetic-solenoid vs servo) are compared **on the ground** on a 3-axis thrust-vector
balance; the vehicle flies the servo system. Ground support is the 3-axis TVC balance plus the
static thrust + materials (jetvane) stand.

> **Scope change (2026-08):** the wind tunnel and the airfoil-CFD work that fed it are **removed
> from the program.** Aerodynamic characterization now rests on the Barrowman/drag-buildup model
> inside the flight-sim suite, cross-validated against flight telemetry and the instrumented
> ground stands. See `Documentation/CONFLICTS.md` §7.

A Skylight Rocketry venture. *Supersedes 3.0 (two-stage Pi-5 vehicle).* Completely off-the-shelf,
no custom PCBs — a single Raspberry Pi Pico 2 W (RP2350) runs everything bare-metal.

## Why this is simpler than 3.0
- **Single stage, single Raspberry Pi Pico 2 W (RP2350)** = flight computer *and* real-time
  controller. Dual-core (one core dedicated to the 500 Hz control loop), no Linux, native hardware
  PWM, deterministic control, far lighter and lower power.
- **A/B TVC comparison moved to the ground** (3-axis balance, repeatable, measures the thrust vector
  directly) — a better experiment than flying both. The vehicle flies servo-only.
- **Materials:** PC-FR only where there's motor heat (nose, engine/TVC bay, Bulkhead A); **ASA-Aero**
  everywhere else (body tube, FC & recovery sections, Bulkhead B) — saves ~100 g.

## Key recalculated numbers (see `Simulations/we4_sim.py` → `plots4/`)
| | value |
|---|---|
| Liftoff mass | **705 g** (finned 72 mm, no ballast) | ASA-Aero main airframe; PC-FR only at bulkheads/tube/engine (was 812 g all-PC-FR) |
| T/W | **2.08 avg / 3.66 peak** |
| CG / gimbal pivot / control arm | 49.1 cm / 62 cm / **12.9 cm** from nose |
| Pitch inertia Iyy | 0.0209 kg·m² |
| Burnout | 3.45 s · 72.8 m · 35.7 m/s |
| Apogee | **~429 ft / 130.8 m** (RK4+Barrowman, stable +1.10 cal) @ 6.82 s |
| Recovery | F15-4 motor ejection via bypass tube; ejects t≈7.45 s (+0.63 s past apogee) @ ~6.1 m/s; 18″ chute → 6.2 m/s descent |
| TVC | gimbal stays within ±8°; control authority positive throughout the burn |

> **Apogee/deploy note:** the ASA-Aero airframe (PC-FR only at bulkheads/tube/engine) drops liftoff to
> 705 g and lifts apogee to ~429 ft. Recovery is the F15-4 motor ejection charge (fixed 4 s delay),
> firing +0.63 s past apogee at a gentle ~6.1 m/s — no timer to retune. The lighter ASA nose moved the
> CG aft, so fins were grown 58→72 mm to hold the 1.0-cal margin without ballast.

## Repository structure

```
WYVERN Project/
├── README.md                        ← this file
├── .gitignore
├── Documentation/                   ← all engineering docs, BOM, and build readiness
│   ├── README.md                    ← documentation index
│   ├── WYVERN_E4_BUILD_READINESS.md ← GO/NO-GO reconciliation report
│   ├── WYVERN_E4_Mathematics.md     ← mass/CG/inertia, T/W, trajectory, TVC, recovery
│   ├── WYVERN_E4_Stability_FinSizing.md
│   ├── WYVERN_E4_FEA_Structural.md
│   ├── WYVERN_E4_Recovery.md
│   ├── WYVERN_E4_Camera_Solution.md
│   ├── WYVERN_E4_GSE_TestStands.md
│   ├── WYVERN_E4_PID_AUTOTUNE_REPORT.md
│   ├── WYVERN_E4_BOM.xlsx           ← master BOM (8 sections) + purchase links
│   ├── WYVERN_E4_Timeline_14Day.md  ← day-by-day build-to-flight schedule
│   ├── WYVERN_E4_Build_Guide.md     ← print/bench/assembly/ground-test/range procedures
│   ├── FLIGHT_READINESS.md
│   ├── COMPATIBILITY.md
│   └── CONFLICTS.md
├── Flight Computer/                 ← Pico 2 W spec, firmware, wiring, GSE test rigs
│   └── README.md
├── Simulations/                     ← Python RK4 suite, OpenRocket, dataset generator
│   ├── we4_flight_reduce.py         ← post-flight SD log → RQ3/RQ4 results (--selftest first)
│   ├── README.md
│   └── wyvern_datagen/              ← Monte Carlo atmospheric dataset generator + GUI
│       └── README.md
├── 3D parts/                        ← 70 mm 3-bay airframe + gimbal STL/STEP
├── Motor Test Stand/                ← static thrust + jetvane stand + 3-axis TVC balance
├── Senior Research/                 ← proposal documents (DOCX / MD / PDF)
├── Data/                            ← flight and motor data (populated during testing)
└── Paper/                           ← final research paper
```


## Fin finding (2026-06-21)
35 mm fins are **unstable** (−0.99 cal) on this aft-CG vehicle; 1.0 cal needs ≥68.8 mm and 1.5 cal would need ~91.8 mm fins (or nose ballast, which costs apogee). **Finned TVC at 72 mm / +1.10 cal is the flown config** — see `Documentation/WYVERN_E4_Stability_FinSizing.md`. Motor prices corrected: F15-4 $17/ea, E16-4 $15/ea.


## Latest spec deltas (2026-07)
Light 2S LiPo → one 5 V UBEC (Zeee 2S 450 mAh + Hobbywing UBEC; ~76 g power+cam group, keeps the 705 g budget) · EMAX ES08MA II servos @ 5 V · i3 4K Thumb Action Camera cam (~36 g) · Picos from Amazon · No ArduCam · phenolic motor liner + Nomex bore sleeve · motor-ejection recovery (no pyro of our own) · printed 1010 rail buttons · BOM reconciled to actual Amazon/Adafruit/Estes/Bambu carts · trajectory via unified RK4+Barrowman (`we4_flightsim.py`).
