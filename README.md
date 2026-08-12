# GTR70 WYVERN-E

A single-stage, 70 mm, finned active-TVC sustainer built around a **Raspberry Pi Pico 2 W (RP2350)**
flight computer, powered by an **Estes F15-4**. 

The vehicle answers five research questions, run across computational simulation, ground-based
instrumented testing, and powered flight:

- **RQ1, actuator class.** Magnetic-solenoid vs. servo thrust-vector control, compared on two
  physically separate, instrumentally identical three-axis load balances. The vehicle flies servo.
- **RQ2, zoned materials.** ASA-Aero, PETG-CF, and PC-FR allocated by section (upper body, lower
  body/fins, TVC assembly), justified by three-point-bend and thermal data, plus a jetvane
  blast-shield materials screen on the static-fire stand.
- **RQ3, fin aerofoil selection.** Bench wind-tunnel measurement of candidate fin sections,
  cross-checked against a 2D vortex-panel CFD solver.
- **RQ4, wind-tunnel-vs-flight calibration.** Barrowman-predicted stability checked against both
  tunnel measurement and reconstructed flight telemetry.
- **RQ5, control-gain sensitivity.** Closed-loop PID gain sets compared across repeated flights on
  the single-controller avionics architecture.

## Vehicle summary

Single stage, single Pico 2 W as flight computer and real-time controller: dual-core, one core
dedicated to the 500 Hz TVC loop, no Linux, native hardware PWM. The magnetic-vs-servo comparison
runs on the ground (two matched three-axis load balances) rather than in flight, since that gives a
repeatable, directly-measured thrust vector instead of a single noisy flight-to-flight data point.

**Materials.** ASA-Aero forms the upper body that houses avionics (nose, recovery bay tube, FC bay
tube); PETG-CF forms the lower body and fins; PC-FR forms the TVC assembly (motor mount, gimbal).
Zoning follows thermal exposure and structural role, not a single blanket material.

## Key numbers (see `Simulations/we4_sim.py` → `plots4/`)

| | value |
|---|---|
| Liftoff / dry mass | **729 g / 627 g** (finned 87 mm, no ballast) |
| T/W | 2.01 avg / 3.54 peak |
| CG / gimbal pivot / control arm | 50.8 cm / 62 cm / 11.2 cm from nose |
| Pitch inertia Iyy | 0.0257 kg·m² |
| Burnout | 3.45 s · 68.7 m · 33.7 m/s |
| Apogee | ~397 ft / 121.1 m, +1.20 cal margin, @ 6.67 s (RK4+Barrowman) |
| Recovery | F15-4 motor ejection; deploys t≈7.45 s (+0.78 s past apogee) @ ~7.7 m/s; 24″ chute → 4.8 m/s descent |
| TVC | gimbal stays within ±8°; control authority positive throughout the burn |
| FAA class | Class 1, no waiver (729 g < 1500 g, F-class motor) |

## Repository structure

```
WYVERN Project/
├── README.md ← this file
├── .gitignore
├── Documentation/ ← all engineering docs, BOM, and build readiness
│ ├── README.md ← documentation index
│ ├── WYVERN_E4_BUILD_READINESS.md ← GO/NO-GO reconciliation report
│ ├── WYVERN_E4_Mathematics.md ← mass/CG/inertia, T/W, trajectory, TVC, recovery
│ ├── WYVERN_E4_Stability_FinSizing.md
│ ├── WYVERN_E4_FEA_Structural.md
│ ├── WYVERN_E4_Recovery.md
│ ├── WYVERN_E4_Camera_Solution.md
│ ├── WYVERN_E4_GSE_TestStands.md
│ ├── WYVERN_E4_PID_AUTOTUNE_REPORT.md
│ ├── WYVERN_E4_BOM.xlsx ← master BOM + purchase links
│ ├── WYVERN_E4_Timeline_3Month.md ← build-to-flight schedule
│ ├── WYVERN_E4_Build_Guide.md ← print/bench/assembly/ground-test/range procedures
│ ├── FLIGHT_READINESS.md
│ ├── COMPATIBILITY.md
│ └── CONFLICTS.md ← defect log and design-decision record
├── Flight Computer/ ← Pico 2 W spec, firmware, wiring, GSE test rigs
│ └── README.md
├── Simulations/ ← Python RK4 suite, OpenRocket, dataset generator
│ ├── we4_flight_reduce.py ← post-flight SD log → RQ3/RQ4 results (--selftest first)
│ ├── README.md
│ └── wyvern_datagen/ ← Monte Carlo atmospheric dataset generator + GUI
│ └── README.md
├── 3D parts/ ← airframe + gimbal STL/STEP
├── Motor Test Stand/ ← static thrust stand + 3-axis TVC balances
├── Wind Tunnel/ ← bench aerofoil rig
├── Senior Research/ ← proposal documents (DOCX / MD / PDF)
├── Data/ ← flight and motor data (populated during testing)
└── Paper/ ← final research paper
```

## Ground test program

Four purpose-built stands: a static-fire stand (thrust-curve calibration, engine-bay thermal
verification, and the RQ2 jetvane blast-shield screen), a servo TVC stand, a physically separate
magnetic TVC stand, and a bench wind tunnel for RQ3/RQ4 aerofoil work. See
`Documentation/WYVERN_E4_GSE_TestStands.md` for the full build-out and instrumentation.

Design and decision history, including why specific numbers or scope changed, lives in
`Documentation/CONFLICTS.md` rather than here.
