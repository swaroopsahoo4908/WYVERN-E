# GTR70E WYVERN

**Authors:** Swaroop Sahoo, Chris Liu, Allison Hong  
**Date:** 2026-08-12  
**Program:** GTR70E WYVERN


A single-stage, 70 mm, finned active-TVC sustainer built around **PCB1**, a custom-designed
Ø62 mm flight computer board carrying a bare **RP2350B** (QFN-80, no onboard radio), powered by an
**Estes F15-4**.

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

Single stage, single custom PCB1 board as flight computer and real-time controller: dual-core
RP2350B, one core dedicated to the 500 Hz TVC loop, no Linux, native hardware PWM, no onboard
WiFi/BLE radio (telemetry is logged to microSD, not streamed). The magnetic-vs-servo comparison
runs on the ground (two matched three-axis load balances) rather than in flight, since that gives a
repeatable, directly-measured thrust vector instead of a single noisy flight-to-flight data point.

**Materials.** ASA-Aero forms the upper body that houses avionics (nose, recovery bay tube, FC bay
tube) and, as of the 2026-08-12 mass pass, the lower body tube (chute/TVC bay); PETG-CF forms the
fins and the bulkhead joint; PC-FR forms the TVC assembly proper (motor mount, gimbal). Zoning
follows thermal exposure and structural role, not a single blanket material.

## Key numbers (see `Simulations/we4_sim.py` → `plots4/`)

| | value |
|---|---|
| Liftoff / dry mass | **698 g / 638 g** (finned 87 mm, no ballast) |
| T/W | 2.10 avg / 3.70 peak |
| CG / gimbal pivot / control arm | 50.1 cm / 62 cm / 11.9 cm from nose |
| Pitch inertia Iyy | 0.0262 kg·m² |
| Burnout | 3.45 s · 74.0 m · 36.3 m/s |
| Apogee | ~439 ft / 133.7 m, +1.31 cal margin, @ 6.87 s (RK4+Barrowman) |
| Recovery | F15-4 motor ejection; deploys t≈7.45 s (+0.58 s past apogee) @ ~5.7 m/s; 24″ chute → 4.7 m/s descent |
| TVC | gimbal stays within ±8°; control authority positive throughout the burn |
| FAA class | Class 1, no waiver (698 g < 1500 g, F-class motor) |
| PCB1 | Ø62 mm, 2 copper layers, 65 components, custom RP2350B (QFN-80) |

## Repository structure

```
GTR70E WYVERN/
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
├── PCB/ ← PCB1 EasyEDA export (schematic, fab files, BOM, Gerbers)
├── Flight Computer/ ← custom PCB1 spec, firmware, wiring, GSE test rigs
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
`Documentation/WYVERN_E4_GSE_TestStands.md` for the full build-out and instrumentation. The ground
rigs' own DAQ runs on off-the-shelf Raspberry Pi Pico/Pico 2 W boards — a separate, bench-only
controller from the flight computer (PCB1).

Design and decision history, including why specific numbers or scope changed, lives in
`Documentation/CONFLICTS.md` rather than here.

## References

Raspberry Pi Ltd. (2024). *RP2350 datasheet*. https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf

Estes Industries. (n.d.). *F15-4 engines* [Product specification]. Retrieved August 12, 2026, from https://estesrockets.com/products/f15-4-engines
