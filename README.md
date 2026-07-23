# WYVERN-E 4.0

A single-stage, 70 mm, finless **active-TVC sustainer** demonstrating closed-loop thrust-vector
control on a **Raspberry Pi Pico 2 W (RP2350)** flight computer, powered by the **Estes F15-4**. The two TVC actuation
methods (magnetic-solenoid vs servo) are now compared **on the ground** on a 3-axis thrust-vector
balance; the vehicle flies the servo system. Ground-support carries over from 3.0: the Hofferth
wind tunnel (RQ1/RQ2 aerofoil testing) and the static thrust + materials (jetvane) stand.

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
| CG / gimbal pivot / control arm | 45.0 cm / 62 cm / **17.0 cm** from nose |
| Pitch inertia Iyy | 0.0219 kg·m² |
| Burnout | 3.45 s · 83.6 m · 39.6 m/s |
| Apogee | **~435 ft / 133 m** (RK4+Barrowman, stable +1.10 cal) @ 6.81 s |
| Recovery | F15-4 motor ejection via bypass tube; ejects t≈7.45 s (+0.66 s past apogee) @ ~6.5 m/s; 18″ chute → 6 m/s descent |
| TVC | gimbal stays within ±8°; control authority positive throughout the burn |

> **Apogee/deploy note:** the ASA-Aero airframe (PC-FR only at bulkheads/tube/engine) drops liftoff to
> 705 g and lifts apogee to ~435 ft. Recovery is the F15-4 motor ejection charge (fixed 4 s delay),
> firing +0.64 s past apogee at a gentle ~6.3 m/s — no timer to retune. The lighter ASA nose moved the
> CG aft, so fins were grown 58→72 mm to hold the 1.0-cal margin without ballast.

## Repository structure

```
WYVERN-E 4.0/
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
│   ├── WYVERN_E4_BOM.xlsx           ← 60-line master BOM + purchase links
│   ├── FLIGHT_READINESS.md
│   ├── COMPATIBILITY.md
│   └── CONFLICTS.md
├── Flight Computer/                 ← Pico 2 W spec, firmware, wiring, GSE test rigs
│   └── README.md
├── Simulations/                     ← Python RK4 suite, OpenRocket, dataset generator
│   ├── README.md
│   └── wyvern_datagen/              ← Monte Carlo atmospheric dataset generator + GUI
│       └── README.md
├── 3D parts/                        ← 70 mm 3-bay airframe + gimbal STL/STEP
├── Wind Tunnel/                     ← Hofferth tunnel (carried over from 3.0)
├── Motor Test Stand/                ← static thrust + jetvane stand + 3-axis TVC balance
├── Senior Research/                 ← proposal documents (DOCX / MD / PDF)
├── Data/                            ← flight, tunnel, and motor data (populated during testing)
└── Paper/                           ← final research paper
```


## Fin finding (2026-06-21)
35 mm fins are **unstable** (−0.52 cal) on this aft-CG vehicle; 1.5 cal needs ~75 mm fins or nose ballast. **Finned TVC (1.0 cal) is the flown config** — see `Documentation/WYVERN_E4_Stability_FinSizing.md`. Motor prices corrected: F15-4 $17/ea, E16-4 $15/ea.


## Latest spec deltas (2026-07)
Light 2S LiPo → one 5 V UBEC (Zeee 2S 450 mAh + Hobbywing UBEC; ~76 g power+cam group, keeps the 705 g budget) · EMAX ES08MA II servos @ 5 V · i3 4K Thumb Action Camera cam (~36 g) · Picos from Amazon · No ArduCam · phenolic motor liner + Nomex bore sleeve · motor-ejection recovery (no pyro of our own) · printed 1010 rail buttons · BOM reconciled to actual Amazon/Adafruit/Estes/Bambu carts · trajectory via unified RK4+Barrowman (`we4_flightsim.py`).
