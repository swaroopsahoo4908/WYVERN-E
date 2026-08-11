# WYVERN-E, Ground Test Stands & Motor Plan

**2026-08-10: full project spec restored.** Four independent ground-test rigs, one per research
question that needs physical validation before flight: static fire (RQ2 materials + calibration),
servo TVC stand (RQ1/RQ5), magnetic TVC stand (RQ1/RQ5), wind tunnel (RQ3/RQ4). Jetvane testing,
dropped in the 2026-08 scope cut, is back on the static-fire stand under a new protocol (below).

## 1. Static fire test stand, calibration, thrust curves, jetvane materials screen
Axial-only load path (1x 5 kg cell + HX711) with a **steel blast deflector**, sized for the F15-0's
3.45 s burn thermal case. Three jobs run on this one stand:

- **RQ1 baseline**: total impulse, average and peak thrust, burn time and curve shape for the
  F15-0, the reference curve both TVC stands are compared against.
- **Stand/DAQ calibration**: known hanging masses calibrate the load cell before every firing
  campaign; E16-4 commissioning firings validate the calibrated chain against a published curve.
- **RQ2 jetvane blast-shield screen (redefined 2026-08-10)**: this is a materials screen, not a
  vane-deflection test. A flat coupon plate of each candidate material, 5 mm thick and printed at
  100% infill, is mounted directly in the exhaust path like a blast shield and fired on. The motor
  runs straight into the plate; the response is melt-through, surface ablation depth, and slag
  buildup, not thrust or deflection. Six materials go through the screen: PLA, PETG-CF, ABS,
  ASA-Aero, PC, and PC-FR. Any material that survives 5 mm without melting through gets retested
  at 4 mm, then 3 mm, then 2 mm, until it fails, giving a failure-thickness ranking across the six
  candidates rather than a single pass/fail. The BME688 already on the stand logs plume-adjacent
  temperature alongside each firing so the melt/slag result has a temperature record to go with it.

## 2. Servo TVC test stand
The servo-gimbal actuator bolts to a thrust block restrained from a fixed base by **three
strain-gauge load cells through flexures**: one **axial (Z)** and two **lateral (X, Y)**. Resolves
the full thrust vector, magnitude *and* direction:

$$T=\sqrt{F_x^2+F_y^2+F_z^2},\quad \theta=\arctan\frac{\sqrt{F_x^2+F_y^2}}{F_z},\quad
\phi=\operatorname{atan2}(F_y,F_x)$$

Sized for the small motors here: **F15 peak 25.3 N**, side force at ±8° ≈ **3.5 N** → a **5 kg
axial + two 1 kg lateral** cells + 3× HX711 → one of the three custom RP2350B flight-computer PCBs
(2026-08-10: repurposed from a bare Pico DAQ) at 80 SPS. Logging commanded vs
measured (θ, φ) gives bandwidth, slew, overshoot, and steady-state error for the servo system,
this is the rig the flight vehicle's actual TVC gets qualified on (`wyvern4_gse_servo_rig`).

## 3. Magnetic TVC test stand
A second, physically separate rig (`wyvern4_gse_solenoid_rig`) using the same three-load-cell
flexure and DAQ chain as the servo stand, but with the magnetic-solenoid gimbal actuator in place
of the servo gimbal. Running the identical instrumentation chain on both stands is what makes the
**RQ1 magnetic-vs-servo A/B comparison** valid, commanded vs measured (θ, φ) from each stand feed
directly into the same bandwidth/slew/overshoot/steady-state-error metrics for a like-for-like
comparison. Only the servo system flies (RQ1 answers which actuation type wins on the bench; the
flight vehicle carries the winner's specs into the record either way).

## 4. Wind tunnel, aerofoil performance testing (RQ3/RQ4)
Bench aerofoil rig (`Wind Tunnel/`) for direct measurement of fin aerofoil performance, 
lift/drag/stall behavior across the flown fin's angle-of-attack range, to calibrate against the
Barrowman-derived stability numbers in `WYVERN_E4_Stability_FinSizing.md`. This is the RQ4
wind-tunnel-vs-flight comparison: tunnel-measured coefficients here, flight-derived weathercocking
and stability margin from telemetry (`Simulations/plots_val/05_wind_weathercock.png`) after launch.
Feeds RQ3 (fin aerofoil selection) directly, whichever aerofoil the tunnel run favors is the one
committed to the flown fin can.

## 5. Motor plan & counts (verified specs, jetvane blast-shield screen included)
| Motor | Spec (verified) | Use |
|---|---|---|
| **Estes F15-4** | 49.6 N.s, 14.4 N avg / 25.3 N pk, 3.45 s, 4 s delay + ejection | **flight only** (ejection charge = recovery) |
| **Estes F15-0** | 49.6 N.s, 14.4 N avg / 25.3 N pk, 3.45 s, 0-delay (plugged) | **ground stands** (no ejection into fixtures) |
| **Estes/AeroTech E16-4** | ~16 N avg, E-class | stand commissioning firings |

**Counts:**
- *F15-4* = **4 (flight only)**: the ejection charge is the recovery system.
- *F15-0 (plugged)* = **13-24** for ground, split three ways: 6 TVC-stand firings (3 servo + 3
  magnetic), 2 static thrust-curve firings, and the jetvane blast-shield screen, which is adaptive
  rather than a fixed count. Six materials at up to 4 thickness steps each (5/4/3/2 mm, stopping at
  first melt-through) tops out at 24 firings if every material survives to 2 mm; in practice most
  candidates are expected to fail by 4 mm, so budget for roughly 12-16 firings and treat 24 as the
  worst-case ceiling. Plan for **13 firings as the low estimate plus a spares allowance for repeats**.
  Ground fixtures use the 0-delay F15-0 so no ejection charge fires into the stand; the thrust curve
  is identical to the F15-4 (same F15 propellant), so ground data transfers directly to the flight motor.
- *E16-4 (calibration)* = **6 recommended** (3 per TVC stand for repeatability); **4 is the floor**
  (2/stand). Load cells are *calibrated* with known hanging masses, free and precise; the E16 firings
  *commission and validate* the calibrated stand against a published curve.

## 6. Data paths
Static fire + both TVC stands → `Data/Motor (thrust curves)/` and `Data/TVC (vector + control)/`.
Wind tunnel → `Data/Wind Tunnel/`.
