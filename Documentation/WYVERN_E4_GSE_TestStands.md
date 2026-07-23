# WYVERN-E 4.0 — Ground Test Stands & Motor Plan

## 1. TVC thrust-vector balance (3-axis; magnetic & servo)
The actuator under test (magnetic-solenoid gimbal *or* servo gimbal) bolts to a thrust block
restrained from a fixed base by **three strain-gauge load cells through flexures**: one **axial (Z)**
and two **lateral (X, Y)**. This resolves the full thrust vector — magnitude *and* direction:

$$T=\sqrt{F_x^2+F_y^2+F_z^2},\quad \theta=\arctan\frac{\sqrt{F_x^2+F_y^2}}{F_z},\quad
\phi=\operatorname{atan2}(F_y,F_x)$$

Sized for the small motors here: **F15 peak 25.3 N**, side force at ±8° ≈ **3.5 N** → a **5 kg
axial + two 1 kg lateral** cells + 3× HX711 → Raspberry Pi Pico DAQ at 80 SPS. (Elegant alternative: a
cruciform flexure with a Wheatstone bridge per arm = a one-piece 3-axis F/T sensor.) The rig is
actuator-agnostic, so the magnetic-vs-servo A/B comparison runs here, repeatably, *before* flight —
logging commanded vs measured (θ, φ) gives bandwidth, slew, overshoot, steady-state error per system.

## 2. Static thrust + materials (jetvane) stand — carried over, + deflector
Axial-only (1× load cell + HX711), with a **steel blast deflector**. Validates the F15-0 thrust
curve *and* holds candidate **jetvane material coupons (ASA-Aero, ABS, PC-FR, PLA)** in the F15
plume for flame-resistance/erosion ranking. The deflector + mounts are sized for the F15-0's 3.45 s
burn thermal case. (A flight-usable jetvane must be graphite/tungsten — every printed polymer,
ASA-Aero and ABS included, ablates in the exhaust; the coupons quantify how fast. See the datagen
"Jetvane Suitability" tab.)

## 3. Motor plan & counts (verified specs)
| Motor | Spec (verified) | Use |
|---|---|---|
| **Estes F15-4** | 49.6 N·s, 14.4 N avg / 25.3 N pk, 3.45 s, 4 s delay + ejection | **flight only** (ejection charge = recovery) |
| **Estes F15-0** | 49.6 N·s, 14.4 N avg / 25.3 N pk, 3.45 s, 0-delay (plugged) | **ground stands + jetvane** (no ejection into fixtures) |
| **Estes/AeroTech E16-4** | ~16 N avg, E-class | stand commissioning firings |

**Counts:**
- *F15-4* = **4 (flight only)** — the ejection charge is the recovery system.
- *F15-0 (plugged)* = **13** for ground = 6 TVC-stand (3/system × 2) + 5 static (curve + jetvane) + 2 spare.
  Ground fixtures use the 0-delay F15-0 so no ejection charge fires into the stand; the thrust curve is
  identical to the F15-4 (same F15 propellant), so ground data transfers directly to the flight motor.
- *E16-4 (calibration)* = **6 recommended** (3 per stand for repeatability); **4 is the floor**
  (2/stand). Note: load cells are *calibrated* with known hanging masses (free, precise); the E16
  firings *commission/validate* the calibrated stand against a published curve.

## 4. Both stands → `Data/Motor (thrust curves)/` and `Data/TVC (vector + control)/`.
