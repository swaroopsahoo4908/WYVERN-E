# WYVERN-E, Research Proposal

### A Skylight Rocketry Venture · 70 mm Single-Stage Servo-TVC Sustainer
##### Allison Hong · Chris Liu · Swaroop K. Sahoo

## Executive Summary
WYVERN-E is a 70 mm, single-stage, **3D-printed active-thrust-vector-control (TVC)** research
rocket that demonstrates closed-loop flight stabilization on a bare-metal **Raspberry Pi Pico 2 W (RP2350)** flight
computer, powered by the **Estes F15-4**. It carries four 72 mm fins (no ballast) for
**passive stability during launch and the F15 ignition spike**, after which (t ≥ 0.5 s) the TVC loop
engages on the smooth portion of the thrust curve to stabilize and execute a commanded maneuver. The
two candidate TVC actuators, a tri-solenoid *magnetic* gimbal and a *servo* gimbal, are compared
quantitatively on a purpose-built **3-axis thrust-vector balance** that resolves both thrust
magnitude and vector direction; the flight vehicle carries the servo system. The program retains the
static motor/materials test regimen; the aerofoil wind-tunnel campaign was removed in 2026-08 and its
aerodynamic question is now answered analytically and validated in flight (RQ3). All flights are FAA
Class-1 (no waiver, no certification). Predicted apogee ≈ 324 ft; project cost ≈ $1,720.

## 1. Background & Motivation
### 1.1 Thrust vector control
Thrust vectoring steers a rocket by gimbaling the exhaust, producing a control moment without
aerodynamic surfaces, the method used by every orbital launch vehicle. A small, low-cost, fully
printed TVC vehicle makes the control problem tangible while producing real flight data.

### 1.2 WYVERN lineage
WYVERN-E 1.0 (interceptor study) → 2.0 (84 mm two-stage, custom avionics) → 3.0 (Raspberry Pi 5,
solenoid-vs-servo A/B in flight) established the aero, structures, and avionics groundwork. 4.0
distills those lessons into the simplest vehicle that still answers the core question: a single
stage, a single bare-metal controller, and the A/B actuator comparison moved to a repeatable ground
balance.

### 1.3 The 4.0 thesis
*A small finned rocket can use passive stability to survive launch and the ignition transient, then
hand authority to a closed-loop TVC system on the smooth thrust curve, and the relative merit of
magnetic vs servo gimbals can be measured directly on the ground before flight.*

## 2. Research Questions
Numbering matches `WYVERN_E4_Proposal_rev2.md` §2 (canonical). The wind tunnel and the airfoil-CFD
package were removed from the program in 2026-08; the aerofoil-polar question they served is replaced
by RQ3 below, which is answered analytically and validated in flight.

| # | Question | Method A | Method B | Primary metric |
|---|---|---|---|---|
| RQ1 | Magnetic vs servo TVC actuation | 3-axis thrust-vector balance, F15-0 | bench signal model + SIL | bandwidth, slew, overshoot, SSE, max vector angle |
| RQ2 | PLA vs PETG-CF for TVC structures | 3-point bend coupons, both materials | lumped-capacitance thermal + measured engine-bay wall temp from static fires | flexural modulus, HDT margin |
| RQ3 | Predicted vs in-situ passive stability | Barrowman CP/CN + RK4 dispersion | flight-telemetry reconstruction of margin and coast Cd | Δ static margin (cal), Δ Cd |
| RQ4 | Closed-loop gain sensitivity in flight | 24-point phase/gain-margin sweep + Monte Carlo | onboard log, up to 4 flights | pitch error, gimbal track, PM/GM, recovery |

## 3. Vehicle Architecture
### 3.1 Configuration & mass budget
70 mm OD, 0.74 m, three bays + two bulkheads; 4 fins (72 mm), no ballast.

| Section | Key mass | Subtotal |
|---|---|---|
| Nose (PLA) | ellipsoid nose 21 g | 21 g |
| Recovery bay (PLA) | bay tube, chute+Kevlar cord, Nomex, BNO085 (vote), **PETG-CF** bypass tube, **PETG-CF** sealed Bulkhead B, ejection plenum | 137 g |
| FC bay (PLA) | bay tube, Pico 2 W, BNO085, baro, µSD, i3 4K Thumb Action Camera, 2S LiPo + 5 V UBEC | 122 g |
| Engine/TVC bay (PETG-CF) | bay tube, Bulkhead A, gimbal, 2 servos, BNO085, mount | 268 g |
| Structure | **4 PLA fins (72 mm)** + wiring | 50 g |
| **Dry** | | **690 g** |
| Motor | F15-4 (60 g prop) | 102 g |
| **Liftoff** | | **792 g** |

### 3.2 Materials
- **PETG-CF** (ρ ≈ 1.30 g/cm³, HDT ≈ 80 °C): both bulkheads, the ejection bypass tube, and the engine assembly (engine/TVC bay + motor mount + gimbal), i.e. the whole ejection-gas path plus the TVC structures.
- **PLA** (ρ ≈ 1.24 g/cm³, HDT ≈ 55 °C): nose, both body tubes, fins, everything with no motor heat and no gas contact. Walls run 1.2 mm (3 perimeters) rather than 1.6 mm; the airframe is print/handling-limited at ~340× safety factor, so the thicker wall was carrying margin it never needed, and dropping it recovers 45.6 g.
- **Zoning rationale is thermal, not mass.** PLA at 55 °C HDT cannot be trusted in contact with ejection gas; PETG-CF at 80 °C is the minimum defensible material for that path.

### 3.3 Stability, fins + ballast + the 0.5 s rule
An apogee sweep shows ballast lowers altitude, so we use **no ballast** and size fins to the minimum stable 1.0 cal: 4 × 72 mm fins → CP 56.8 cm, CG 48.4 cm = **+1.20 cal**
static margin (stable). This passive margin holds the vehicle through launch and the F15 ignition
spike; the TVC controller is **inhibited until t = 0.5 s**, then engages on the smooth curve. A
historical finless variant (margin −5.6 cal) was rejected because it is statically unstable and cannot survive
the pre-TVC transient.

### 3.4 Structural & thermal margins
First-order analysis (`we4_analysis.py`): minimum safety factor **> 300×** (the 25 N motor leaves the
1.6 mm airframe print/handling-limited, not load-limited); fin flutter velocity well above the 25 m/s
flight regime; engine-bay thermal margin to PETG-CF HDT ≈ 40 °C (wall peaks ~40 °C against an 80 °C HDT).

## 4. Propulsion & Trajectory
### 4.1 Motors (verified)
| Motor | Spec | Role | Qty |
|---|---|---|---|
| Estes F15-4 | 49.6 N·s, 14.4 N avg / 25.3 N pk, 3.45 s, 4 s delay + ejection | flight only (ejection = recovery) | 4 |
| Estes F15-0 | 49.6 N·s, 14.4 N avg / 25.3 N pk, 3.45 s, 0-delay (plugged) | static stand + TVC balance (MTVC and servo) | 10 |
| Estes E16-4 | ~16 N avg | stand commissioning | 6 |

### 4.2 Predicted performance (unified RK4 + Barrowman, `we4_flightsim.py`)
T/W 1.85 avg / 3.26 peak; Cd 0.539; burnout 3.45 s, 59.1 m, 28.9 m/s; **apogee ~324 ft @ 6.27 s**; motor
ejection at t = 7.45 s (+1.18 s past apogee) @ 11.5 m/s; 24″ chute → 5.0 m/s descent. Dispersion (±5 % mass, ±15 % Cd): see
`plots4/06_dispersion.png`.

## 5. Flight Computer & Control
### 5.1 Raspberry Pi Pico 2 W (RP2350)
A single dual-core 150 MHz RP2350 is flight computer *and* 500 Hz controller: core 0 reads three
BNO085 + two baros (BME688 + BMP388) and closes the TVC loop driving 2 servos via hardware PWM;
core 1 handles microSD logging and Wi-Fi bench telemetry. No Linux, no scheduler jitter. Recovery is
the motor's own ejection charge (the FC only logs/observes).

### 5.2 IMUs, Game Rotation Vector
All three BNO085 run in **Game Rotation Vector** (accel + gyro, magnetometer disabled) to reject the
magnetic interference of the adjacent servos. Gimbal deflection = $q_{body}^{-1}\otimes q_{gimbal}$,
giving true nozzle attitude relative to the body (catches linkage backlash/flex). The recovery-bay
unit votes against the FC unit for fault detection.

### 5.3 Control law
Per-axis PID ($K_p{=}0.10,\ K_i{=}0.40,\ K_d{=}0.18$; margin- and auto-tune-validated), output clamped to ±8°, servo lag τ ≈ 0.04 s. **TVC
inhibited for the first 0.5 s** (ignition spike), fins hold attitude, then engages: stabilize to
vertical, then a 4° commanded maneuver. Required gimbal torque 0.56 kg·cm (micro-servo class);
control authority positive throughout the powered phase.

### 5.4 Power & data
A light 2S LiPo (7.4 V, ~450 mAh) feeds a single 5 V/6 V UBEC (set 5 V) whose one rail powers Pico 2 W VSYS, the camera, and both servos (EMAX ES08MA II, running at 5 V ≈ 1.8 kg·cm). Shared-rail decoupling (1000 µF bulk at the servos, 100 µF + an SS34 hold-up Schottky at VSYS) keeps servo-stall transients from browning-out the Pico. No recovery battery, recovery is the motor's own ejection charge.
Pack voltage is monitored on GP26/ADC0 (100k/62k divider; warn 6.4 V, arm-inhibit 6.0 V). Onboard log: full-rate IMU/baro/control + i3 4K Thumb Action Camera 1080p60 video. The power+camera group (LiPo ~30 g + UBEC ~10 g + i3 4K Thumb Action Camera ~36 g ≈ 76 g) sits inside the 122 g FC-bay budget.

## 6. Recovery
Recovery uses the **F15-4 motor's own ejection charge** (4 s delay → fires t ≈ 7.45 s, ~0.66 s past
apogee), routed through a solid-walled 12 mm PETG-CF bypass tube past the *sealed* FC bay into the
recovery bay to release a friction-fit nose, **no RRC3+, no 9 V, no e-match/BP, no CO2, no FC
involvement**. Feasibility (`we4_ejection_feasibility.py`): tube loss ≈ 0.06 kPa; bay pressurizes to
~140 kPa vs a 14–41 kPa nose-release threshold = **3.4× margin**. F15-4 is the closest Estes delay
to the ~3.5 s coast optimum (F15-6/-8 eject 2.5 s/4.5 s late, too fast/low). Single passive event,
no electronic backup. Opening at ~11.5 m/s; 1/8″ Kevlar cord (> 800× margin) + Nomex protector; 24″
chute → ~6 m/s.

## 7. Ground Test Program
### 7.1 3-axis thrust-vector balance
Motor + gimbal on a thrust block restrained by one axial (5 kg) + two lateral (1 kg) strain-gauge
load cells → $T,\ \theta,\ \phi$. Actuator-agnostic, both magnetic and servo systems tested
identically. RQ3 metrics logged vs commanded.
### 7.2 Static thrust + materials stand
Axial cell + steel deflector: validates the F15-0 thrust curve and carries the engine-bay wall thermocouple for RQ2 (plugged 0-delay on the ground, no ejection into the fixture). Jetvane screening is out of scope as of 2026-08.
### 7.3 Motor & calibration plan
Load cells dead-weight calibrated, then commissioned with **6 × E16-4** (3 per stand). Counts: F15-4 ×4 (flight), F15-0 ×13 (ground), E16-4 ×6.

## 8. Simulation Suite (RQ3/RQ4 Method A)
RK4 + Barrowman trajectory with componentwise drag buildup, power-law wind shear, and Monte Carlo
dispersion over the atmospheric and build-tolerance envelope; 500 Hz closed-loop pitch model with
servo lag, transport delay, and Dryden-spectrum gust forcing; phase/gain-margin sweep across the
burn-time × atmosphere grid; and a software-in-the-loop flight computer writing logs in the onboard
recorder's schema. See `Simulations/README.md`.

## 9. Safety & Regulatory
Single F15-4: 49.6 N·s, 60 g propellant, ≤ F class, liftoff 792 g < 1500 g → **FAA Class-1, no waiver,
no Level-1 certification**. Remote ignition, ≥ 3 m standoff on the stands, gimbal-neutral fail-safe,
motor-integral ejection (igniter installed at the pad; no electronic ejection circuit to arm or inhibit).

## 10. Budget
≈ **$1,720** total program spend (vehicle + 3-axis balance + static/materials stand + one-time tools
+ all motors): $1,241 still to buy + $479 already acquired. This is down from the $1,882 originally
scoped; the Hofferth wind tunnel section was deleted from the BOM in the 2026-08 scope change. Live
per-line pricing in
`Documentation/WYVERN_E4_BOM.xlsx`. Per-flight consumable ≈ F15-4 $17 (integral delay/ejection; no
separate initiator or BP charge).

## 11. Schedule & Milestones
| Phase | Wk | Milestone |
|---|---|---|
| Print + assemble | 1–3 | airframe, gimbal, both stands |
| Bench bring-up | 4 | self-test all-PASS, control loop dry-run |
| TVC balance A/B (RQ1) | 5–6 | magnetic vs servo dataset |
| Static fires (RQ2) | 7 | thrust-curve verification + engine-bay wall temperature |
| Sim + margin analysis (RQ3/RQ4) | 8–9 | dispersion, PM/GM sweep, SIL |
| Flight tests (RQ3/RQ4) | 10–11 | up to 4 flights, onboard logs |
| Analysis + paper | 12–14 | results, paper |

## 12. Risk Register
| Risk | Likelihood | Mitigation |
|---|---|---|
| Pre-TVC instability | Med | 4× 72 mm fins, no ballast (+1.20 cal at liftoff); TVC inhibit 0.5 s |
| Servo slew too slow | Med | bench-verify on balance before flight; fast digital micro |
| Hard deploy at ~11.5 m/s (motor ejection, +1.18 s past apogee) | Low | Kevlar harness >800× margin, Nomex protector, ground-tested charge |
| Launch-detect miss at low T/W | Low | tune arming alt; verify on 2.2 g spike |
| Camera/SD throughput | Low | self-contained i3 4K Thumb Action Camera (decoupled from FC) |

## 13. Expected Outcomes & Deliverables
A flight-validated small TVC vehicle; a quantitative magnetic-vs-servo TVC dataset; a quantified
accuracy bound on Barrowman-class stability prediction versus flight telemetry; a PLA-vs-PETG-CF
structural comparison justifying the flown material zoning; full open documentation (CAD, firmware, wiring, sims) and a research paper.

## References
Barrowman, J. *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned
Vehicles*, NASA, 1967. Box, J. (BPS.space) *Signal/Echo* TVC flight series. NAR/Tripoli Model Rocket
Safety Codes. Bosch Sensortec BNO085 datasheet; Raspberry Pi RP2350 / Pico 2 W datasheet.
