# WYVERN-E 4.0 — Research Proposal

### A Skylight Rocketry Venture · 70 mm Single-Stage Servo-TVC Sustainer
##### Allison Hong · Chris Liu · Swaroop K. Sahoo

## Executive Summary
WYVERN-E 4.0 is a 70 mm, single-stage, **3D-printed active-thrust-vector-control (TVC)** research
rocket that demonstrates closed-loop flight stabilization on a bare-metal **Raspberry Pi Pico 2 W (RP2350)** flight
computer, powered by the **Estes F15-4**. It carries four 72 mm fins (no ballast) for
**passive stability during launch and the F15 ignition spike**, after which (t ≥ 0.5 s) the TVC loop
engages on the smooth portion of the thrust curve to stabilize and execute a commanded maneuver. The
two candidate TVC actuators — a tri-solenoid *magnetic* gimbal and a *servo* gimbal — are compared
quantitatively on a purpose-built **3-axis thrust-vector balance** that resolves both thrust
magnitude and vector direction; the flight vehicle carries the servo system. The program retains the
WYVERN aerofoil wind-tunnel (RQ1/RQ2) and static motor/materials test regimens. All flights are FAA
Class-1 (no waiver, no certification). Predicted apogee ≈ 435 ft; project cost ≈ $1,882.

## 1. Background & Motivation
### 1.1 Thrust vector control
Thrust vectoring steers a rocket by gimbaling the exhaust, producing a control moment without
aerodynamic surfaces — the method used by every orbital launch vehicle. A small, low-cost, fully
printed TVC vehicle makes the control problem tangible while producing real flight data.

### 1.2 WYVERN lineage
WYVERN-E 1.0 (interceptor study) → 2.0 (84 mm two-stage, custom avionics) → 3.0 (Raspberry Pi 5,
solenoid-vs-servo A/B in flight) established the aero, structures, and avionics groundwork. 4.0
distills those lessons into the simplest vehicle that still answers the core question: a single
stage, a single bare-metal controller, and the A/B actuator comparison moved to a repeatable ground
balance.

### 1.3 The 4.0 thesis
*A small finned rocket can use passive stability to survive launch and the ignition transient, then
hand authority to a closed-loop TVC system on the smooth thrust curve — and the relative merit of
magnetic vs servo gimbals can be measured directly on the ground before flight.*

## 2. Research Questions
| # | Question | Method | Primary metric |
|---|---|---|---|
| RQ1 | Aerofoil lift/drag & deflection behaviour | Hofferth wind tunnel, 0.5° sweeps | Cl, Cd, Cl/Cd vs α |
| RQ2 | Print-material flame/erosion as jetvane candidates | F15-0 plume on static stand + deflector | mass-loss, char depth |
| RQ3 | Magnetic vs servo TVC (ground) | 3-axis thrust-vector balance | bandwidth, slew, overshoot, SSE, max vector angle |
| RQ4 | Closed-loop flight stabilization + maneuver | onboard log, up to 4 flights | pitch error, gimbal track, recovery |

## 3. Vehicle Architecture
### 3.1 Configuration & mass budget
70 mm OD, 0.74 m, three bays + two bulkheads; 4 fins (72 mm), no ballast.

| Section | Key mass | Subtotal |
|---|---|---|
| Nose (ASA-Aero) | ellipsoid nose 21 g | 21 g |
| Recovery bay (ASA-Aero) | bay tube, chute+Kevlar cord, Nomex, BNO085 (vote), **PC-FR** bypass tube, **PC-FR** sealed Bulkhead B, ejection plenum | 137 g |
| FC bay (ASA-Aero) | bay tube, Pico 2 W, BNO085, baro, µSD, i3 4K Thumb Action Camera, 2S LiPo + 5 V UBEC | 122 g |
| Engine/TVC bay (PC-FR) | bay tube, Bulkhead A, gimbal, 2 servos, BNO085, mount | 268 g |
| Structure | **4 ASA-Aero fins (72 mm)** + wiring | 50 g |
| **Dry** | | **603 g** |
| Motor | F15-4 (60 g prop) | 102 g |
| **Liftoff** | | **705 g** |

### 3.2 Materials
- **PC-FR** (ρ 1.25 g/cm³, HDT ~110 °C, UL94-V0): both bulkheads, ejection bypass tube, and the engine assembly (engine/TVC bay + motor mount + gimbal) — heat/ejection-pressure zones only.
- **ABS** (ρ ~1.04 g/cm³, HDT ~98 °C, not flame-rated): **jetvane material coupon only** (exhaust erosion screen alongside ASA-Aero); not a structural airframe material.
- **ASA-Aero** (foamed, ρ ~0.65 g/cm³): nose, body tube, fins, FC/recovery bays — primary structure,
  saves ~130 g where there is no motor heat. Thermal check: engine-bay wall peaks **~38 °C** over the 3.45 s burn (no liner
  needed; first-order lumped model, `we4_analysis.py`).

### 3.3 Stability — fins + ballast + the 0.5 s rule
An apogee sweep shows ballast lowers altitude, so we use **no ballast** and size fins to the minimum stable 1.0 cal: 4 × 72 mm fins → CP 56.8 cm, CG 49.1 cm = **+1.10 cal**
static margin (stable). This passive margin holds the vehicle through launch and the F15 ignition
spike; the TVC controller is **inhibited until t = 0.5 s**, then engages on the smooth curve. A
finless variant (margin −5.6 cal) was rejected because it is statically unstable and cannot survive
the pre-TVC transient.

### 3.4 Structural & thermal margins
First-order analysis (`we4_analysis.py`): minimum safety factor **> 300×** (the 25 N motor leaves the
1.6 mm airframe print/handling-limited, not load-limited); fin flutter velocity well above the 25 m/s
flight regime; engine-bay thermal margin to PC-FR HDT > 70 °C.

## 4. Propulsion & Trajectory
### 4.1 Motors (verified)
| Motor | Spec | Role | Qty |
|---|---|---|---|
| Estes F15-4 | 49.6 N·s, 14.4 N avg / 25.3 N pk, 3.45 s, 4 s delay + ejection | flight only (ejection = recovery) | 4 |
| Estes F15-0 | 49.6 N·s, 14.4 N avg / 25.3 N pk, 3.45 s, 0-delay (plugged) | ground stands + jetvane | 13 |
| Estes E16-4 | ~16 N avg | stand commissioning | 6 |

### 4.2 Predicted performance (unified RK4 + Barrowman, `we4_flightsim.py`)
T/W 2.08 avg / 3.66 peak; Cd 0.54; burnout 3.45 s, ~75 m, ~36 m/s; **apogee ~435 ft @ 6.81 s**; deploy
forced t = 4.0 s @ ~29 m/s; 18″ chute → ~6 m/s descent. Dispersion (±5 % mass, ±15 % Cd): see
`plots4/06_dispersion.png`.

## 5. Flight Computer & Control
### 5.1 Raspberry Pi Pico 2 W (RP2350)
A single dual-core 150 MHz RP2350 is flight computer *and* 500 Hz controller: core 0 reads three
BNO085 + two baros (BME688 + BMP388) and closes the TVC loop driving 2 servos via hardware PWM;
core 1 handles microSD logging and Wi-Fi bench telemetry. No Linux, no scheduler jitter. Recovery is
the motor's own ejection charge (the FC only logs/observes).

### 5.2 IMUs — Game Rotation Vector
All three BNO085 run in **Game Rotation Vector** (accel + gyro, magnetometer disabled) to reject the
magnetic interference of the adjacent servos. Gimbal deflection = $q_{body}^{-1}\otimes q_{gimbal}$,
giving true nozzle attitude relative to the body (catches linkage backlash/flex). The recovery-bay
unit votes against the FC unit for fault detection.

### 5.3 Control law
Per-axis PID ($K_p{=}0.10,\ K_i{=}0.40,\ K_d{=}0.18$; margin- and auto-tune-validated), output clamped to ±8°, servo lag τ ≈ 0.04 s. **TVC
inhibited for the first 0.5 s** (ignition spike) — fins hold attitude — then engages: stabilize to
vertical, then a 4° commanded maneuver. Required gimbal torque 0.56 kg·cm (micro-servo class);
control authority positive throughout the powered phase.

### 5.4 Power & data
A light 2S LiPo (7.4 V, ~450 mAh) feeds a single 5 V/6 V UBEC (set 5 V) whose one rail powers Pico 2 W VSYS, the camera, and both servos (EMAX ES08MA II, running at 5 V ≈ 1.8 kg·cm). Shared-rail decoupling (1000 µF bulk at the servos, 100 µF + an SS34 hold-up Schottky at VSYS) keeps servo-stall transients from browning-out the Pico. No recovery battery — recovery is the motor's own ejection charge.
Pack voltage is monitored on GP26/ADC0 (100k/62k divider; warn 6.4 V, arm-inhibit 6.0 V). Onboard log: full-rate IMU/baro/control + i3 4K Thumb Action Camera 1080p60 video. The power+camera group (LiPo ~30 g + UBEC ~10 g + i3 4K Thumb Action Camera ~36 g ≈ 76 g) sits inside the 122 g FC-bay budget.

## 6. Recovery
Recovery uses the **F15-4 motor's own ejection charge** (4 s delay → fires t ≈ 7.45 s, ~0.66 s past
apogee), routed through a solid-walled 12 mm PC-FR bypass tube past the *sealed* FC bay into the
recovery bay to release a friction-fit nose — **no RRC3+, no 9 V, no e-match/BP, no CO2, no FC
involvement**. Feasibility (`we4_ejection_feasibility.py`): tube loss ≈ 0.06 kPa; bay pressurizes to
~140 kPa vs a 14–41 kPa nose-release threshold = **3.4× margin**. F15-4 is the closest Estes delay
to the ~3.5 s coast optimum (F15-6/-8 eject 2.5 s/4.5 s late, too fast/low). Single passive event,
no electronic backup. Opening at ~6.5 m/s; 1/8″ Kevlar cord (> 800× margin) + Nomex protector; 18″
chute → ~6 m/s.

## 7. Ground Test Program
### 7.1 3-axis thrust-vector balance
Motor + gimbal on a thrust block restrained by one axial (5 kg) + two lateral (1 kg) strain-gauge
load cells → $T,\ \theta,\ \phi$. Actuator-agnostic — both magnetic and servo systems tested
identically. RQ3 metrics logged vs commanded.
### 7.2 Static thrust + materials stand
Axial cell + steel deflector: validates the F15-0 curve and screens jetvane materials in the plume (plugged 0-delay on the ground — no ejection into the fixture).
### 7.3 Motor & calibration plan
Load cells dead-weight calibrated, then commissioned with **6 × E16-4** (3 per stand). Counts: F15-4 ×4 (flight), F15-0 ×13 (ground), E16-4 ×6.

## 8. Wind Tunnel (RQ1/RQ2)
Hofferth modular STEM tunnel with the AC Infinity Cloudline A8 (724 CFM) for force-capable
test-section velocity; fin articles on the Gridfinity strut/sting and sidewall half-span mounts.

## 9. Safety & Regulatory
Single F15-4: 49.6 N·s, 60 g propellant, ≤ F class, liftoff 705 g < 1500 g → **FAA Class-1, no waiver,
no Level-1 certification**. Remote ignition, ≥ 3 m standoff on the stands, gimbal-neutral fail-safe,
motor-integral ejection (igniter installed at the pad; no electronic ejection circuit to arm or inhibit).

## 10. Budget
≈ **$1,882** total program spend (vehicle + 3-axis balance + static/materials stand + wind tunnel +
one-time tools + all motors): $1,403 still to buy + $479 already acquired, with live links in
`Documentation/WYVERN_E4_BOM.xlsx`. Per-flight consumable ≈ F15-4 $17 (integral delay/ejection; no
separate initiator or BP charge).

## 11. Schedule & Milestones
| Phase | Wk | Milestone |
|---|---|---|
| Print + assemble | 1–3 | airframe, gimbal, both stands |
| Bench bring-up | 4 | self-test all-PASS, control loop dry-run |
| TVC balance A/B (RQ3) | 5–6 | magnetic vs servo dataset |
| Static fires + jetvane (RQ2) | 7 | thrust-curve + material ranking |
| Wind tunnel (RQ1) | 8–9 | aerofoil polars |
| Flight tests (RQ4) | 10–11 | up to 4 flights, onboard logs |
| Analysis + paper | 12–14 | results, paper |

## 12. Risk Register
| Risk | Likelihood | Mitigation |
|---|---|---|
| Pre-TVC instability | Med | fins + ballast (+1.5 cal); TVC inhibit 0.5 s |
| Servo slew too slow | Med | bench-verify on balance before flight; fast digital micro |
| Hard 19 m/s deploy | Low | Kevlar > 60× margin, elastic leader, ground-tested charge |
| Launch-detect miss at low T/W | Low | tune arming alt; verify on 2.2 g spike |
| Camera/SD throughput | Low | self-contained i3 4K Thumb Action Camera (decoupled from FC) |

## 13. Expected Outcomes & Deliverables
A flight-validated small TVC vehicle; a quantitative magnetic-vs-servo TVC dataset; aerofoil polars
and jetvane material rankings; full open documentation (CAD, firmware, wiring, sims) and a research
paper.

## References
Hofferth, J. *Modular Wind Tunnel for STEM Education*, AIAA SCITECH 2025, doi:10.2514/6.2025-2560.
Barrowman, J. *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned
Vehicles*, NASA, 1967. Box, J. (BPS.space) *Signal/Echo* TVC flight series. NAR/Tripoli Model Rocket
Safety Codes. Bosch Sensortec BNO085 datasheet; Raspberry Pi RP2350 / Pico 2 W datasheet.
